#!/usr/bin/env python3
import argparse
import math
import re
import sys
import threading
import time
from collections import deque
from copy import deepcopy

import rclpy
from rcl_interfaces.msg import Log
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtWidgets import (
    QApplication,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

import matplotlib

matplotlib.use("Qt5Agg")
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


FLOAT_RE = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:e[-+]?\d+)?"
OSQP_ITER_RE = re.compile(
    rf"^\s*(\d+)\s+({FLOAT_RE})\s+({FLOAT_RE})\s+({FLOAT_RE})\s+({FLOAT_RE})\s+({FLOAT_RE})\s+([^\s]+)\s+({FLOAT_RE})s\s*$",
    re.IGNORECASE,
)
OSQP_POLISH_RE = re.compile(
    rf"^\s*plsh\s+({FLOAT_RE})\s+({FLOAT_RE})\s+({FLOAT_RE})\s+({FLOAT_RE})\s+({FLOAT_RE})\s+([^\s]+)\s+({FLOAT_RE})s\s*$",
    re.IGNORECASE,
)
PROBLEM_RE = re.compile(r"variables n = (\d+), constraints m = (\d+)")
BOX_RE = re.compile(r"Iteration \(Box Size:\s*([^)]+)\)")
OVERALL_RE = re.compile(
    r"Overall:\s*(\d+)\s*\|\s*Convexify:\s*(\d+)\s*\|\s*Trust Region:\s*(\d+)\s*\|\s*Penalty:\s*(\d+)"
)
STATUS_RE = re.compile(r"status:\s*(.+)")
NUM_ITER_RE = re.compile(r"number of iterations:\s*(\d+)")
RUN_TIME_RE = re.compile(r"run time:\s*(" + FLOAT_RE + r")s", re.IGNORECASE)
RHO_EST_RE = re.compile(r"optimal rho estimate:\s*(" + FLOAT_RE + r")", re.IGNORECASE)
RHO_RE = re.compile(r"rho =\s*(" + FLOAT_RE + r")", re.IGNORECASE)
EPS_RE = re.compile(r"eps_abs =\s*(" + FLOAT_RE + r"), eps_rel =\s*(" + FLOAT_RE + r")", re.IGNORECASE)
RUN_CHUNK_RE = re.compile(r"\[Run\] Chunk (\d+)/(\d+) \(poses (\d+)-(\d+)\)")
RUN_ALL_RE = re.compile(r"\[Run\] All (\d+) chunks complete: (\d+) pts, (" + FLOAT_RE + r") s")
RUN_DONE_RE = re.compile(r"\[Run\] Chunk (\d+)/(\d+) done → (\d+) pts, (" + FLOAT_RE + r") s total")
TABLE_ROW_RE = re.compile(r"^\|.*\|$")


def safe_float(text):
    text = text.strip()
    if not text or text == "----------" or text == "------":
        return None
    try:
        return float(text)
    except ValueError:
        return None


def fmt_num(value, digits=3):
    if value is None:
        return "-"
    if value == 0:
        return "0"
    return f"{value:.{digits}e}"


def fmt_fixed(value, digits=4):
    if value is None:
        return "-"
    return f"{value:.{digits}f}"


class OptimizerDebugState:
    def __init__(self, max_osqp_points=5000, max_events=400):
        self._lock = threading.Lock()
        self.max_osqp_points = max_osqp_points
        self.max_events = max_events
        self.reset()

    def reset(self):
        self.node_name = ""
        self.last_msg_time = 0.0
        self.last_msg = ""
        self.total_rosout_msgs = 0
        self.accepted_rosout_msgs = 0
        self.run_summary = "Waiting for planner logs"
        self.chunk_summary = "-"
        self.final_summary = "-"
        self.latest_status = "-"
        self.latest_warning = "-"
        self.latest_convergence = "-"
        self.seed_status = "-"
        self.trust_region_event = "-"
        self.box_size = None
        self.overall_iter = None
        self.convexify_iter = None
        self.trust_region_iter = None
        self.penalty_iter = None
        self.osqp_problem = {}
        self.osqp_settings = {}
        self.osqp_iterations = []
        self.osqp_last = {}
        self.osqp_history = []
        self.cached_collision_used = 0
        self.cached_collision_missed = 0
        self.cost_rows = []
        self.constraint_rows = []
        self.sum_costs = None
        self.sum_constraints = None
        self.total_merit = None
        self.constraints_satisfied = None
        self.recent_events = deque(maxlen=self.max_events)
        self.current_table_section = None
        self.in_osqp_block = False

    def update_from_log(self, log: Log):
        stamp = log.stamp.sec + log.stamp.nanosec * 1e-9
        message = log.msg.strip("\n")
        if not message:
            return

        with self._lock:
            self.accepted_rosout_msgs += 1
            self.node_name = log.name or self.node_name
            self.last_msg_time = stamp or time.time()
            self.last_msg = message
            self._parse_message(log.level, message)

    def note_rosout_seen(self):
        with self._lock:
            self.total_rosout_msgs += 1

    def _parse_message(self, level, message):
        level_name = {
            Log.DEBUG: "DEBUG",
            Log.INFO: "INFO",
            Log.WARN: "WARN",
            Log.ERROR: "ERROR",
            Log.FATAL: "FATAL",
        }.get(level, "INFO")

        if "Seed Min Length Task Succeeded" in message:
            self.seed_status = "Seed min length task succeeded"
            self._push_event(level_name, message)
            return

        if "Using cached collision check" in message:
            self.cached_collision_used += 1
            return

        if "Not using cached collision check" in message:
            self.cached_collision_missed += 1
            return

        if "Expanded trust region" in message or "Shrunk trust region" in message:
            self.trust_region_event = message
            self._push_event(level_name, message)
            return

        if "Converged because improvement was small" in message:
            self.latest_convergence = message
            self._push_event(level_name, message)
            return

        if "constraints are satisfied" in message:
            self.latest_status = message
            self._push_event(level_name, message)
            return

        if "post-plan contact check flagged residual contact" in message:
            self.latest_warning = message
            self._push_event(level_name, message)
            return

        run_chunk = RUN_CHUNK_RE.search(message)
        if run_chunk:
            chunk_idx, chunk_total, pose_start, pose_end = run_chunk.groups()
            self.chunk_summary = f"Chunk {chunk_idx}/{chunk_total} | poses {pose_start}-{pose_end}"
            self.run_summary = message
            self._push_event(level_name, message)
            return

        run_done = RUN_DONE_RE.search(message)
        if run_done:
            chunk_idx, chunk_total, pts, total_time = run_done.groups()
            self.final_summary = f"Chunk {chunk_idx}/{chunk_total} done | {pts} pts | {total_time}s"
            self._push_event(level_name, message)
            return

        run_all = RUN_ALL_RE.search(message)
        if run_all:
            chunk_total, pts, total_time = run_all.groups()
            self.final_summary = f"All {chunk_total} chunks complete | {pts} pts | {total_time}s"
            self._push_event(level_name, message)
            return

        if "OSQP v" in message:
            self.in_osqp_block = True
            self.osqp_iterations = []
            self.osqp_last = {}
            self.osqp_problem = {}
            self.osqp_settings = {}
            self._push_event(level_name, "OSQP solve started")
            return

        if self.in_osqp_block:
            problem = PROBLEM_RE.search(message)
            if problem:
                self.osqp_problem["n_vars"] = int(problem.group(1))
                self.osqp_problem["n_constraints"] = int(problem.group(2))
                return

            eps = EPS_RE.search(message)
            if eps:
                self.osqp_settings["eps_abs"] = float(eps.group(1))
                self.osqp_settings["eps_rel"] = float(eps.group(2))
                return

            rho = RHO_RE.search(message)
            if rho and "adaptive" in message:
                self.osqp_settings["rho"] = float(rho.group(1))
                return

            iter_match = OSQP_ITER_RE.match(message)
            if iter_match:
                row = {
                    "iter": int(iter_match.group(1)),
                    "objective": float(iter_match.group(2)),
                    "prim_res": float(iter_match.group(3)),
                    "dual_res": float(iter_match.group(4)),
                    "gap": float(iter_match.group(5)),
                    "rel_kkt": float(iter_match.group(6)),
                    "rho": safe_float(iter_match.group(7).replace("*", "")),
                    "time": float(iter_match.group(8)),
                }
                self.osqp_iterations.append(row)
                self.osqp_iterations = self.osqp_iterations[-self.max_osqp_points :]
                self.osqp_last = row
                return

            polish_match = OSQP_POLISH_RE.match(message)
            if polish_match:
                self.osqp_last = {
                    "iter": "plsh",
                    "objective": float(polish_match.group(1)),
                    "prim_res": float(polish_match.group(2)),
                    "dual_res": float(polish_match.group(3)),
                    "gap": float(polish_match.group(4)),
                    "rel_kkt": float(polish_match.group(5)),
                    "rho": None,
                    "time": float(polish_match.group(7)),
                }
                return

            status = STATUS_RE.search(message)
            if status:
                self.latest_status = status.group(1).strip()
                return

            num_iter = NUM_ITER_RE.search(message)
            if num_iter:
                self.osqp_problem["num_iterations"] = int(num_iter.group(1))
                return

            run_time = RUN_TIME_RE.search(message)
            if run_time:
                self.osqp_problem["run_time"] = float(run_time.group(1))
                return

            rho_est = RHO_EST_RE.search(message)
            if rho_est:
                self.osqp_problem["rho_estimate"] = float(rho_est.group(1))
                self.osqp_history.append(
                    {
                        "timestamp": self.last_msg_time,
                        "status": self.latest_status,
                        "iterations": self.osqp_problem.get("num_iterations"),
                        "run_time": self.osqp_problem.get("run_time"),
                        "objective": self.osqp_last.get("objective"),
                        "prim_res": self.osqp_last.get("prim_res"),
                        "dual_res": self.osqp_last.get("dual_res"),
                        "gap": self.osqp_last.get("gap"),
                        "box_size": self.box_size,
                    }
                )
                self.osqp_history = self.osqp_history[-200:]
                self.in_osqp_block = False
                self._push_event(
                    level_name,
                    f"OSQP finished | status={self.latest_status} | iter={self.osqp_problem.get('num_iterations', '-')}"
                    f" | runtime={fmt_fixed(self.osqp_problem.get('run_time'), 4)}s",
                )
                return

        box = BOX_RE.search(message)
        if box:
            self.box_size = safe_float(box.group(1))
            self.cost_rows = []
            self.constraint_rows = []
            self.sum_costs = None
            self.sum_constraints = None
            self.total_merit = None
            self.constraints_satisfied = None
            self.current_table_section = None
            self._push_event(level_name, f"TrajOpt iteration started | box size={box.group(1).strip()}")
            return

        overall = OVERALL_RE.search(message)
        if overall:
            self.overall_iter = int(overall.group(1))
            self.convexify_iter = int(overall.group(2))
            self.trust_region_iter = int(overall.group(3))
            self.penalty_iter = int(overall.group(4))
            return

        if "INDIVIDUAL COSTS" in message:
            self.current_table_section = "costs"
            return

        if "CONSTRAINTS" in message:
            self.current_table_section = "constraints"
            return

        if "SUM COSTS" in message:
            parsed = self._parse_table_row(message)
            if parsed:
                self.sum_costs = parsed["new_exact"]
            return

        if "SUM CONSTRAINTS" in message:
            parsed = self._parse_table_row(message)
            if parsed:
                self.sum_constraints = parsed["new_exact"]
                self.constraints_satisfied = "Satisfied (True)" in message
            return

        if "TOTAL = SUM COSTS + SUM CONSTRAINTS" in message:
            parsed = self._parse_table_row(message)
            if parsed:
                self.total_merit = parsed["oldexact"]
            return

        if TABLE_ROW_RE.match(message) and self.current_table_section in {"costs", "constraints"}:
            parsed = self._parse_table_row(message)
            if parsed and parsed["name"] and not parsed["name"].startswith("DiscreteCollision"):
                if self.current_table_section == "costs":
                    self.cost_rows.append(parsed)
                    self.cost_rows = sorted(
                        self.cost_rows,
                        key=lambda row: abs(row["new_exact"] if row["new_exact"] is not None else 0.0),
                        reverse=True,
                    )[:25]
                else:
                    self.constraint_rows.append(parsed)
                    self.constraint_rows = sorted(
                        self.constraint_rows,
                        key=lambda row: abs(row["oldexact"] if row["oldexact"] is not None else 0.0),
                        reverse=True,
                    )[:25]
            return

        if level >= Log.WARN or "process succeeded" in message or "[Run]" in message:
            self._push_event(level_name, message)

    def _parse_table_row(self, message):
        parts = [part.strip() for part in message.strip().strip("|").split("|")]
        if len(parts) < 8:
            return None
        return {
            "merit": safe_float(parts[0]),
            "oldexact": safe_float(parts[1]),
            "new_exact": safe_float(parts[2]),
            "new_approx": safe_float(parts[3]),
            "dapprox": safe_float(parts[4]),
            "dexact": safe_float(parts[5]),
            "ratio": safe_float(parts[6]),
            "name": parts[7],
        }

    def _push_event(self, level_name, message):
        ts = time.strftime("%H:%M:%S", time.localtime(self.last_msg_time or time.time()))
        self.recent_events.appendleft(f"[{ts}] {level_name}: {message}")

    def snapshot(self):
        with self._lock:
            return {
                "node_name": self.node_name,
                "last_msg_time": self.last_msg_time,
                "last_msg": self.last_msg,
                "total_rosout_msgs": self.total_rosout_msgs,
                "accepted_rosout_msgs": self.accepted_rosout_msgs,
                "run_summary": self.run_summary,
                "chunk_summary": self.chunk_summary,
                "final_summary": self.final_summary,
                "latest_status": self.latest_status,
                "latest_warning": self.latest_warning,
                "latest_convergence": self.latest_convergence,
                "seed_status": self.seed_status,
                "trust_region_event": self.trust_region_event,
                "box_size": self.box_size,
                "overall_iter": self.overall_iter,
                "convexify_iter": self.convexify_iter,
                "trust_region_iter": self.trust_region_iter,
                "penalty_iter": self.penalty_iter,
                "osqp_problem": deepcopy(self.osqp_problem),
                "osqp_settings": deepcopy(self.osqp_settings),
                "osqp_iterations": list(self.osqp_iterations),
                "osqp_last": deepcopy(self.osqp_last),
                "osqp_history": list(self.osqp_history),
                "cached_collision_used": self.cached_collision_used,
                "cached_collision_missed": self.cached_collision_missed,
                "cost_rows": list(self.cost_rows),
                "constraint_rows": list(self.constraint_rows),
                "sum_costs": self.sum_costs,
                "sum_constraints": self.sum_constraints,
                "total_merit": self.total_merit,
                "constraints_satisfied": self.constraints_satisfied,
                "recent_events": list(self.recent_events),
            }


class RosoutMonitor(Node):
    def __init__(self, state: OptimizerDebugState, node_filter: str):
        super().__init__("motomini_optimizer_debug_gui")
        self.state = state
        self.node_filter = node_filter
        self.filter_candidates = self._build_filter_candidates(node_filter)
        self.subscription = self.create_subscription(Log, "/rosout", self._callback, 200)

    def _callback(self, msg: Log):
        self.state.note_rosout_seen()
        if self._accept(msg):
            self.state.update_from_log(msg)

    def _accept(self, msg: Log):
        if not self.node_filter:
            return True
        haystacks = []
        if msg.name:
            haystacks.append(msg.name)
        if msg.msg:
            haystacks.append(msg.msg)
        if hasattr(msg, "file") and msg.file:
            haystacks.append(msg.file)
        for candidate in self.filter_candidates:
            for haystack in haystacks:
                if candidate and candidate in haystack:
                    return True
        return False

    def _build_filter_candidates(self, raw_filter: str):
        candidates = []
        if not raw_filter:
            return candidates
        candidates.append(raw_filter)
        no_launch_suffix = re.sub(r"-\d+$", "", raw_filter)
        if no_launch_suffix and no_launch_suffix not in candidates:
            candidates.append(no_launch_suffix)
        no_namespace = no_launch_suffix.split("/")[-1]
        if no_namespace and no_namespace not in candidates:
            candidates.append(no_namespace)
        generic = no_namespace.replace("_node", "")
        if generic and generic not in candidates:
            candidates.append(generic)
        return candidates


class RosThread(threading.Thread):
    def __init__(self, state: OptimizerDebugState, node_filter: str, ros_args):
        super().__init__(daemon=True)
        self.state = state
        self.node_filter = node_filter
        self.ros_args = ros_args
        self.executor = None
        self.node = None

    def run(self):
        rclpy.init(args=self.ros_args)
        self.node = RosoutMonitor(self.state, self.node_filter)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        try:
            self.executor.spin()
        finally:
            if self.executor is not None:
                self.executor.shutdown()
            if self.node is not None:
                self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    def stop(self):
        if self.executor is not None:
            self.executor.shutdown()


class OptimizerDebugWindow(QMainWindow):
    def __init__(self, state: OptimizerDebugState, node_filter: str):
        super().__init__()
        self.state = state
        self.node_filter = node_filter
        self.setWindowTitle(f"Motomini Optimizer Debug GUI | filter={node_filter}")
        self.resize(1600, 980)

        root = QWidget()
        self.setCentralWidget(root)
        main_layout = QVBoxLayout(root)

        top_grid = QGridLayout()
        self.labels = {}
        label_keys = [
            ("Node", "node"),
            ("Traffic", "traffic"),
            ("Run", "run"),
            ("Chunk", "chunk"),
            ("Planner", "planner"),
            ("Box Size", "box"),
            ("Iterations", "iters"),
            ("OSQP", "osqp"),
            ("Cache", "cache"),
            ("Costs", "costs"),
            ("Constraints", "constraints"),
            ("Convergence", "convergence"),
            ("Warning", "warning"),
        ]
        for idx, (title, key) in enumerate(label_keys):
            title_label = QLabel(f"{title}:")
            title_label.setStyleSheet("font-weight: bold;")
            value_label = QLabel("-")
            value_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
            top_grid.addWidget(title_label, idx // 2, (idx % 2) * 2)
            top_grid.addWidget(value_label, idx // 2, (idx % 2) * 2 + 1)
            self.labels[key] = value_label
        main_layout.addLayout(top_grid)

        splitter = QSplitter(Qt.Vertical)
        main_layout.addWidget(splitter, stretch=1)

        top_widget = QWidget()
        top_layout = QHBoxLayout(top_widget)
        self.figure = Figure(figsize=(12, 7), tight_layout=True)
        self.canvas = FigureCanvas(self.figure)
        top_layout.addWidget(self.canvas, stretch=3)
        splitter.addWidget(top_widget)

        bottom_splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(bottom_splitter)

        self.cost_table = self._create_table("Top Costs")
        self.constraint_table = self._create_table("Top Constraints")
        self.events_box = QTextEdit()
        self.events_box.setReadOnly(True)

        bottom_splitter.addWidget(self.cost_table["container"])
        bottom_splitter.addWidget(self.constraint_table["container"])
        bottom_splitter.addWidget(self._wrap_widget("Recent Events", self.events_box))

        splitter.setSizes([600, 320])
        bottom_splitter.setSizes([420, 420, 520])

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(400)
        self.refresh()

    def _create_table(self, title):
        table = QTableWidget(0, 4)
        table.setHorizontalHeaderLabels(["Name", "Old", "New", "Delta"])
        table.horizontalHeader().setStretchLastSection(True)
        table.setAlternatingRowColors(True)
        table.setSortingEnabled(False)
        return {"widget": table, "container": self._wrap_widget(title, table)}

    def _wrap_widget(self, title, widget):
        container = QWidget()
        layout = QVBoxLayout(container)
        label = QLabel(title)
        label.setStyleSheet("font-weight: bold;")
        layout.addWidget(label)
        layout.addWidget(widget)
        return container

    def refresh(self):
        snap = self.state.snapshot()
        self._update_labels(snap)
        self._update_plots(snap)
        self._update_table(self.cost_table["widget"], snap["cost_rows"])
        self._update_table(self.constraint_table["widget"], snap["constraint_rows"])
        self.events_box.setPlainText("\n".join(snap["recent_events"][:80]))

    def _update_labels(self, snap):
        node_name = snap["node_name"] or self.node_filter
        age = time.time() - snap["last_msg_time"] if snap["last_msg_time"] else math.inf
        age_text = f"{age:.1f}s ago" if math.isfinite(age) else "no logs yet"
        self.labels["node"].setText(f"{node_name} | last log {age_text}")
        self.labels["traffic"].setText(
            f"/rosout seen={snap['total_rosout_msgs']} | matched={snap['accepted_rosout_msgs']}"
        )
        self.labels["run"].setText(snap["run_summary"])
        self.labels["chunk"].setText(f"{snap['chunk_summary']} | {snap['final_summary']}")
        self.labels["planner"].setText(
            f"{snap['seed_status']} | status={snap['latest_status'] or '-'} | trust={snap['trust_region_event'] or '-'}"
        )
        self.labels["box"].setText(
            f"{fmt_fixed(snap['box_size'], 4)} | overall={snap['overall_iter'] or '-'}"
            f" convexify={snap['convexify_iter'] or '-'} trust={snap['trust_region_iter'] or '-'}"
            f" penalty={snap['penalty_iter'] or '-'}"
        )
        self.labels["iters"].setText(
            f"OSQP iters={snap['osqp_problem'].get('num_iterations', '-')} | "
            f"n={snap['osqp_problem'].get('n_vars', '-')} m={snap['osqp_problem'].get('n_constraints', '-')}"
        )
        self.labels["osqp"].setText(
            f"runtime={fmt_fixed(snap['osqp_problem'].get('run_time'), 4)}s | "
            f"rho={fmt_num(snap['osqp_settings'].get('rho'))} | "
            f"rho_est={fmt_num(snap['osqp_problem'].get('rho_estimate'))} | "
            f"eps_abs={fmt_num(snap['osqp_settings'].get('eps_abs'))} | "
            f"eps_rel={fmt_num(snap['osqp_settings'].get('eps_rel'))}"
        )
        total_cache = snap["cached_collision_used"] + snap["cached_collision_missed"]
        ratio = snap["cached_collision_used"] / total_cache if total_cache else 0.0
        self.labels["cache"].setText(
            f"used={snap['cached_collision_used']} miss={snap['cached_collision_missed']} reuse={ratio:.1%}"
        )
        self.labels["costs"].setText(
            f"sum_costs={fmt_num(snap['sum_costs'])} | total_merit={fmt_num(snap['total_merit'])}"
        )
        satisfied = snap["constraints_satisfied"]
        self.labels["constraints"].setText(
            f"sum_constraints={fmt_num(snap['sum_constraints'])} | satisfied={satisfied if satisfied is not None else '-'}"
        )
        self.labels["convergence"].setText(snap["latest_convergence"] or "-")
        self.labels["warning"].setText(snap["latest_warning"] or "-")

    def _update_plots(self, snap):
        self.figure.clear()
        axs = self.figure.subplots(2, 2)

        iter_rows = snap["osqp_iterations"]
        if iter_rows:
            xs = [row["iter"] for row in iter_rows]
            obj = [abs(row["objective"]) for row in iter_rows]
            prim = [max(abs(row["prim_res"]), 1e-18) for row in iter_rows]
            dual = [max(abs(row["dual_res"]), 1e-18) for row in iter_rows]
            gap = [max(abs(row["gap"]), 1e-18) for row in iter_rows]

            axs[0, 0].plot(xs, obj, color="#0b84a5", linewidth=1.6, label="|objective|")
            axs[0, 0].set_title("OSQP Objective")
            axs[0, 0].set_xlabel("Iteration")
            axs[0, 0].set_yscale("log")
            axs[0, 0].grid(True, alpha=0.3)

            axs[0, 1].plot(xs, prim, color="#f6c85f", linewidth=1.6, label="primal residual")
            axs[0, 1].plot(xs, dual, color="#6f4e7c", linewidth=1.6, label="dual residual")
            axs[0, 1].plot(xs, gap, color="#9dd866", linewidth=1.6, label="gap")
            axs[0, 1].set_title("Residuals And Gap")
            axs[0, 1].set_xlabel("Iteration")
            axs[0, 1].set_yscale("log")
            axs[0, 1].grid(True, alpha=0.3)
            axs[0, 1].legend(fontsize="small")
        else:
            axs[0, 0].text(0.5, 0.5, "Waiting for OSQP rows", ha="center", va="center")
            axs[0, 1].text(0.5, 0.5, "Waiting for residuals", ha="center", va="center")

        history = snap["osqp_history"]
        if history:
            solve_idx = list(range(1, len(history) + 1))
            runtimes = [max(item.get("run_time") or 0.0, 1e-8) for item in history]
            final_prim = [max(abs(item.get("prim_res") or 0.0), 1e-18) for item in history]
            axs[1, 0].plot(solve_idx, runtimes, marker="o", color="#e45756", linewidth=1.4, label="runtime (s)")
            axs[1, 0].set_title("Solve Runtime History")
            axs[1, 0].set_xlabel("Solve Index")
            axs[1, 0].grid(True, alpha=0.3)
            ax2 = axs[1, 0].twinx()
            ax2.plot(solve_idx, final_prim, marker="x", color="#2ca02c", linewidth=1.2, label="final primal residual")
            ax2.set_yscale("log")
            ax2.set_ylabel("Final Prim Residual")
        else:
            axs[1, 0].text(0.5, 0.5, "Waiting for completed solves", ha="center", va="center")

        used = snap["cached_collision_used"]
        missed = snap["cached_collision_missed"]
        axs[1, 1].bar(["cache hit", "cache miss"], [used, missed], color=["#4caf50", "#ff7043"])
        axs[1, 1].set_title("Collision Cache Usage")
        axs[1, 1].set_ylabel("Count")
        axs[1, 1].grid(True, axis="y", alpha=0.3)

        self.canvas.draw()

    def _update_table(self, table: QTableWidget, rows):
        table.setRowCount(len(rows[:15]))
        for r, row in enumerate(rows[:15]):
            values = [
                row["name"],
                fmt_num(row["oldexact"]),
                fmt_num(row["new_exact"]),
                fmt_num(row["dexact"]),
            ]
            for c, value in enumerate(values):
                item = QTableWidgetItem(value)
                if c > 0:
                    item.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
                table.setItem(r, c, item)
        table.resizeColumnsToContents()


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Live GUI for Motomini TrajOpt/OSQP logs from /rosout",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--node-filter",
        default="motomini_planning_node",
        help="Substring matched against /rosout Log.name and Log.msg",
    )
    return parser.parse_known_args(argv)


def main(argv=None):
    args, ros_args = parse_args(argv if argv is not None else sys.argv[1:])

    state = OptimizerDebugState()
    ros_thread = RosThread(state, args.node_filter, ros_args)
    ros_thread.start()

    app = QApplication(sys.argv[:1])
    window = OptimizerDebugWindow(state, args.node_filter)
    window.show()

    exit_code = 0
    try:
        exit_code = app.exec_()
    finally:
        ros_thread.stop()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
