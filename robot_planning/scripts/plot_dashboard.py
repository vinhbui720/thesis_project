#!/usr/bin/env python3
import os
import json
import pandas as pd
import numpy as np

import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QCheckBox
from PyQt5.QtCore import QTimer, Qt

import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
PKG_DIR = os.path.dirname(SCRIPT_DIR)
SUMMARY_CSV = os.path.join(PKG_DIR, 'tuning_results', 'trials_summary.csv')
PARAMS_CSV = os.path.join(PKG_DIR, 'tuning_results', 'params_history.csv')
STATE_JSON = os.path.join(PKG_DIR, 'tuning_results', 'optimizer_state.json')

class LiveDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Optuna Live Dashboard")
        self.setGeometry(100, 100, 1200, 800)
        
        self.advanced_mode = False
        
        # Central widget
        self.main_widget = QWidget(self)
        self.setCentralWidget(self.main_widget)
        self.layout = QVBoxLayout(self.main_widget)
        
        # Top Bar
        self.top_bar = QHBoxLayout()
        self.lbl_status = QLabel("Loading data...")
        self.lbl_status.setStyleSheet("font-size: 14px; font-weight: bold; color: #333;")
        self.cb_advanced = QCheckBox("Advanced Mode (Show All)")
        self.cb_advanced.stateChanged.connect(self.toggle_mode)
        
        self.top_bar.addWidget(self.lbl_status)
        self.top_bar.addStretch()
        self.top_bar.addWidget(self.cb_advanced)
        
        self.layout.addLayout(self.top_bar)
        
        # Canvas
        self.fig = Figure(figsize=(10, 8), tight_layout=True)
        self.canvas = FigureCanvas(self.fig)
        self.layout.addWidget(self.canvas)
        
        # Timer
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(2000)  # Refresh every 2 seconds
        
        self.update_plot()

    def toggle_mode(self, state):
        self.advanced_mode = state == Qt.Checked
        self.update_plot()

    def update_plot(self):
        # Read JSON state
        state_str = "Status: Waiting for Optimizer..."
        if os.path.exists(STATE_JSON):
            try:
                with open(STATE_JSON, 'r') as f:
                    st = json.load(f)
                fail_rate = st.get('fail_rate', 0.0) * 100
                state_str = f"Trials: {st.get('n_trials', 0)} | Fails: {st.get('n_fail', 0)} ({fail_rate:.1f}%) | Best Cost: {st.get('best_cost', 'N/A'):.4f} (Trial {st.get('best_trial', 'N/A')})"
            except Exception:
                pass
        self.lbl_status.setText(state_str)

        # Read CSVs
        if not os.path.exists(SUMMARY_CSV):
            return
            
        try:
            df = pd.read_csv(SUMMARY_CSV)
        except Exception:
            return

        if df.empty:
            return
            
        # Also read params if advanced
        df_p = None
        if self.advanced_mode and os.path.exists(PARAMS_CSV):
            try:
                df_p = pd.read_csv(PARAMS_CSV)
            except Exception:
                pass

        self.fig.clear()
        
        trials = df['trial']
        valid_mask = ~df['safety_fail']
        
        if self.advanced_mode:
            axs = self.fig.subplots(2, 3)
            axs = axs.flatten()
        else:
            axs = self.fig.subplots(1, 3)

        # 1. Cost Plot
        ax = axs[0]
        ax.scatter(trials[valid_mask], df.loc[valid_mask, 'cost'], c='blue', alpha=0.6, label='Cost')
        ax.scatter(trials[~valid_mask], df.loc[~valid_mask, 'cost'], c='red', alpha=0.6, marker='x', label='Fail')
        if 'best_cost_so_far' in df:
            ax.plot(trials, df['best_cost_so_far'], c='green', lw=2, label='Best Cost')
        ax.set_title('Convergence')
        ax.set_xlabel('Trial')
        ax.set_yscale('log')
        ax.legend()
        ax.grid(True, alpha=0.3)

        # 2. Safety & Exploration
        ax = axs[1]
        if 'fail_rate_so_far' in df:
            ax.plot(trials, df['fail_rate_so_far'], c='red', label='Fail Rate')
        if 'exploration_distance' in df:
            ax.plot(trials, df['exploration_distance'], c='orange', alpha=0.7, label='Exploration Dist')
        ax.set_title('Safety & Exploration')
        ax.set_xlabel('Trial')
        ax.legend()
        ax.grid(True, alpha=0.3)

        # 3. Component Costs
        ax = axs[2]
        if 'J_approach' in df:
            ax.plot(trials[valid_mask], df.loc[valid_mask, 'J_approach'], label='Approach')
            ax.plot(trials[valid_mask], df.loc[valid_mask, 'J_near'], label='Near')
            ax.plot(trials[valid_mask], df.loc[valid_mask, 'J_tracking'], label='Tracking')
        ax.set_title('Cost Components')
        ax.set_xlabel('Trial')
        ax.set_yscale('log')
        ax.legend()
        ax.grid(True, alpha=0.3)

        if self.advanced_mode:
            # 4. Tracking Metrics
            ax = axs[3]
            if 'rmse' in df:
                ax.plot(trials[valid_mask], df.loc[valid_mask, 'rmse'], label='RMSE (m)')
            if 'overshoot' in df:
                ax.plot(trials[valid_mask], df.loc[valid_mask, 'overshoot'], label='Overshoot (m)')
            if 'arrival_jitter' in df:
                ax.plot(trials[valid_mask], df.loc[valid_mask, 'arrival_jitter'], label='Jitter (m)')
            ax.set_title('Tracking Accuracy')
            ax.set_xlabel('Trial')
            ax.legend()
            ax.grid(True, alpha=0.3)

            # 5. Time Metrics
            ax = axs[4]
            if 'rise_time' in df:
                ax.plot(trials[valid_mask], df.loc[valid_mask, 'rise_time'], label='Rise Time (s)')
            if 'settling_time' in df:
                ax.plot(trials[valid_mask], df.loc[valid_mask, 'settling_time'], label='Settling (s)')
            if 'ramp_lag' in df:
                ax.plot(trials[valid_mask], df.loc[valid_mask, 'ramp_lag'], label='Ramp Lag (m)')
            ax.set_title('Time Metrics')
            ax.set_xlabel('Trial')
            ax.legend()
            ax.grid(True, alpha=0.3)

            # 6. Params History
            ax = axs[5]
            if df_p is not None and not df_p.empty:
                cols_to_plot = [c for c in df_p.columns if c != 'trial']
                # Normalize params roughly for visualization
                for c in cols_to_plot:
                    val = df_p[c]
                    max_v = val.max() if val.max() > 0 else 1.0
                    ax.plot(df_p['trial'], val / max_v, label=c, alpha=0.6)
                ax.set_title('Normalized Params')
                ax.set_xlabel('Trial')
                # Too many legends, place outside or small
                ax.legend(fontsize='x-small', loc='upper right')
            ax.grid(True, alpha=0.3)

        self.canvas.draw()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = LiveDashboard()
    ex.show()
    sys.exit(app.exec_())
