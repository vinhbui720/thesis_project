#!/usr/bin/env python3
import os
import time
import yaml
import json
import signal
import subprocess
import pandas as pd
import optuna
import numpy as np
import argparse

# Resolve paths relative to this script
SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
PKG_DIR = os.path.dirname(SCRIPT_DIR)
BASE_YAML = os.path.join(PKG_DIR, 'config', 'collision_opt_base.yaml')
TRIAL_YAML = os.path.join(PKG_DIR, 'config', 'collision_trial.yaml')
SUMMARY_CSV = os.path.join(PKG_DIR, 'tuning_results', 'collision_trials_summary.csv')
PARAMS_CSV = os.path.join(PKG_DIR, 'tuning_results', 'collision_params_history.csv')
STATE_JSON = os.path.join(PKG_DIR, 'tuning_results', 'collision_optimizer_state.json')

PHASE_1_SPACE = {
    'collision_normal_force_max': (2.0, 10.0),
    'collision_normal_damping': (2.0, 25.0),
    'collision_force_max_per_contact': (2.0, 10.0),
    'collision_force_attack_hz': (8.0, 35.0),
    'collision_force_release_hz': (3.0, 20.0),
    'collision_force_slew_rate': (20.0, 150.0),
}

PHASE_2_SPACE = {
    'collision_tangent_gain': (0.5, 10.0),
    'collision_tangent_force_max': (1.0, 8.0),
    'collision_tangent_force_ratio': (0.15, 0.60),
    'collision_tangent_gamma_power': (1.0, 3.5),
    'collision_tangent_speed_scale': (0.02, 0.15),
    'collision_tangent_velocity_deadband': (0.003, 0.03),
}

PHASE_3_SPACE = {
    'collision_force_release_hz': (4.0, 25.0),
    'collision_force_slew_rate': (20.0, 150.0),
    'collision_normal_fade_power': (0.7, 3.0),
    'collision_force_scale': (0.3, 1.5),
    'collision_force_max': (2.0, 8.0),
    'collision_constraint_timeout_sec': (0.08, 0.35),
}

DISTANCE_SPACE = {
    'collision_influence_distance': (0.02, 0.08),
    'collision_safe_distance': (0.01, 0.04),
    'collision_guard_distance': (0.005, 0.03),
    'collision_task_distance': (0.002, 0.015),
}

GLOBAL_PHASE = 'all'
PARAM_BOUNDS = {}

OPT_STATE = {
    'best_cost': float('inf'),
    'best_trial': None,
    'best_params': None,
    'n_trials': 0,
    'n_fail': 0,
    'prev_x': None,
}

def normalize_params(params, bounds):
    x = []
    for k, (lo, hi) in bounds.items():
        if k in params:
            x.append((float(params[k]) - lo) / max(hi - lo, 1e-12))
    return np.array(x, dtype=float)

def load_yaml(path):
    if not os.path.exists(path):
        return {'online_collision_debugger': {'ros__parameters': {}}}
    with open(path, 'r') as f:
        return yaml.safe_load(f)

def write_trial_yaml(base_path, trial_path, params):
    data = load_yaml(base_path)
    if 'online_collision_debugger' not in data:
        data['online_collision_debugger'] = {'ros__parameters': {}}
    ros_params = data['online_collision_debugger']['ros__parameters']
    
    for k, v in params.items():
        ros_params[k] = float(v) if isinstance(v, (int, float)) else v
        
    # Enable tangent force if we are tuning Phase 2, 3, 4, or All
    if GLOBAL_PHASE in ['2', '3', '4', 'all']:
        ros_params['collision_tangent_enabled'] = True
    else:
        ros_params['collision_tangent_enabled'] = False
        
    with open(trial_path, 'w') as f:
        yaml.safe_dump(data, f, sort_keys=False)

def launch_controller(param_file):
    feedback_yaml = os.path.join(PKG_DIR, 'config', 'feedback_controller.yaml')
    cmd = [
        'ros2', 'launch', 'motomini', 'motomini.launch.py',
        'real_robot:=false', 'debug:=true', 'vel_streaming:=true', 'jogging:=true',
        'bayesian:=true', 
        f'collision_yaml_file:={os.path.abspath(param_file)}',
        f'feedback_yaml_file:={feedback_yaml}'
    ]
    return subprocess.Popen(cmd, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def stop_process(proc):
    if proc is None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=5.0)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass

def run_one_trial(params, trial_index):
    trial_start = time.time()
    write_trial_yaml(BASE_YAML, TRIAL_YAML, params)
    print(f"--- Starting Collision Trial {trial_index} (Phase: {GLOBAL_PHASE}) ---")

    proc = launch_controller(TRIAL_YAML)
    try:
        time.sleep(10.0)

        subprocess.run(['ros2', 'service', 'call', '/pose_following/stop', 'std_srvs/srv/Trigger', '{}'], timeout=5)
        time.sleep(0.5)
        subprocess.run(['ros2', 'service', 'call', '/pose_following/start', 'std_srvs/srv/Trigger', '{}'], timeout=5)
        time.sleep(0.5)

        cmd = [
            'python3', os.path.join(SCRIPT_DIR, 'run_collision_trial.py'),
            '--ros-args', '-p', f'trial_index:={trial_index}', '-p', 'duration:=15.0'
        ]
        out = subprocess.check_output(cmd, text=True, timeout=40.0)
        
        metrics_line = out.strip().splitlines()[-1]
        try:
            metrics = json.loads(metrics_line)
        except json.JSONDecodeError:
            metrics = {'error': 'Failed to parse JSON', 'safety_fail': True, 'raw_out': out}
            print(f"JSON Parse Error. Output was: {out}")

        cost = compute_cost(metrics)
        trial_duration_sec = time.time() - trial_start
        save_trial(trial_index, params, metrics, cost, trial_duration_sec)
        return cost

    except Exception as e:
        metrics = {'error': str(e), 'safety_fail': True}
        cost = 1e9
        trial_duration_sec = time.time() - trial_start
        save_trial(trial_index, params, metrics, cost, trial_duration_sec)
        return cost
    finally:
        stop_process(proc)
        time.sleep(2.0)

def compute_cost(m):
    if m.get('safety_fail', False):
        return 1e9

    min_distance = float(m.get('min_distance', 0.1))
    force_jitter = float(m.get('force_jitter', 1.0))
    velocity_spike = float(m.get('velocity_spike', 1.0))
    penetration_depth = float(m.get('penetration_depth', 1.0))
    max_force = float(m.get('max_force', 0.0))
    progress_error = float(m.get('progress_error', 1.0)) 
    arrival_jitter = float(m.get('arrival_jitter', 1.0))
    final_error = float(m.get('final_error', 1.0))
    error_at_stop = float(m.get('error_at_stop', 1.0))
    dynamic_tracking_error = float(m.get('dynamic_tracking_error', 1.0))

    # Cost function ưu tiên:
    # 1. Tới được đích (progress_error tiến về 0) => Vượt qua vật cản
    # 2. Bắt kịp target TRƯỚC khi target dừng (error_at_stop)
    # 3. Không xuyên vật cản sâu (penetration)
    # 4. Chống nhảy qua nhảy lại sau khi né (arrival_jitter, final_error)
    # 5. Lực tác động và vận tốc mượt mà (jitter, spike)
    cost = (
        1000.0 * progress_error / 0.1
        + 800.0 * error_at_stop / 0.01
        + 100.0 * penetration_depth / 0.01
        + 50.0 * force_jitter / 1.0
        + 20.0 * velocity_spike / 0.1
        + 300.0 * arrival_jitter / 0.002
        + 100.0 * final_error / 0.01
        + 50.0 * dynamic_tracking_error / 0.05
    )

    if progress_error > 0.05:
        cost += 1e5 + progress_error * 1e6
        
    if error_at_stop > 0.02:
        cost += 1e4 + (error_at_stop - 0.02) * 1e5
        
    if penetration_depth > 0.015:
        cost += 1e6 + penetration_depth * 1e8

    if force_jitter > 2.0:
        cost += 1e4 + (force_jitter - 2.0) * 1e5
        
    if arrival_jitter > 0.005:
        cost += 1e4 + (arrival_jitter - 0.005) * 1e6

    return float(cost)

def save_trial(trial_index, params, metrics, cost, trial_duration_sec=0.0):
    os.makedirs(os.path.dirname(SUMMARY_CSV), exist_ok=True)
    OPT_STATE['n_trials'] += 1
    if metrics.get('safety_fail', False):
        OPT_STATE['n_fail'] += 1

    prev_best = OPT_STATE['best_cost']
    is_best = cost < prev_best

    if is_best:
        OPT_STATE['best_cost'] = float(cost)
        OPT_STATE['best_trial'] = int(trial_index)
        OPT_STATE['best_params'] = dict(params)

    x_now = normalize_params(params, PARAM_BOUNDS)
    exploration_distance = 0.0 if OPT_STATE['prev_x'] is None else float(np.linalg.norm(x_now - OPT_STATE['prev_x']))
    OPT_STATE['prev_x'] = x_now
    fail_rate = OPT_STATE['n_fail'] / max(OPT_STATE['n_trials'], 1)

    summary_row = {
        'trial': trial_index,
        'cost': float(cost),
        'best_cost_so_far': float(OPT_STATE['best_cost']),
        'is_best': bool(is_best),
        'safety_fail': bool(metrics.get('safety_fail', False)),
        'fail_reason': metrics.get('error', ''),
        'min_distance': metrics.get('min_distance', np.nan),
        'penetration_depth': metrics.get('penetration_depth', np.nan),
        'force_jitter': metrics.get('force_jitter', np.nan),
        'velocity_spike': metrics.get('velocity_spike', np.nan),
        'max_force': metrics.get('max_force', np.nan),
        'progress_error': metrics.get('progress_error', np.nan),
        'arrival_jitter': metrics.get('arrival_jitter', np.nan),
        'final_error': metrics.get('final_error', np.nan),
        'error_at_stop': metrics.get('error_at_stop', np.nan),
        'dynamic_tracking_error': metrics.get('dynamic_tracking_error', np.nan),
    }

    pd.DataFrame([summary_row]).to_csv(SUMMARY_CSV, mode='a', index=False, header=not os.path.exists(SUMMARY_CSV))
    
    params_row = {'trial': trial_index}
    params_row.update(params)
    pd.DataFrame([params_row]).to_csv(PARAMS_CSV, mode='a', index=False, header=not os.path.exists(PARAMS_CSV))

def objective(trial):
    params = {}
    for param_name, (low, high) in PARAM_BOUNDS.items():
        params[param_name] = trial.suggest_float(param_name, low, high)

    # Reject invalid distance constraints if we are tuning distances
    if 'collision_influence_distance' in params:
        inf = params['collision_influence_distance']
        saf = params['collision_safe_distance']
        grd = params['collision_guard_distance']
        tsk = params['collision_task_distance']
        if not (inf > saf and saf > grd and grd > tsk):
            return 1e9

    cost = run_one_trial(params, trial.number)
    return cost

def optuna_callback(study, trial):
    print(f"[Optuna] trial={trial.number} value={trial.value:.6f} best={study.best_value:.6f}")

def main():
    global GLOBAL_PHASE, PARAM_BOUNDS
    parser = argparse.ArgumentParser()
    parser.add_argument('--n-trials', type=int, default=50)
    parser.add_argument('--phase', type=str, choices=['1', '2', '3', '4', 'all'], default='all', 
                        help='Phase to tune (1: Normal, 2: Tangent, 3: Release, 4: Distances, all: All combined)')
    args = parser.parse_args()
    
    GLOBAL_PHASE = args.phase
    
    if args.phase == '1':
        PARAM_BOUNDS = PHASE_1_SPACE
    elif args.phase == '2':
        PARAM_BOUNDS = PHASE_2_SPACE
    elif args.phase == '3':
        PARAM_BOUNDS = PHASE_3_SPACE
    elif args.phase == '4':
        PARAM_BOUNDS = DISTANCE_SPACE
    else:
        # All phases
        PARAM_BOUNDS = {**PHASE_1_SPACE, **PHASE_2_SPACE, **PHASE_3_SPACE, **DISTANCE_SPACE}

    print(f"Starting Bayesian Optimization for Collision Phase: {args.phase}")
    print(f"Parameters to tune: {list(PARAM_BOUNDS.keys())}")

    sampler = optuna.samplers.TPESampler(n_startup_trials=10, multivariate=True, group=True, seed=0)
    study = optuna.create_study(direction='minimize', sampler=sampler)
    study.optimize(objective, n_trials=args.n_trials, callbacks=[optuna_callback])

    study.trials_dataframe().to_csv(os.path.join(PKG_DIR, 'tuning_results', 'optuna_collision_trials.csv'), index=False)

    print("Best trial:")
    trial = study.best_trial
    print(f"  Value: {trial.value}")
    print("  Params: ")
    for key, value in trial.params.items():
        print(f"    {key}: {value}")
        
    with open(os.path.join(PKG_DIR, 'tuning_results', 'best_collision_params.yaml'), 'w') as f:
        yaml.safe_dump(trial.params, f)

if __name__ == '__main__':
    main()
