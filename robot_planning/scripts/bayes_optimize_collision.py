#!/usr/bin/env python3
import os
import datetime
import time
import yaml
import json
import signal
import subprocess
import pandas as pd
import optuna
from optuna.trial import FrozenTrial, TrialState
import optuna.distributions
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

PARAM_BOUNDS = {
    'collision_k_rep': (0.00005, 0.0005),
    'collision_normal_damping': (2.0, 25.0),
    'collision_tangent_gain': (1.0, 15.0),
    'collision_tangent_damping': (0.1, 5.0),
}

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

def write_trial_yaml(trial_path, params):
    data = load_yaml(trial_path)
    if data is None:
        data = {'online_collision_debugger': {'ros__parameters': {}}}
    if 'online_collision_debugger' not in data:
        data['online_collision_debugger'] = {'ros__parameters': {}}
    if 'ros__parameters' not in data['online_collision_debugger']:
        data['online_collision_debugger']['ros__parameters'] = {}
        
    ros_params = data['online_collision_debugger']['ros__parameters']
    
    for k, v in params.items():
        ros_params[k] = float(v) if isinstance(v, (int, float)) else v
        
    ros_params['collision_tangent_enabled'] = True
        
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
    write_trial_yaml(TRIAL_YAML, params)
    print(f"--- Starting Collision Trial {trial_index} ---")

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
    if OPT_STATE['prev_x'] is None or len(OPT_STATE['prev_x']) != len(x_now):
        exploration_distance = 0.0
    else:
        exploration_distance = float(np.linalg.norm(x_now - OPT_STATE['prev_x']))
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

    with open(STATE_JSON, 'w') as f:
        json.dump({
            'n_trials': OPT_STATE['n_trials'],
            'n_fail': OPT_STATE['n_fail'],
            'fail_rate': fail_rate,
            'best_trial': OPT_STATE['best_trial'],
            'best_cost': OPT_STATE['best_cost'],
            'best_params': OPT_STATE['best_params'],
        }, f, indent=2)

def objective(trial):
    params = {}
    for param_name, (low, high) in PARAM_BOUNDS.items():
        params[param_name] = trial.suggest_float(param_name, low, high)

    cost = run_one_trial(params, trial.number)
    return cost

def optuna_callback(study, trial):
    print(f"[Optuna] trial={trial.number} value={trial.value:.6f} best={study.best_value:.6f}")

def load_past_trials(study: optuna.Study) -> int:
    if not os.path.exists(PARAMS_CSV) or not os.path.exists(SUMMARY_CSV):
        print("[Resume] No existing trial data found — starting fresh.")
        return 0

    params_df = pd.read_csv(PARAMS_CSV)
    summary_df = pd.read_csv(SUMMARY_CSV)

    summary_sub = summary_df[['trial', 'cost', 'safety_fail']].copy()
    merged = pd.merge(params_df, summary_sub, on='trial', how='inner')
    merged = merged.dropna(subset=['cost'])

    if merged.empty:
        print("[Resume] CSV files exist but contain no usable rows — starting fresh.")
        return 0

    distributions = {
        k: optuna.distributions.FloatDistribution(lo, hi)
        for k, (lo, hi) in PARAM_BOUNDS.items()
    }

    n_loaded = 0
    for _, row in merged.iterrows():
        params = {
            k: float(row[k])
            for k in PARAM_BOUNDS
            if k in row and pd.notna(row[k])
        }
        if len(params) != len(PARAM_BOUNDS):
            continue

        cost = float(row['cost'])

        _ts = datetime.datetime(2000, 1, 1) + datetime.timedelta(seconds=n_loaded)
        frozen = FrozenTrial(
            number=n_loaded,
            trial_id=n_loaded,
            state=TrialState.COMPLETE,
            value=cost,
            values=None,
            datetime_start=_ts,
            datetime_complete=_ts,
            params=params,
            distributions=distributions,
            user_attrs={},
            system_attrs={},
            intermediate_values={},
        )
        study.add_trial(frozen)
        n_loaded += 1

    print(f"[Resume] Loaded {n_loaded} past trials into the study (TPE warm-start active).")
    return n_loaded

def restore_opt_state() -> None:
    if not os.path.exists(STATE_JSON):
        return
    try:
        with open(STATE_JSON, 'r') as f:
            saved = json.load(f)
        OPT_STATE['best_cost']   = float(saved.get('best_cost',   float('inf')))
        OPT_STATE['best_trial']  = saved.get('best_trial',  None)
        OPT_STATE['best_params'] = saved.get('best_params', None)
        OPT_STATE['n_trials']    = int(saved.get('n_trials', 0))
        OPT_STATE['n_fail']      = int(saved.get('n_fail',   0))
        print(f"[Resume] Restored OPT_STATE: best_cost={OPT_STATE['best_cost']:.6f}, "
              f"n_trials={OPT_STATE['n_trials']}, n_fail={OPT_STATE['n_fail']}")
    except Exception as e:
        print(f"[Resume] Could not restore optimizer_state.json: {e}")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--n-trials', type=int, default=50, help='Number of NEW trials to run (additional when --resume).')
    parser.add_argument('--resume', action='store_true', help='Warm-start from existing CSV logs instead of starting fresh.')
    args = parser.parse_args()
    
    print(f"Starting Bayesian Optimization for Collision Avoidance")
    print(f"Parameters to tune: {list(PARAM_BOUNDS.keys())}")

    sampler = optuna.samplers.TPESampler(n_startup_trials=10, multivariate=True, group=True, seed=0)
    study = optuna.create_study(direction='minimize', sampler=sampler)

    if args.resume:
        print("[Resume] Resume mode enabled — loading past trials …")
        restore_opt_state()
        n_loaded = load_past_trials(study)
        if n_loaded == 0:
            print("[Resume] No past trials found; running full search.")
        else:
            print(f"[Resume] Study now has {len(study.trials)} warm-start trials. Running {args.n_trials} additional trial(s).")
    else:
        print("[Fresh] Starting a new optimization run (no history loaded).")

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
