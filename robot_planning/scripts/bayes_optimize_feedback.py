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

# Resolve paths relative to this script
SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
PKG_DIR = os.path.dirname(SCRIPT_DIR)
BASE_YAML = os.path.join(PKG_DIR, 'config', 'feedback_controller_opt_base.yaml')
TRIAL_YAML = os.path.join(PKG_DIR, 'config', 'feedback_controller_trial.yaml')
SUMMARY_CSV = os.path.join(PKG_DIR, 'tuning_results', 'trials_summary.csv')
PARAMS_CSV = os.path.join(PKG_DIR, 'tuning_results', 'params_history.csv')
STATE_JSON = os.path.join(PKG_DIR, 'tuning_results', 'optimizer_state.json')

PARAM_BOUNDS = {
    'm_pos_min': (0.001, 0.1),
    'm_pos_max': (0.1, 0.5),
    'k_pos_min': (100.0, 500.0),
    'k_pos_max': (1000.0, 3000.0),
    'zeta_pos': (0.5, 1.2),
    'adaptive_lambda': (10.0, 100.0),
    'adaptive_alpha_pos': (5.0, 50.0),
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
    with open(path, 'r') as f:
        return yaml.safe_load(f)

def write_trial_yaml(base_path, trial_path, params):
    data = load_yaml(base_path)
    ros_params = data['/**']['ros__parameters']
    for k, v in params.items():
        ros_params[k] = float(v) if isinstance(v, (int, float)) else v
    with open(trial_path, 'w') as f:
        yaml.safe_dump(data, f, sort_keys=False)

def launch_controller(param_file):
    # Launch motomini.launch.py with bayesian:=true and feedback_yaml_file
    cmd = [
        'ros2', 'launch', 'motomini', 'motomini.launch.py',
        'real_robot:=true', 'debug:=true', 'vel_streaming:=true', 'jogging:=true',
        'bayesian:=true', f'feedback_yaml_file:={os.path.abspath(param_file)}'
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
    print(f"--- Starting Trial {trial_index} ---")

    proc = launch_controller(TRIAL_YAML)
    try:
        time.sleep(10.0)  # wait controller startup and joint state sync

        # Reset services
        subprocess.run(['ros2', 'service', 'call', '/pose_following/stop', 'std_srvs/srv/Trigger', '{}'], timeout=5)
        time.sleep(0.5)
        subprocess.run(['ros2', 'service', 'call', '/pose_following/start', 'std_srvs/srv/Trigger', '{}'], timeout=5)
        time.sleep(0.5)

        # Run tracking trial. This script should print JSON metrics to stdout.
        cmd = [
            'python3', os.path.join(SCRIPT_DIR, 'run_tracking_trial.py'),
            '--ros-args', '-p', f'trial_index:={trial_index}', '-p', 'duration:=10.0'
        ]
        out = subprocess.check_output(cmd, text=True, timeout=30.0)
        
        # parse the last line which should be JSON
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

    rmse = float(m.get('rmse', 1.0))
    ramp_lag = float(m.get('ramp_lag', 1.0))
    overshoot = float(m.get('overshoot', 1.0))
    jitter = float(m.get('arrival_jitter', 1.0))
    rise_time = float(m.get('rise_time', 10.0))
    settling_time = float(m.get('settling_time', 10.0))
    max_feedback_vel = float(m.get('max_feedback_vel', 0.0))
    max_joint_vel_ratio = float(m.get('max_joint_vel_ratio', 0.0))

    cost = (
        1.5 * rise_time / 1.0
        + 2.0 * rmse / 0.001
        + 2.0 * ramp_lag / 0.001
        + 50.0 * overshoot / 0.001
        + 4.0 * jitter / 0.0002
        + 1.0 * settling_time / 1.0
    )

    if overshoot > 0.001:
        cost += 1e6 + overshoot * 1e8

    if max_feedback_vel > 0.85:
        cost += 1e5 + (max_feedback_vel - 0.85) * 1e6

    if max_joint_vel_ratio > 0.95:
        cost += 1e6 + (max_joint_vel_ratio - 0.95) * 1e6

    return float(cost)


def save_trial(trial_index, params, metrics, cost, trial_duration_sec=0.0):
    os.makedirs(os.path.dirname(SUMMARY_CSV), exist_ok=True)

    OPT_STATE['n_trials'] += 1

    if metrics.get('safety_fail', False):
        OPT_STATE['n_fail'] += 1

    prev_best = OPT_STATE['best_cost']
    is_best = cost < prev_best

    if is_best:
        improvement = 0.0 if not np.isfinite(prev_best) else prev_best - cost
        relative_improvement = 0.0 if not np.isfinite(prev_best) else improvement / max(abs(prev_best), 1e-9)
        OPT_STATE['best_cost'] = float(cost)
        OPT_STATE['best_trial'] = int(trial_index)
        OPT_STATE['best_params'] = dict(params)
    else:
        improvement = 0.0
        relative_improvement = 0.0

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
        'improvement': float(improvement),
        'relative_improvement': float(relative_improvement),
        'is_best': bool(is_best),

        'safety_fail': bool(metrics.get('safety_fail', False)),
        'fail_reason': metrics.get('error', ''),
        'fail_rate_so_far': float(fail_rate),

        'J_approach': metrics.get('J_approach', np.nan),
        'J_near': metrics.get('J_near', np.nan),
        'J_tracking': metrics.get('J_tracking', np.nan),

        'rmse': metrics.get('rmse', np.nan),
        'ramp_lag': metrics.get('ramp_lag', np.nan),
        'overshoot': metrics.get('overshoot', np.nan),
        'arrival_jitter': metrics.get('arrival_jitter', np.nan),
        'rise_time': metrics.get('rise_time', np.nan),
        'settling_time': metrics.get('settling_time', np.nan),
        'final_error': metrics.get('final_error', np.nan),

        'max_feedback_vel': metrics.get('max_feedback_vel', np.nan),
        'max_joint_vel_ratio': metrics.get('max_joint_vel_ratio', np.nan),
        'vel_violation': metrics.get('vel_violation', np.nan),

        'exploration_distance': float(exploration_distance),
        'trial_duration_sec': float(trial_duration_sec),
    }

    pd.DataFrame([summary_row]).to_csv(
        SUMMARY_CSV,
        mode='a',
        index=False,
        header=not os.path.exists(SUMMARY_CSV)
    )

    params_row = {'trial': trial_index}
    params_row.update(params)

    pd.DataFrame([params_row]).to_csv(
        PARAMS_CSV,
        mode='a',
        index=False,
        header=not os.path.exists(PARAMS_CSV)
    )

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
    params = {
        'm_pos_min': trial.suggest_float('m_pos_min', 0.001, 0.1),
        'm_pos_max': trial.suggest_float('m_pos_max', 0.1, 0.5),
        'k_pos_min': trial.suggest_float('k_pos_min', 100.0, 500.0),
        'k_pos_max': trial.suggest_float('k_pos_max', 1000.0, 3000.0),
        'zeta_pos': trial.suggest_float('zeta_pos', 0.5, 1.2),
        'adaptive_lambda': trial.suggest_float('adaptive_lambda', 10.0, 100.0),
        'adaptive_alpha_pos': trial.suggest_float('adaptive_alpha_pos', 5.0, 50.0)
    }
    
    # Ensure min < max
    if params['m_pos_min'] >= params['m_pos_max']:
        return 1e9
    if params['k_pos_min'] >= params['k_pos_max']:
        return 1e9

    cost = run_one_trial(params, trial.number)
    return cost

def optuna_callback(study, trial):
    print(
        f"[Optuna] trial={trial.number} "
        f"value={trial.value:.6f} "
        f"best={study.best_value:.6f}"
    )

def load_past_trials(study: optuna.Study) -> int:
    """Re-register all past CSV trials into `study` so TPE can warm-start.

    Returns the number of trials successfully loaded.
    """
    if not os.path.exists(PARAMS_CSV) or not os.path.exists(SUMMARY_CSV):
        print("[Resume] No existing trial data found — starting fresh.")
        return 0

    params_df = pd.read_csv(PARAMS_CSV)
    summary_df = pd.read_csv(SUMMARY_CSV)

    # Keep only the columns we need from summary.
    summary_sub = summary_df[['trial', 'cost', 'safety_fail']].copy()
    merged = pd.merge(params_df, summary_sub, on='trial', how='inner')
    merged = merged.dropna(subset=['cost'])

    if merged.empty:
        print("[Resume] CSV files exist but contain no usable rows — starting fresh.")
        return 0

    # Build the distribution objects that match PARAM_BOUNDS.
    distributions = {
        k: optuna.distributions.FloatDistribution(lo, hi)
        for k, (lo, hi) in PARAM_BOUNDS.items()
    }

    n_loaded = 0
    for _, row in merged.iterrows():
        # Only load params that are still in PARAM_BOUNDS (handles schema changes).
        params = {
            k: float(row[k])
            for k in PARAM_BOUNDS
            if k in row and pd.notna(row[k])
        }
        if len(params) != len(PARAM_BOUNDS):
            # Skip rows where any parameter is missing.
            continue

        cost = float(row['cost'])
        # Safety-fail trials are kept but with their large cost so TPE avoids
        # that region — do not skip them.

        # datetime_start/complete must be non-None for COMPLETE trials.
        _ts = datetime.datetime(2000, 1, 1) + datetime.timedelta(seconds=n_loaded)
        frozen = FrozenTrial(
            number=n_loaded,            # renumbered sequentially
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
    """Restore OPT_STATE from optimizer_state.json if it exists."""
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
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--n-trials', type=int, default=50,
                        help='Number of NEW trials to run (additional when --resume).')
    parser.add_argument('--resume', action='store_true',
                        help='Warm-start from existing CSV logs instead of starting fresh.')
    args = parser.parse_args()

    sampler = optuna.samplers.TPESampler(
        n_startup_trials=10,
        multivariate=True,
        group=True,
        seed=0,
    )

    study = optuna.create_study(
        direction='minimize',
        sampler=sampler,
    )

    if args.resume:
        print("[Resume] Resume mode enabled — loading past trials …")
        restore_opt_state()
        n_loaded = load_past_trials(study)
        if n_loaded == 0:
            print("[Resume] No past trials found; running full search.")
        else:
            print(f"[Resume] Study now has {len(study.trials)} warm-start trials. "
                  f"Running {args.n_trials} additional trial(s).")
    else:
        print("[Fresh] Starting a new optimization run (no history loaded).")

    study.optimize(objective, n_trials=args.n_trials, callbacks=[optuna_callback])

    study.trials_dataframe().to_csv(
        os.path.join(PKG_DIR, 'tuning_results', 'optuna_trials.csv'),
        index=False
    )

    print("Best trial:")
    trial = study.best_trial
    print(f"  Value: {trial.value}")
    print("  Params: ")
    for key, value in trial.params.items():
        print(f"    {key}: {value}")

    print("Best parameters saved to best_params.yaml")
    with open(os.path.join(PKG_DIR, 'tuning_results', 'best_params.yaml'), 'w') as f:
        yaml.safe_dump(trial.params, f)

if __name__ == '__main__':
    main()
