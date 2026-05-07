#!/usr/bin/env python3
"""
Tuning analysis: cost breakdown, param correlations, convergence plots.
Usage:  python3 analyze_tuning.py
        python3 analyze_tuning.py --top 10   # show top-N trials detail
"""
import os
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.colors import LogNorm

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
PKG_DIR    = os.path.dirname(SCRIPT_DIR)
RESULTS    = os.path.join(PKG_DIR, 'tuning_results')
SUMMARY_CSV = os.path.join(RESULTS, 'trials_summary.csv')
PARAMS_CSV  = os.path.join(RESULTS, 'params_history.csv')
OUT_DIR     = os.path.join(RESULTS, 'analysis_plots')

PARAM_NAMES = [
    'm_pos_min', 'm_pos_max',
    'k_pos_min', 'k_pos_max',
    'zeta_pos', 'adaptive_lambda', 'adaptive_alpha_pos',
    'i_gain_pos', 'i_gain_ori', 'i_clamp_pos', 'i_clamp_ori',
]

METRIC_NAMES = [
    'rmse', 'ramp_lag', 'overshoot', 'arrival_jitter',
    'rise_time', 'settling_time',
    'orientation_rmse', 'orientation_max_error',
    'max_feedback_vel', 'max_joint_vel_ratio',
]

PENALTY_THRESHOLDS = {
    'overshoot':           (0.001, '+1e6'),
    'max_feedback_vel':    (0.85,  '+1e5'),
    'max_joint_vel_ratio': (0.95,  '+1e6'),
    'orientation_max_error': (0.20, '+5e5'),
}

# Effective multipliers (weight / normalization) from compute_cost
COST_WEIGHTS = {
    'rise_time':              1.5  / 1.0,
    'rmse':                   2.0  / 0.001,
    'ramp_lag':               2.0  / 0.001,
    'overshoot':              50.0 / 0.001,
    'arrival_jitter':         4.0  / 0.0002,
    'settling_time':          1.0  / 1.0,
    'orientation_rmse':       6.0  / 0.05,
    'orientation_max_error':  4.0  / 0.1,
}


def load_data():
    if not os.path.exists(SUMMARY_CSV):
        raise FileNotFoundError(f"Missing {SUMMARY_CSV}")
    if not os.path.exists(PARAMS_CSV):
        raise FileNotFoundError(f"Missing {PARAMS_CSV}")
    summary = pd.read_csv(SUMMARY_CSV)
    params  = pd.read_csv(PARAMS_CSV)
    df = pd.merge(summary, params, on='trial', how='inner')
    return df


def classify_penalty_reason(row):
    """Return primary reason why a trial hit a hard penalty cliff."""
    if row.get('safety_fail', False):
        return 'safety_fail'
    reasons = []
    for col, (thresh, label) in PENALTY_THRESHOLDS.items():
        val = row.get(col, np.nan)
        if pd.notna(val) and val > thresh:
            reasons.append(f'{col}>{thresh} ({label})')
    return '; '.join(reasons) if reasons else 'none'


# ─────────────────────────────────────────────────────────────
# PLOT 1 – Convergence curve
# ─────────────────────────────────────────────────────────────
def plot_convergence(df, ax=None):
    standalone = ax is None
    if standalone:
        fig, ax = plt.subplots(figsize=(10, 4))

    valid = df[~df['safety_fail'].astype(bool)]
    best  = df['best_cost_so_far']

    ax.semilogy(df['trial'], df['cost'], 'o', color='steelblue',
                alpha=0.4, markersize=3, label='Trial cost')
    ax.semilogy(df['trial'], best, 'r-', linewidth=2, label='Best so far')

    # Mark safety-fail trials
    fails = df[df['safety_fail'].astype(bool)]
    if not fails.empty:
        ax.scatter(fails['trial'], fails['cost'],
                   marker='x', color='red', s=40, label='Safety fail', zorder=5)

    ax.set_xlabel('Trial #')
    ax.set_ylabel('Cost (log scale)')
    ax.set_title('Convergence')
    ax.legend(fontsize=8)
    ax.grid(True, which='both', ls='--', alpha=0.4)

    if standalone:
        plt.tight_layout()
        plt.savefig(os.path.join(OUT_DIR, '1_convergence.png'), dpi=150)
        plt.close()


# ─────────────────────────────────────────────────────────────
# PLOT 2 – Cost component breakdown (top N trials, stacked bar)
# ─────────────────────────────────────────────────────────────
def plot_cost_breakdown(df, top_n=20):
    valid = df[~df['safety_fail'].astype(bool)].nsmallest(top_n, 'cost').copy()
    if valid.empty:
        print("[WARN] No valid (non-fail) trials to plot breakdown.")
        return

    components = {}
    for metric, w in COST_WEIGHTS.items():
        if metric in valid.columns:
            components[metric] = valid[metric].fillna(0).clip(lower=0) * w

    comp_df = pd.DataFrame(components, index=valid['trial'])
    comp_df = comp_df.clip(lower=0)

    fig, ax = plt.subplots(figsize=(max(10, top_n * 0.6), 5))
    comp_df.plot(kind='bar', stacked=True, ax=ax, colormap='tab10', width=0.85)
    ax.set_xlabel('Trial #')
    ax.set_ylabel('Cost contribution')
    ax.set_title(f'Cost breakdown — top {top_n} lowest-cost trials')
    ax.legend(fontsize=7, loc='upper right')
    plt.xticks(rotation=45)
    plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR, '2_cost_breakdown.png'), dpi=150)
    plt.close()


# ─────────────────────────────────────────────────────────────
# PLOT 3 – Penalty cliff hit frequency
# ─────────────────────────────────────────────────────────────
def plot_penalty_hits(df):
    fig, axes = plt.subplots(1, len(PENALTY_THRESHOLDS), figsize=(14, 4), sharey=False)
    for ax, (col, (thresh, label)) in zip(axes, PENALTY_THRESHOLDS.items()):
        if col not in df.columns:
            ax.set_title(f'{col}\n(no data)')
            continue
        vals = df[col].dropna()
        hit  = (vals > thresh).sum()
        ok   = (vals <= thresh).sum()
        ax.bar(['≤ thresh\n(OK)', f'> {thresh}\n(PENALTY)'],
               [ok, hit], color=['#2ecc71', '#e74c3c'])
        ax.set_title(f'{col}\npenalty: {label}', fontsize=9)
        ax.set_ylabel('# trials')

        # Overlay distribution
        ax2 = ax.twinx()
        ax2.hist(vals, bins=30, alpha=0.25, color='gray', density=True)
        ax2.axvline(thresh, color='red', ls='--', lw=1.5, label=f'thresh={thresh}')
        ax2.set_ylabel('density', fontsize=7)
        ax2.legend(fontsize=7)

    plt.suptitle('Penalty cliff analysis', fontsize=11)
    plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR, '3_penalty_hits.png'), dpi=150)
    plt.close()


# ─────────────────────────────────────────────────────────────
# PLOT 4 – Param vs cost scatter (each param, color=cost)
# ─────────────────────────────────────────────────────────────
def plot_param_vs_cost(df):
    present = [p for p in PARAM_NAMES if p in df.columns]
    valid   = df[~df['safety_fail'].astype(bool)].copy()
    if valid.empty:
        print("[WARN] No valid trials for param-vs-cost scatter.")
        return

    n_col = 4
    n_row = int(np.ceil(len(present) / n_col))
    fig, axes = plt.subplots(n_row, n_col, figsize=(16, n_row * 3))
    axes = axes.flatten()

    # Use log-clipped cost for color
    cost_vals = valid['cost'].clip(lower=1).values
    vmin, vmax = cost_vals.min(), np.percentile(cost_vals, 95)

    for i, param in enumerate(present):
        ax = axes[i]
        sc = ax.scatter(valid[param], valid['cost'],
                        c=cost_vals,
                        norm=LogNorm(vmin=max(vmin, 1), vmax=max(vmax, 2)),
                        cmap='RdYlGn_r', s=15, alpha=0.7)
        ax.set_xlabel(param, fontsize=8)
        ax.set_ylabel('cost', fontsize=8)
        ax.set_yscale('log')
        ax.set_title(param, fontsize=9)
        ax.grid(True, ls='--', alpha=0.3)

        # Annotate best trial value
        best_row = valid.loc[valid['cost'].idxmin()]
        ax.axvline(best_row[param], color='blue', ls=':', lw=1.5,
                   label=f'best={best_row[param]:.4g}')
        ax.legend(fontsize=6)

    # Hide unused subplots
    for j in range(len(present), len(axes)):
        axes[j].set_visible(False)

    fig.colorbar(sc, ax=axes[:len(present)], label='cost', shrink=0.6)
    plt.suptitle('Parameter vs Cost (scatter)', fontsize=12, y=1.01)
    plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR, '4_param_vs_cost.png'), dpi=150, bbox_inches='tight')
    plt.close()


# ─────────────────────────────────────────────────────────────
# PLOT 5 – Correlation heatmap: params → metrics
# ─────────────────────────────────────────────────────────────
def plot_correlation_heatmap(df):
    present_params  = [p for p in PARAM_NAMES  if p in df.columns]
    present_metrics = [m for m in METRIC_NAMES if m in df.columns] + ['cost']
    valid = df[~df['safety_fail'].astype(bool)][present_params + present_metrics].dropna()

    if len(valid) < 5:
        print("[WARN] Too few valid trials for correlation heatmap.")
        return

    corr = valid[present_params].corrwith(
        valid[present_metrics].apply(lambda c: c.clip(upper=np.percentile(c.dropna(), 99)))
    ).fillna(0)

    # Full cross-correlation matrix
    full_corr = valid.corr(numeric_only=True)
    sub = full_corr.loc[present_params, present_metrics]

    fig, ax = plt.subplots(figsize=(max(8, len(present_metrics)), max(5, len(present_params) * 0.55)))
    im = ax.imshow(sub.values, aspect='auto', cmap='RdBu_r', vmin=-1, vmax=1)
    plt.colorbar(im, ax=ax, label='Pearson r')
    ax.set_xticks(range(len(present_metrics)))
    ax.set_yticks(range(len(present_params)))
    ax.set_xticklabels(present_metrics, rotation=45, ha='right', fontsize=8)
    ax.set_yticklabels(present_params, fontsize=8)
    ax.set_title('Param → Metric Correlation (Pearson)', fontsize=10)

    # Annotate values
    for r in range(len(present_params)):
        for c in range(len(present_metrics)):
            v = sub.values[r, c]
            ax.text(c, r, f'{v:.2f}', ha='center', va='center',
                    fontsize=6, color='black' if abs(v) < 0.6 else 'white')

    plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR, '5_correlation_heatmap.png'), dpi=150)
    plt.close()


# ─────────────────────────────────────────────────────────────
# PLOT 6 – Metric distributions: good vs bad trials
# ─────────────────────────────────────────────────────────────
def plot_metric_distributions(df):
    present = [m for m in METRIC_NAMES if m in df.columns]
    valid   = df[~df['safety_fail'].astype(bool)].copy()
    if valid.empty:
        return

    median_cost = valid['cost'].median()
    good = valid[valid['cost'] <= median_cost]
    bad  = valid[valid['cost'] >  median_cost]

    n_col = 4
    n_row = int(np.ceil(len(present) / n_col))
    fig, axes = plt.subplots(n_row, n_col, figsize=(16, n_row * 3))
    axes = axes.flatten()

    for i, metric in enumerate(present):
        ax = axes[i]
        g_vals = good[metric].dropna().clip(upper=np.percentile(valid[metric].dropna(), 99))
        b_vals = bad[metric].dropna().clip(upper=np.percentile(valid[metric].dropna(), 99))
        bins = np.linspace(min(g_vals.min(), b_vals.min()), max(g_vals.max(), b_vals.max()), 30)
        ax.hist(g_vals, bins=bins, alpha=0.6, color='green', label='low cost', density=True)
        ax.hist(b_vals, bins=bins, alpha=0.6, color='red',   label='high cost', density=True)
        thresh_info = PENALTY_THRESHOLDS.get(metric)
        if thresh_info:
            ax.axvline(thresh_info[0], color='black', ls='--', lw=1.5,
                       label=f'thresh={thresh_info[0]}')
        ax.set_title(metric, fontsize=9)
        ax.legend(fontsize=6)
        ax.grid(True, ls='--', alpha=0.3)

    for j in range(len(present), len(axes)):
        axes[j].set_visible(False)

    plt.suptitle('Metric distributions: good (low cost) vs bad (high cost) trials', fontsize=11)
    plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR, '6_metric_distributions.png'), dpi=150)
    plt.close()


# ─────────────────────────────────────────────────────────────
# PLOT 7 – Exploration distance over time
# ─────────────────────────────────────────────────────────────
def plot_exploration(df):
    if 'exploration_distance' not in df.columns:
        return
    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

    axes[0].plot(df['trial'], df['exploration_distance'], color='purple', alpha=0.7)
    axes[0].set_ylabel('Exploration distance')
    axes[0].set_title('Search exploration over trials')
    axes[0].grid(True, ls='--', alpha=0.4)

    axes[1].semilogy(df['trial'], df['cost'].clip(lower=1), color='steelblue', alpha=0.6)
    axes[1].semilogy(df['trial'], df['best_cost_so_far'], 'r-', lw=2)
    axes[1].set_xlabel('Trial #')
    axes[1].set_ylabel('Cost (log)')
    axes[1].grid(True, which='both', ls='--', alpha=0.4)

    plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR, '7_exploration.png'), dpi=150)
    plt.close()


# ─────────────────────────────────────────────────────────────
# TEXT SUMMARY
# ─────────────────────────────────────────────────────────────
def print_summary(df, top_n=10):
    print("\n" + "="*60)
    print("TUNING SUMMARY")
    print("="*60)
    total   = len(df)
    n_fail  = df['safety_fail'].astype(bool).sum()
    print(f"Total trials  : {total}")
    print(f"Safety fails  : {n_fail} ({100*n_fail/max(total,1):.1f}%)")

    valid = df[~df['safety_fail'].astype(bool)]
    if valid.empty:
        print("No valid (non-fail) trials.")
        return

    print(f"Valid trials  : {len(valid)}")
    print(f"Best cost     : {valid['cost'].min():.4f}  (trial #{valid.loc[valid['cost'].idxmin(),'trial']})")
    print(f"Median cost   : {valid['cost'].median():.4f}")

    print("\n--- PENALTY CLIFF HITS (valid trials only) ---")
    for col, (thresh, label) in PENALTY_THRESHOLDS.items():
        if col in valid.columns:
            n_hit = (valid[col].dropna() > thresh).sum()
            pct   = 100 * n_hit / max(len(valid[col].dropna()), 1)
            print(f"  {col} > {thresh}: {n_hit} trials ({pct:.1f}%)  → penalty {label}")

    print(f"\n--- TOP {top_n} TRIALS ---")
    top = valid.nsmallest(top_n, 'cost')[
        ['trial', 'cost'] + [m for m in METRIC_NAMES if m in valid.columns]
    ]
    with pd.option_context('display.max_columns', None, 'display.width', 200,
                           'display.float_format', '{:.5g}'.format):
        print(top.to_string(index=False))

    print("\n--- PRIMARY COST DRIVERS (best trial) ---")
    best_row = valid.loc[valid['cost'].idxmin()]
    contribs = {m: best_row.get(m, 0) * w for m, w in COST_WEIGHTS.items()
                if pd.notna(best_row.get(m, np.nan))}
    for m, v in sorted(contribs.items(), key=lambda x: -x[1]):
        pct = 100 * v / max(sum(contribs.values()), 1e-12)
        print(f"  {m:<30s}: {v:.4f}  ({pct:.1f}%)")

    # Penalty reason per trial
    df['penalty_reason'] = df.apply(classify_penalty_reason, axis=1)
    reasons = df[df['penalty_reason'] != 'none']['penalty_reason'].value_counts().head(5)
    if not reasons.empty:
        print("\n--- TOP PENALTY REASONS ---")
        print(reasons.to_string())


def main():
    parser = argparse.ArgumentParser(description='Analyze Bayesian tuning results.')
    parser.add_argument('--top', type=int, default=10,
                        help='Number of top trials to show in summary.')
    args = parser.parse_args()

    os.makedirs(OUT_DIR, exist_ok=True)

    print(f"Loading data from {RESULTS} …")
    df = load_data()
    print(f"Loaded {len(df)} trials.")

    print_summary(df, top_n=args.top)

    print("\nGenerating plots …")
    plot_convergence(df)
    plot_cost_breakdown(df, top_n=min(20, len(df)))
    plot_penalty_hits(df)
    plot_param_vs_cost(df)
    plot_correlation_heatmap(df)
    plot_metric_distributions(df)
    plot_exploration(df)

    print(f"\nAll plots saved to: {OUT_DIR}/")
    print("Files:")
    for f in sorted(os.listdir(OUT_DIR)):
        print(f"  {f}")


if __name__ == '__main__':
    main()
