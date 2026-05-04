#!/usr/bin/env python3
"""
Generate IEEE-style figures from Bayesian optimization tuning results.
Output images are saved to the thesis LaTeX images folder.
"""

import os
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import matplotlib.patches as mpatches

# ── Paths ─────────────────────────────────────────────────────────────────────
SCRIPT_DIR  = os.path.dirname(os.path.realpath(__file__))
SUMMARY_CSV = os.path.join(SCRIPT_DIR, 'trials_summary.csv')
PARAMS_CSV  = os.path.join(SCRIPT_DIR, 'params_history.csv')
OUT_DIR     = '/home/vinbui/HCMUT/Thesis/LVTN/texdoc/chapter_ket_qua/images'
os.makedirs(OUT_DIR, exist_ok=True)

# ── IEEE RC params ─────────────────────────────────────────────────────────────
# Single-column IEEE width ≈ 3.5 in; double-column ≈ 7.16 in
IEEE_W1 = 3.5    # single column
IEEE_W2 = 7.16   # double column

matplotlib.rcParams.update({
    # Font
    'font.family'        : 'serif',
    'font.serif'         : ['Times New Roman', 'DejaVu Serif'],
    'mathtext.fontset'   : 'stix',
    'font.size'          : 10,
    'axes.titlesize'     : 10,
    'axes.labelsize'     : 10,
    'xtick.labelsize'    : 9,
    'ytick.labelsize'    : 9,
    'legend.fontsize'    : 9,
    'legend.framealpha'  : 0.85,
    'legend.edgecolor'   : '0.6',
    # Lines
    'lines.linewidth'    : 1.0,
    'axes.linewidth'     : 0.6,
    'xtick.major.width'  : 0.6,
    'ytick.major.width'  : 0.6,
    'xtick.minor.width'  : 0.4,
    'ytick.minor.width'  : 0.4,
    'xtick.direction'    : 'in',
    'ytick.direction'    : 'in',
    'xtick.top'          : True,
    'ytick.right'        : True,
    # Grid
    'axes.grid'          : True,
    'grid.linestyle'     : '--',
    'grid.linewidth'     : 0.4,
    'grid.alpha'         : 0.5,
    # Save
    'figure.dpi'         : 150,
    'savefig.dpi'        : 300,
    'savefig.bbox'       : 'tight',
    'savefig.pad_inches' : 0.02,
})

# ── IEEE colour palette (print-safe, colour-blind friendly) ───────────────────
C0 = '#000000'   # black      – primary line / best
C1 = '#555555'   # dark grey  – secondary
C2 = '#AAAAAA'   # light grey – tertiary / median
C3 = '#000000'   # black dashed for reference lines
MARKER_BEST   = 'D'   # diamond  – best trial
MARKER_FAIL   = 'x'   # cross    – failed trial
MARKER_VALID  = 'o'   # circle   – valid trial

def remove_top_right(ax):
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.yaxis.set_ticks_position('left')
    ax.xaxis.set_ticks_position('bottom')
    # Keep grid but disable top/right ticks
    matplotlib.rcParams['xtick.top']   = False
    matplotlib.rcParams['ytick.right'] = False

# ── Load data ─────────────────────────────────────────────────────────────────
df   = pd.read_csv(SUMMARY_CSV)
prms = pd.read_csv(PARAMS_CSV)

df['failed'] = df['safety_fail'].astype(bool)
df['valid']  = ~df['failed']
df_valid     = df[df['valid']].copy()
merged       = df_valid.merge(prms, on='trial', how='left')
valid_costs  = df_valid['cost'].values

best_trial_no = 86
best_prm_row  = prms[prms['trial'] == best_trial_no]

# ─────────────────────────────────────────────────────────────────────────────
# Fig 1 – Convergence curve  (double-column width)
# ─────────────────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(IEEE_W2, 2.4))

ax.plot(df['trial'], df['best_cost_so_far'],
        color=C0, linewidth=1.2, zorder=4,
        label='Chi phí tốt nhất tích lũy')

ax.scatter(df_valid['trial'], df_valid['cost'],
           marker=MARKER_VALID, s=6, color=C1, alpha=0.45, zorder=2,
           linewidths=0, label='Thử nghiệm hợp lệ')

fail_y = df['best_cost_so_far'].iloc[-1] + 3
ax.scatter(df[df['failed']]['trial'],
           [fail_y] * df['failed'].sum(),
           marker=MARKER_FAIL, s=14, color=C0, alpha=0.55, linewidths=0.7,
           zorder=3, label='Vi phạm ràng buộc')

best_pts = df[df['is_best'] == True]
ax.scatter(best_pts['trial'], best_pts['cost'],
           marker=MARKER_BEST, s=40, facecolors='none', edgecolors=C0,
           linewidths=1.0, zorder=5, label='Điểm cải thiện')

offsets = {0: (6, 2.5), 2: (6, 2.5), 11: (6, 2.5), 86: (-20, 2.5)}
for _, row in best_pts.iterrows():
    dx, dy = offsets.get(int(row['trial']), (6, 2.5))
    ax.annotate(f"$t={int(row['trial'])}$\n$J={row['cost']:.2f}$",
                xy=(row['trial'], row['cost']),
                xytext=(row['trial'] + dx, row['cost'] + dy),
                fontsize=8,
                arrowprops=dict(arrowstyle='->', lw=0.6, color='0.3'),
                color='0.2')

ax.set_xlabel('Chỉ số thử nghiệm')
ax.set_ylabel('Hàm chi phí $J$')
ax.set_xlim(-2, 152)
ax.legend(loc='upper right', ncol=2)
ax.grid(True, linestyle='--', linewidth=0.4, alpha=0.5)
fig.tight_layout()
fig.savefig(os.path.join(OUT_DIR, 'convergence_curve.pdf'))
fig.savefig(os.path.join(OUT_DIR, 'convergence_curve.png'))
plt.close(fig)
print("Saved: convergence_curve")

# ─────────────────────────────────────────────────────────────────────────────
# Fig 2 – Cost distribution histogram  (single-column)
# ─────────────────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(IEEE_W1, 2.2))

bins = np.linspace(valid_costs.min() - 1, valid_costs.max() + 5, 28)
ax.hist(valid_costs, bins=bins,
        color=C1, alpha=0.65, edgecolor='white', linewidth=0.3,
        label=f'Thử nghiệm hợp lệ ($n={len(df_valid)}$)')
ax.axvline(valid_costs.min(), color=C0, linewidth=1.0, linestyle='--',
           label=f'Tốt nhất $J={valid_costs.min():.2f}$')
ax.axvline(np.median(valid_costs), color=C0, linewidth=1.0, linestyle=':',
           label=f'Trung vị $J={np.median(valid_costs):.2f}$')

n_fail = df['failed'].sum()
ax.text(0.97, 0.96,
        f'Vi phạm: $n={n_fail}$ (đã loại)',
        transform=ax.transAxes, ha='right', va='top',
        fontsize=8, color='0.35')

ax.set_xlabel('Hàm chi phí $J$')
ax.set_ylabel('Số lượng')
ax.legend(loc='upper right')
ax.yaxis.set_major_locator(ticker.MaxNLocator(integer=True))
fig.tight_layout()
fig.savefig(os.path.join(OUT_DIR, 'cost_distribution.pdf'))
fig.savefig(os.path.join(OUT_DIR, 'cost_distribution.png'))
plt.close(fig)
print("Saved: cost_distribution")

# ─────────────────────────────────────────────────────────────────────────────
# Fig 3 – Scatter: each param vs cost  (double-column, 2×4 grid)
# ─────────────────────────────────────────────────────────────────────────────
param_cols = ['m_pos_min', 'm_pos_max', 'k_pos_min', 'k_pos_max',
              'zeta_pos', 'adaptive_lambda', 'adaptive_alpha_pos']
param_lbls = [r'$m_{\min}$ (kg)', r'$m_{\max}$ (kg)',
              r'$k_{\min}$ (N/m)', r'$k_{\max}$ (N/m)',
              r'$\zeta$', r'$\lambda$', r'$\alpha_{\mathrm{pos}}$']

fig, axes = plt.subplots(2, 4, figsize=(IEEE_W2, 4.4))
fig.subplots_adjust(left=0.08, right=0.87, bottom=0.12, top=0.97,
                    wspace=0.55, hspace=0.5)
axes = axes.flatten()

vmin, vmax = valid_costs.min(), min(70, valid_costs.max())

for i, (col, lbl) in enumerate(zip(param_cols, param_lbls)):
    ax = axes[i]
    if col not in merged.columns:
        ax.set_visible(False)
        continue

    sc = ax.scatter(merged[col], merged['cost'],
                    c=merged['cost'].clip(upper=vmax),
                    cmap='Greys', vmin=vmin, vmax=vmax,
                    s=7, alpha=0.7, linewidths=0, zorder=3)

    if col in best_prm_row.columns:
        bval = best_prm_row[col].values[0]
        ax.axvline(bval, color=C0, linewidth=0.9, linestyle='--', zorder=4)
        ax.text(bval, ax.get_ylim()[1] if ax.get_ylim()[1] > 0 else 70,
                f' {bval:.3g}', fontsize=7, va='top', color='0.2')

    ax.set_xlabel(lbl)
    if i % 4 == 0:
        ax.set_ylabel('Hàm chi phí $J$')

axes[-1].set_visible(False)

# Colorbar trong trục riêng — tránh đè lên biểu đồ
cax = fig.add_axes([0.895, 0.12, 0.018, 0.85])
cb = fig.colorbar(sc, cax=cax)
cb.ax.tick_params(labelsize=9)
cb.set_label('Hàm chi phí $J$', fontsize=10)

fig.savefig(os.path.join(OUT_DIR, 'scatter_params_cost.pdf'))
fig.savefig(os.path.join(OUT_DIR, 'scatter_params_cost.png'))
plt.close(fig)
print("Saved: scatter_params_cost")

# ─────────────────────────────────────────────────────────────────────────────
# Fig 4 – Metric comparison bar chart  (double-column)
# ─────────────────────────────────────────────────────────────────────────────
metric_cols   = ['J_approach', 'J_near', 'J_tracking',
                 'rmse', 'ramp_lag', 'rise_time', 'settling_time']
metric_lbls   = [r'$J_{\mathrm{app}}$', r'$J_{\mathrm{near}}$',
                 r'$J_{\mathrm{track}}$',
                 'RMSE\n(mm)', 'Lag\n(ms)', r'$t_r$'+'\n(s)', r'$t_s$'+'\n(ms)']
scale_factors = [1, 1, 1, 1e3, 1e3, 1, 1e3]

best_row   = df_valid[df_valid['trial'] == best_trial_no].iloc[0]
median_idx = (df_valid['cost'] - df_valid['cost'].median()).abs().argsort().iloc[0]
median_row = df_valid.iloc[median_idx]

best_vals   = [float(best_row[c])   * s for c, s in zip(metric_cols, scale_factors)]
median_vals = [float(median_row[c]) * s for c, s in zip(metric_cols, scale_factors)]

x = np.arange(len(metric_cols))
w = 0.32

fig, ax = plt.subplots(figsize=(IEEE_W2, 2.5))
b1 = ax.bar(x - w/2, best_vals, w,
            color=C0, alpha=0.85, edgecolor='none',
            label=f'Tốt nhất (Thử nghiệm {best_trial_no})')
b2 = ax.bar(x + w/2, median_vals, w,
            color=C2, alpha=0.85, edgecolor='none',
            label=f'Trung vị (Thử nghiệm {int(median_row["trial"])})')

ax.set_xticks(x)
ax.set_xticklabels(metric_lbls, fontsize=9)
ax.set_ylabel('Giá trị (đơn vị quy đổi)')
ax.legend(loc='upper left', ncol=2)
ax.bar_label(b1, fmt='%.3g', fontsize=7.5, padding=1.5)
ax.bar_label(b2, fmt='%.3g', fontsize=7.5, padding=1.5)
fig.tight_layout()
fig.savefig(os.path.join(OUT_DIR, 'metric_comparison.pdf'))
fig.savefig(os.path.join(OUT_DIR, 'metric_comparison.png'))
plt.close(fig)
print("Saved: metric_comparison")

# ─────────────────────────────────────────────────────────────────────────────
# Fig 5 – Cumulative failure rate  (single-column)
# ─────────────────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(IEEE_W1, 2.0))

ax.plot(df['trial'], df['fail_rate_so_far'] * 100,
        color=C0, linewidth=1.0)
ax.fill_between(df['trial'], df['fail_rate_so_far'] * 100,
                alpha=0.12, color=C0)
ax.axhline(df['fail_rate_so_far'].iloc[-1] * 100,
           color=C0, linewidth=0.7, linestyle='--',
           label=f"Tỉ lệ cuối = {df['fail_rate_so_far'].iloc[-1]*100:.1f}%")

ax.set_xlabel('Chỉ số thử nghiệm')
ax.set_ylabel('Tỉ lệ vi phạm tích lũy (%)')
ax.set_ylim(0, None)
ax.yaxis.set_major_formatter(ticker.FuncFormatter(lambda v, _: f'{v:.0f}%'))
ax.legend(loc='upper right')
fig.tight_layout()
fig.savefig(os.path.join(OUT_DIR, 'fail_rate_evolution.pdf'))
fig.savefig(os.path.join(OUT_DIR, 'fail_rate_evolution.png'))
plt.close(fig)
print("Saved: fail_rate_evolution")

print(f"\nAll figures saved to: {OUT_DIR}")
