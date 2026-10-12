#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
aggregate_waypoints.py
======================
Multi-run aggregation for the waypoint-following experiments: loads N runs
per base for ONE path (e.g. 5 PMB2 + 5 OmniBase on the hallway), computes
per-run metrics with the SAME definitions as analyze_results.py (which it
imports - keep the two files in the same folder), and produces the
mean/std tables + comparison figures for the thesis.

Run it once per path:
    python3 aggregate_waypoints.py \
        --pmb2-run-dirs logs/pmb2_hallway_* \
        --omni-run-dirs logs/omni_hallway_* \
        --waypoints waypoints_hallway.yaml \
        --label Hallway \
        --out-dir figures_wp/hallway

It also prints the MEDIAN run (by completion time) of each base: use that
pair with analyze_results.py for the in-depth qualitative figures, per the
reporting convention (aggregate stats over all runs, qualitative figures
from the median run).

Outputs in --out-dir:
    summary_<label>.csv                per-run metric table (both bases)
    trajectories_all_<label>.png       all runs overlaid (repeatability)
    boxplots_<label>.png               duration / tracking err / WP err
    cross_track_band_<label>.png       mean +/- std tracking error vs distance
    waypoint_errors_agg_<label>.png    per-waypoint error, mean +/- std
    recovery_counts_<label>.png        recovery & planning failures, mean +/- std

Dependencies: pandas, numpy, matplotlib, pyyaml + analyze_results.py
"""
import argparse
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import analyze_results as ar   # noqa: E402  (shared metric definitions)

COLORS = ar.COLORS
LABELS = ar.LABELS


# ---------------------------------------------------------------------- #
# Per-run analysis                                                        #
# ---------------------------------------------------------------------- #
def analyze_run(run_dir, waypoints):
    run_name = os.path.basename(run_dir.rstrip('/'))
    df = ar.load_log(os.path.join(run_dir, 'log.csv'))
    t0, t1, completed = ar.run_window(df)
    d = ar.window_df(df, t0, t1)
    summary = ar.load_run_summary(run_dir)

    err_df = ar.waypoint_errors(d, waypoints)
    cte, arclen = ar.compute_plan_tracking_error(
        d, waypoints, os.path.join(run_dir, 'plans'))
    counters = ar.load_bt_counters(run_dir, d)

    m = {
        'run': run_name,
        'completed': completed,
        'duration_s': t1 - t0,
        'path_length_m': ar.path_length(d),
        'straight_ref_m': ar.straight_line_length(
            waypoints, (d['gt_x'].iloc[0], d['gt_y'].iloc[0])),
        'track_err_mean_m': float(np.nanmean(cte)) if np.isfinite(cte).any()
                            else float('nan'),
        'track_err_max_m': float(np.nanmax(cte)) if np.isfinite(cte).any()
                           else float('nan'),
        'track_err_rms_m': float(np.sqrt(np.nanmean(cte**2)))
                           if np.isfinite(cte).any() else float('nan'),
        'wp_pos_err_mean_m': float(err_df['pos_err'].mean()),
        'wp_pos_err_max_m': float(err_df['pos_err'].max()),
        'wp_yaw_err_mean_rad': float(err_df['yaw_err'].mean()),
        'smooth_ang': ar.smoothness_metric(d),
        'smooth_lin': ar.linear_smoothness_metric(d),
        'lateral_usage': ar.lateral_usage(d),
    }
    m['avg_speed_mps'] = (m['path_length_m'] / m['duration_s']
                          if m['duration_s'] > 1e-6 else float('nan'))
    m.update({k: counters[k] for k in
              ('spins', 'backups', 'waits', 'costmap_clears', 'replans',
               'plan_failures', 'plan_fail_rate', 'counters_source')})
    if summary:
        m['duration_sim_s'] = float(summary.get('duration_sim_s',
                                                float('nan')))
        m['rtf'] = float(summary.get('rtf', float('nan')))
        m['succeeded'] = bool(summary.get('succeeded', False))
        if not completed and m['succeeded']:
            m['completed'] = True   # log closed early: trust the summary
    return {'metrics': m, 'wp_err': err_df, 'cte': cte, 'arclen': arclen,
            'traj': (d['gt_x'].values, d['gt_y'].values)}


# ---------------------------------------------------------------------- #
# Figures                                                                 #
# ---------------------------------------------------------------------- #
def plot_trajectories_all(runs, waypoints, label, out_path):
    xy = [r['traj'] for base in ('pmb2', 'omni') for r in runs[base]]
    xlim, ylim, figsize, narrow = ar.geo_layout(xy, waypoints, n_panels=2)
    fig, axes = plt.subplots(1, 2, figsize=figsize)
    for ax, base in zip(axes, ('pmb2', 'omni')):
        for i, r in enumerate(runs[base]):
            gx, gy = r['traj']
            ax.plot(gx, gy, alpha=0.7, linewidth=1.1,
                    label=f'Run {i+1}')
        wx = [w['x'] for w in waypoints]
        wy = [w['y'] for w in waypoints]
        ax.scatter(wx, wy, marker='*', s=180, color='black', zorder=5,
                   label='Waypoints')
        ax.set_title(LABELS[base])
        ax.set_xlabel(r'$x$ [m]')
        ax.set_ylabel(r'$y$ [m]')
        ar.geo_finish(ax, xlim, ylim)
        ar.geo_legend(ax, narrow, fontsize=8)
    fig.suptitle(rf'{label}: trajectories of all runs (ground truth)',
                 fontsize=16)
    fig.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close(fig)


def plot_boxes(mtr, label, out_path):
    keys = [('duration_s', 'Completion time [s]'),
            ('track_err_mean_m', 'Mean tracking error [m]'),
            ('wp_pos_err_mean_m', 'Mean waypoint error [m]')]
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    for ax, (k, lab) in zip(axes, keys):
        data = []
        for base in ('pmb2', 'omni'):
            vals = np.asarray([m[k] for m in mtr[base]], float)
            data.append(vals[np.isfinite(vals)])
        ax.boxplot(data)
        ax.set_xticks([1, 2])
        ax.set_xticklabels(['PMB2', 'OmniBase'])
        for i, (vals, base) in enumerate(zip(data, ('pmb2', 'omni')), 1):
            ax.scatter([i] * len(vals), vals, color=COLORS[base], zorder=3)
        ax.set_title(lab)
        ax.grid(True, alpha=0.3, axis='y')
    fig.suptitle(rf'{label}: per-run metrics', fontsize=16)
    fig.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close(fig)


def plot_cross_track_band(runs, label, out_path, step=0.05):
    """Mean +/- std of the tracking error as a function of the travelled
    distance, per base, interpolated on a common arclength grid (up to the
    shortest run of each base)."""
    fig, ax = plt.subplots(figsize=(11, 5.5))
    plotted = False
    for base in ('pmb2', 'omni'):
        series = []
        for r in runs[base]:
            arc, err = r['arclen'], r['cte']
            ok = np.isfinite(arc) & np.isfinite(err)
            arc, err = arc[ok], err[ok]
            if arc.size < 2:
                continue
            keep = np.concatenate(([True], np.diff(arc) > 0))
            series.append((arc[keep], err[keep]))
        if not series:
            continue
        max_common = min(s[0][-1] for s in series)
        grid = np.arange(0.0, max_common, step)
        if grid.size < 2:
            continue
        arr = np.vstack([np.interp(grid, a, e) for a, e in series])
        mean, std = arr.mean(axis=0), arr.std(axis=0)
        ax.plot(grid, mean, color=COLORS[base], linewidth=2.2,
                label=rf'{LABELS[base]} (mean $\pm$ std)')
        ax.fill_between(grid, mean - std, mean + std, color=COLORS[base],
                        alpha=0.18)
        plotted = True
    if not plotted:
        plt.close(fig)
        print('(!) No tracking error series available: band figure skipped.')
        return
    ax.set_xlabel('Travelled distance [m]')
    ax.set_ylabel('Tracking error [m]')
    ax.set_title(rf'{label}: tracking error vs Nav2 /plan (mean $\pm$ std)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close(fig)


def plot_waypoint_errors_agg(runs, n_wp, label, out_path):
    x = np.arange(n_wp)
    width = 0.35
    fig, ax = plt.subplots(figsize=(max(8, 1.2 * n_wp), 5))
    for off, base in [(-width / 2, 'pmb2'), (width / 2, 'omni')]:
        arr = np.vstack([r['wp_err']['pos_err'].values for r in runs[base]])
        # per-column reduction only where at least one run is finite, so an
        # all-NaN waypoint column never reaches nanmean (no RuntimeWarning)
        colmask = np.isfinite(arr).any(axis=0)
        mean = np.zeros(arr.shape[1])
        std = np.zeros(arr.shape[1])
        mean[colmask] = np.nanmean(arr[:, colmask], axis=0)
        std[colmask] = np.nanstd(arr[:, colmask], axis=0)
        ax.bar(x + off, mean, width, yerr=std, capsize=4,
               label=LABELS[base], color=COLORS[base])
    ax.set_xticks(x)
    ax.set_xticklabels([f'WP{i}' for i in x])
    ax.set_ylabel('Position error [m]')
    ax.set_title(rf'{label}: positioning error per waypoint (mean $\pm$ std)')
    ax.legend()
    ax.grid(True, alpha=0.3, axis='y')
    fig.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close(fig)


def plot_recoveries(mtr, label, out_path):
    behaviors = [('spins', 'Spin'), ('backups', 'BackUp'),
                 ('waits', 'Wait'), ('costmap_clears', 'ClearCostmap'),
                 ('plan_failures', 'Plan failures')]
    x = np.arange(len(behaviors))
    width = 0.35
    fig, ax = plt.subplots(figsize=(10, 5))
    top = 0.0
    for off, base in [(-width / 2, 'pmb2'), (width / 2, 'omni')]:
        means, stds = [], []
        for k, _ in behaviors:
            vals = np.asarray([m.get(k, np.nan) for m in mtr[base]], float)
            vals = vals[np.isfinite(vals)]
            means.append(vals.mean() if vals.size else np.nan)
            stds.append(vals.std() if vals.size else np.nan)
        ax.bar(x + off, means, width, yerr=stds, capsize=4,
               label=LABELS[base], color=COLORS[base])
        hi = np.asarray(means, float) + np.nan_to_num(np.asarray(stds, float))
        if np.isfinite(hi).any():
            top = max(top, float(np.nanmax(hi)))
    if not top > 0:
        # nothing was ever triggered: give the figure an honest empty state
        # instead of matplotlib's autoscaled +/-0.04 axis around zero
        ax.set_ylim(0, 1)
        ax.text(0.5, 0.55, 'no recovery behaviours or planning failures\n'
                           'triggered in any run',
                transform=ax.transAxes, ha='center', va='center',
                fontsize=12, color='dimgray')
    ax.set_xticks(x)
    ax.set_xticklabels([b for _, b in behaviors])
    ax.set_ylabel('Mean count per run')
    ax.set_title(f'{label}: recovery behaviours and planning failures')
    ax.grid(True, alpha=0.3, axis='y')
    ax.legend()
    fig.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close(fig)


# ---------------------------------------------------------------------- #
# Main                                                                    #
# ---------------------------------------------------------------------- #
def main():
    p = argparse.ArgumentParser()
    p.add_argument('--pmb2-run-dirs', nargs='+', required=True)
    p.add_argument('--omni-run-dirs', nargs='+', required=True)
    p.add_argument('--waypoints', required=True)
    p.add_argument('--label', default='path',
                   help="Path name for titles/filenames (e.g. 'Hallway')")
    p.add_argument('--out-dir', default='./figures_wp')
    args = p.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    waypoints = ar.load_waypoints(args.waypoints)

    runs = {'pmb2': [analyze_run(d, waypoints) for d in args.pmb2_run_dirs],
            'omni': [analyze_run(d, waypoints) for d in args.omni_run_dirs]}
    mtr = {b: [r['metrics'] for r in rs] for b, rs in runs.items()}

    show_cols = ['run', 'completed', 'duration_s', 'path_length_m',
                 'track_err_mean_m', 'track_err_max_m', 'wp_pos_err_mean_m',
                 'wp_pos_err_max_m', 'spins', 'backups', 'replans',
                 'plan_failures', 'counters_source']
    frames = []
    for base in ('pmb2', 'omni'):
        tab = pd.DataFrame(mtr[base])
        tab.insert(0, 'base', base)
        frames.append(tab)
        cols = [c for c in show_cols if c in tab.columns]
        print(f'\n=== {LABELS[base]} - {args.label} ===')
        with pd.option_context('display.width', 240):
            print(tab[cols].to_string(index=False))
    summary_csv = os.path.join(args.out_dir, f'summary_{args.label}.csv')
    pd.concat(frames, ignore_index=True).to_csv(summary_csv, index=False)

    print(f'\n=== {args.label}: aggregates (mean ± std across runs) ===')
    num_keys = ['duration_s', 'duration_sim_s', 'path_length_m',
                'straight_ref_m', 'avg_speed_mps',
                'track_err_mean_m', 'track_err_max_m', 'track_err_rms_m',
                'wp_pos_err_mean_m', 'wp_pos_err_max_m',
                'wp_yaw_err_mean_rad', 'smooth_ang', 'smooth_lin',
                'lateral_usage', 'rtf',
                'spins', 'backups', 'waits', 'costmap_clears',
                'replans', 'plan_failures', 'plan_fail_rate']
    for k in num_keys:
        line = f'{k:22s}'
        for base in ('pmb2', 'omni'):
            vals = np.asarray([m.get(k, np.nan) for m in mtr[base]], float)
            vals = vals[np.isfinite(vals)]
            if vals.size:
                line += f'  {base.upper():5s}={vals.mean():9.3f} ± {vals.std():7.3f}'
            else:
                line += f'  {base.upper():5s}=      n/a          '
        print(line)

    # Median run per base (by completion time) -> deep-dive pair
    print(f'\n=== Suggested MEDIAN pair for analyze_results.py ===')
    median_dirs = {}
    for base, arg_dirs in [('pmb2', args.pmb2_run_dirs),
                           ('omni', args.omni_run_dirs)]:
        durs = np.asarray([m['duration_s'] for m in mtr[base]])
        i = int(np.argsort(durs)[len(durs) // 2])
        median_dirs[base] = arg_dirs[i]
        print(f'  {base}: {arg_dirs[i]}  (duration {durs[i]:.1f} s)')
    print('  Command:\n'
          f'    python3 analyze_results.py --pmb2-run-dir {median_dirs["pmb2"]} '
          f'--omni-run-dir {median_dirs["omni"]} '
          f'--waypoints {args.waypoints} '
          f'--out-dir {os.path.join(args.out_dir, "median_pair")}')

    lab = args.label
    plot_trajectories_all(runs, waypoints, lab,
                          os.path.join(args.out_dir,
                                       f'trajectories_all_{lab}.png'))
    plot_boxes(mtr, lab, os.path.join(args.out_dir, f'boxplots_{lab}.png'))
    plot_cross_track_band(runs, lab,
                          os.path.join(args.out_dir,
                                       f'cross_track_band_{lab}.png'))
    plot_waypoint_errors_agg(runs, len(waypoints), lab,
                             os.path.join(args.out_dir,
                                          f'waypoint_errors_agg_{lab}.png'))
    plot_recoveries(mtr, lab,
                    os.path.join(args.out_dir, f'recovery_counts_{lab}.png'))
    print(f'\nSummary table: {summary_csv}')
    print(f'Figures saved in {args.out_dir}/')


if __name__ == '__main__':
    main()
