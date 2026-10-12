#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze_exploration.py  (fixed + English figures)
=================================================
Offline analysis of the AUTONOMOUS EXPLORATION runs: loads N runs per base
(produced by exploration_logger.py + autonomous_explorer_thesis.py) and
computes aggregate metrics (mean/std) + PMB2 vs OmniBase figures.

Unlike analyze_results.py (waypoint following), there is no common reference
trajectory here: the metrics are task-level and are computed over the
EXPLORATION WINDOW, i.e. from the first frontier goal (goal_idx >= 0) to the
completion signal (goal_idx == -1). The return-to-base phase is excluded.

Per-run metrics:
  - exploration duration [s, simulated time]
  - ground-truth path length [m]
  - final explored area [m^2] + coverage-vs-time curve
  - efficiency [m^2/m] = area/path  and  [m^2/s] = area/duration
  - time to reach a COMMON TARGET AREA (default: 95% of the MINIMUM final
    area across all runs, so every run is guaranteed to reach it: compares
    times at equal coverage, robust to runs stopping at slightly different
    final coverages)
  - SLAM localization error vs ground truth (mean/max). The SLAM map frame
    and the Gazebo world frame are ALIGNED PER RUN with a constant SE(2)
    transform estimated from the first samples where both poses are valid
    (standard ATE-style origin alignment). This is necessary because
    slam_toolbox anchors the map frame at the ODOM origin, and the odom
    convention differs between the bases: PMB2 wheel odometry starts at
    (0,0,0) wherever the robot spawns, while the OmniBase planar-move
    odometry is world-referenced. Use --spawn-x/y/yaw only to force the
    map-origin pose manually for ALL runs.
  - smoothness (RMS of commanded angular/linear acceleration), lateral usage
  - recovery counts (Spin / BackUp / Wait / ClearCostmap) and planning
    attempts/failures within the exploration window

Files read from each run directory:
  log.csv                  (required)  main time series + counter columns
  bt_counts.csv            (optional)  authoritative BT counters recovered
                                       from the rosbag by bt_counts_from_bag.py.
                                       If present, it OVERRIDES the counter
                                       columns of log.csv and adds planning
                                       failures and Wait counts.
  exploration_summary.csv  (optional)  goals_sent/reached/failed, timeouts,
                                       rtf, completion cross-check.
Other files in the run folder (plans/, recovery_summary.csv, ...) are NOT
used by this script; they are kept as archive material.

Usage (shells expand globs, so ..._rep* works):
    python3 analyze_exploration.py \
        --pmb2-run-dirs logs_explore/pmb2_expl_rep* \
        --omni-run-dirs logs_explore/omni_expl_rep* \
        --out-dir ./figures_explore

Dependencies: pandas, numpy, matplotlib
"""
import argparse
import math
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

plt.rcParams.update({
    'font.size': 13,
    'axes.titlesize': 15,
    'axes.labelsize': 13,
    'legend.fontsize': 11,
    'xtick.labelsize': 11,
    'ytick.labelsize': 11,
})

COLORS = {'pmb2': 'tab:blue', 'omni': 'tab:orange'}
LABELS = {'pmb2': 'PMB2', 'omni': 'OmniBase'}


# ---------------------------------------------------------------------- #
# Loading and time windows                                                #
# ---------------------------------------------------------------------- #
def load_log(path):
    df = pd.read_csv(path)
    df = df.sort_values('t').reset_index(drop=True)
    df['t'] = df['t'] - df['t'].iloc[0]
    return df


def load_summary(run_dir):
    p = os.path.join(run_dir, 'exploration_summary.csv')
    if os.path.exists(p):
        try:
            return pd.read_csv(p).iloc[0].to_dict()
        except Exception:
            return None
    return None


def load_bt_counts(run_dir):
    """Reads bt_counts.csv written by bt_counts_from_bag.py.
    Returns {(category, transition): count_window} or None if missing."""
    p = os.path.join(run_dir, 'bt_counts.csv')
    if not os.path.exists(p):
        return None
    try:
        df = pd.read_csv(p, comment='#')
        return {(str(r['category']), str(r['transition'])): float(r['count_window'])
                for _, r in df.iterrows()}
    except Exception as e:
        print(f'(!) Could not parse {p}: {e}')
        return None


def explore_window(df):
    """(t_start, t_end, completed):
    t_start = time of the first frontier goal (goal_idx >= 0),
    t_end   = completion signal (first goal_idx == -1), or the end of the
              log if the run never got there."""
    started = df.index[df['goal_idx'] >= 0]
    t_start = float(df['t'].iloc[started[0]]) if len(started) else 0.0
    done = df.index[df['goal_idx'] == -1]
    if len(done):
        return t_start, float(df['t'].iloc[done[0]]), True
    return t_start, float(df['t'].iloc[-1]), False


def window_df(df, t0, t1):
    return df[(df['t'] >= t0) & (df['t'] <= t1)].reset_index(drop=True)


# ---------------------------------------------------------------------- #
# Basic metrics (same definitions as analyze_results.py)                  #
# ---------------------------------------------------------------------- #
def path_length(d):
    dx = d['gt_x'].diff().fillna(0)
    dy = d['gt_y'].diff().fillna(0)
    return float(np.sqrt(dx**2 + dy**2).sum())


def smoothness_metric(d):
    """RMS of the commanded angular acceleration (lower = smoother).
    Samples with dt == 0 are skipped: the sim clock has a coarser
    granularity than the logging rate, so consecutive rows can share the
    same timestamp (would produce inf/NaN in the derivative)."""
    t = d['t'].values
    wz = d['cmd_wz'].values
    if len(t) < 3:
        return float('nan')
    dt = np.diff(t)
    ok = dt > 0
    if not ok.any():
        return float('nan')
    dwz = np.diff(wz)[ok] / dt[ok]
    return float(np.sqrt(np.nanmean(dwz**2)))


def linear_smoothness_metric(d):
    """RMS of the commanded linear acceleration (vx, vy)."""
    t = d['t'].values
    if len(t) < 3:
        return float('nan')
    dt = np.diff(t)
    ok = dt > 0
    if not ok.any():
        return float('nan')
    dvx = np.diff(d['cmd_vx'].values)[ok] / dt[ok]
    dvy = np.diff(d['cmd_vy'].values)[ok] / dt[ok]
    return float(np.sqrt(np.nanmean(dvx**2 + dvy**2)))


def lateral_usage(d):
    """Fraction of the commanded speed attributable to vy (~0 for PMB2)."""
    speed = np.sqrt(d['cmd_vx']**2 + d['cmd_vy']**2)
    nonzero = speed > 1e-3
    if nonzero.sum() == 0:
        return 0.0
    return float((d['cmd_vy'][nonzero].abs() / speed[nonzero]).mean())


# ---------------------------------------------------------------------- #
# Coverage curves and SLAM error                                          #
# ---------------------------------------------------------------------- #
def dedupe_time(t, y):
    """Strictly increasing timestamps (required by np.interp)."""
    t = np.asarray(t, float)
    y = np.asarray(y, float)
    if t.size == 0:
        return t, y
    keep = np.concatenate(([True], np.diff(t) > 0))
    return t[keep], y[keep]


def coverage_curve(d, t0):
    """(t_rel, area): explored area [m^2] vs time since exploration start."""
    t = d['t'].values - t0
    a = pd.to_numeric(d['explored_area_m2'], errors='coerce').ffill().fillna(0.0).values
    return dedupe_time(t, a)


def estimate_map_to_world(df, n_samples=20):
    """Constant SE(2) transform T_world<-map (pose of the SLAM map origin in
    the Gazebo world frame), estimated from the first samples where both the
    SLAM pose and the ground truth are valid (robot still near the start).

    Why estimated from data instead of assumed: slam_toolbox anchors the map
    frame at the ODOM origin, and the odom convention differs between the
    two bases. PMB2 wheel odometry is zero-initialised at spawn, so its map
    origin coincides with the spawn pose; the OmniBase planar-move plugin
    publishes world-referenced odometry, so its map origin coincides with
    the world origin. A per-run estimate handles both (and any other
    constant offset) without assumptions - equivalent to the standard
    ATE origin alignment. Returns None if no valid sample pair exists."""
    gx = pd.to_numeric(df['gt_x'], errors='coerce').values
    gy = pd.to_numeric(df['gt_y'], errors='coerce').values
    gyaw = pd.to_numeric(df['gt_yaw'], errors='coerce').values
    mx = pd.to_numeric(df['slam_x'], errors='coerce').values
    my = pd.to_numeric(df['slam_y'], errors='coerce').values
    myaw = pd.to_numeric(df['slam_yaw'], errors='coerce').values
    ok = (np.isfinite(gx) & np.isfinite(gy) & np.isfinite(gyaw)
          & np.isfinite(mx) & np.isfinite(my) & np.isfinite(myaw))
    idx = np.flatnonzero(ok)[:n_samples]
    if idx.size == 0:
        return None
    dyaw = gyaw[idx] - myaw[idx]
    yaw = math.atan2(float(np.mean(np.sin(dyaw))),
                     float(np.mean(np.cos(dyaw))))
    c, s = math.cos(yaw), math.sin(yaw)
    tx = float(np.mean(gx[idx] - (c * mx[idx] - s * my[idx])))
    ty = float(np.mean(gy[idx] - (s * mx[idx] + c * my[idx])))
    return (tx, ty, yaw)


def slam_error_series(d, t0, tf_wm):
    """|SLAM estimated position - ground truth| [m] over time.
    The ground truth (world frame) is mapped into the SLAM map frame using
    the constant transform tf_wm = T_world<-map: p_map = R(-yaw)(p_world-t).
    With tf_wm = (0,0,0) the transform is the identity."""
    sx = pd.to_numeric(d['slam_x'], errors='coerce').values
    sy = pd.to_numeric(d['slam_y'], errors='coerce').values
    cy, s = math.cos(tf_wm[2]), math.sin(tf_wm[2])
    dx = d['gt_x'].values - tf_wm[0]
    dy = d['gt_y'].values - tf_wm[1]
    gx = cy * dx + s * dy
    gy = -s * dx + cy * dy
    err = np.hypot(gx - sx, gy - sy)
    t = d['t'].values - t0
    ok = np.isfinite(err)
    return t[ok], err[ok]


def time_to_area(cov, target):
    """First time at which coverage reaches `target` m^2 (nan if never)."""
    t, a = cov
    if a.size == 0 or not np.isfinite(target) or a[-1] < target:
        return float('nan')
    i = int(np.argmax(a >= target))
    return float(t[i])


# ---------------------------------------------------------------------- #
# Single-run analysis                                                     #
# ---------------------------------------------------------------------- #
def analyze_run(run_dir, spawn_override=None):
    run_name = os.path.basename(run_dir.rstrip('/'))
    df = load_log(os.path.join(run_dir, 'log.csv'))
    summary = load_summary(run_dir)
    bt = load_bt_counts(run_dir)
    t0, t1, completed = explore_window(df)
    d = window_df(df, t0, t1)

    if spawn_override is not None:
        tf_wm = spawn_override
        src = 'manual override'
    else:
        tf_wm = estimate_map_to_world(df)
        src = 'estimated from data'
        if tf_wm is None:
            tf_wm = (0.0, 0.0, 0.0)
            src = 'identity (no valid SLAM/GT sample pair)'
    print(f'[{run_name}] map->world frame alignment: '
          f'x={tf_wm[0]:+.3f}, y={tf_wm[1]:+.3f}, yaw={tf_wm[2]:+.3f} '
          f'({src})')

    cov = coverage_curve(d, t0)
    terr, err = slam_error_series(d, t0, tf_wm)

    m = {
        'run': run_name,
        'completed': completed,
        'explore_time_s': t1 - t0,
        'path_length_m': path_length(d),
        'final_area_m2': float(cov[1][-1]) if cov[1].size else float('nan'),
        'smooth_ang': smoothness_metric(d),
        'smooth_lin': linear_smoothness_metric(d),
        'lateral_usage': lateral_usage(d),
        'slam_err_mean_m': float(np.nanmean(err)) if err.size else float('nan'),
        'slam_err_max_m': float(np.nanmax(err)) if err.size else float('nan'),
    }
    m['efficiency_m2_per_m'] = (m['final_area_m2'] / m['path_length_m']
                                if m['path_length_m'] > 1e-6 else float('nan'))
    m['efficiency_m2_per_s'] = (m['final_area_m2'] / m['explore_time_s']
                                if m['explore_time_s'] > 1e-6 else float('nan'))

    # Recovery / planning counters within the exploration window.
    # Precedence: bt_counts.csv (recovered from the rosbag, authoritative)
    # over the counter columns of log.csv (live logger, last row of window).
    if bt is not None:
        g = lambda cat, tr: bt.get((cat, tr), 0.0)

        def act(cat):
            # Activations = departures from IDLE. This also covers one-shot
            # service nodes (e.g. the costmap clears) that jump straight
            # from IDLE to SUCCESS without ever reporting RUNNING. Falls
            # back to RUNNING transitions for bt_counts.csv files written
            # by older versions of bt_counts_from_bag.py.
            key = (cat, 'ACTIVATIONS')
            return bt[key] if key in bt else g(cat, 'RUNNING')

        m['spins'] = act('spin')
        m['backups'] = act('backup')
        m['waits'] = act('wait')
        m['costmap_clears'] = (act('clear_global') + act('clear_local')
                               + act('clear_other'))
        m['replans'] = act('planning')
        m['plan_failures'] = g('planning', 'FAILURE')
        m['counters_source'] = 'bag'
    else:
        for col, key in [('spin_count', 'spins'), ('backup_count', 'backups'),
                         ('clearcostmap_count', 'costmap_clears'),
                         ('planning_attempts', 'replans')]:
            m[key] = float(d[col].iloc[-1]) if (col in d.columns and len(d)) \
                else float('nan')
        m['waits'] = float('nan')
        m['plan_failures'] = float('nan')
        m['counters_source'] = 'log'
        print(f'[{run_name}] no bt_counts.csv found: counters taken from '
              f'log.csv (run bt_counts_from_bag.py on the rosbag to recover '
              f'the authoritative values and the planning failures).')

    m['plan_fail_rate'] = (m['plan_failures'] / m['replans']
                           if np.isfinite(m.get('plan_failures', np.nan))
                           and np.isfinite(m.get('replans', np.nan))
                           and m['replans'] > 0 else float('nan'))

    if summary:
        for k in ('goals_sent', 'goals_reached', 'goals_failed', 'timeouts', 'rtf'):
            m[k] = summary.get(k, float('nan'))
        if not completed and bool(summary.get('exploration_completed', False)):
            # the log may have been closed before the -1: trust the summary
            m['completed'] = True

    return {'dir': run_dir, 'window': (t0, t1, completed), 'cov': cov,
            'err': (terr, err), 'metrics': m,
            'traj': (d['gt_x'].values, d['gt_y'].values)}


# ---------------------------------------------------------------------- #
# Figures                                                                 #
# ---------------------------------------------------------------------- #
def plot_coverage(runs_by_base, target_area, out_path):
    ends = [r['cov'][0][-1] for rs in runs_by_base.values() for r in rs
            if r['cov'][0].size]
    if not ends:
        print('(!) No coverage curve available: figure skipped.')
        return
    grid = np.arange(0.0, max(ends) + 1.0, 1.0)

    fig, ax = plt.subplots(figsize=(11, 6))
    for base, rs in runs_by_base.items():
        curves = []
        for r in rs:
            t, a = r['cov']
            if t.size < 2:
                continue
            # beyond the end of a run np.interp holds the last value:
            # consistent, since the area no longer changes once done
            y = np.interp(grid, t, a)
            curves.append(y)
            ax.plot(grid, y, color=COLORS[base], alpha=0.25, linewidth=1)
        if curves:
            arr = np.vstack(curves)
            mean, std = arr.mean(axis=0), arr.std(axis=0)
            ax.plot(grid, mean, color=COLORS[base], linewidth=2.5,
                    label=rf'{LABELS[base]} (mean $\pm$ std)')
            ax.fill_between(grid, mean - std, mean + std,
                            color=COLORS[base], alpha=0.15)
    if np.isfinite(target_area):
        ax.axhline(target_area, color='gray', linestyle='--', linewidth=1.2,
                   label=f'Target area ({target_area:.1f} m²)')
    ax.set_xlabel('Exploration time [s]')
    ax.set_ylabel('Explored area [m²]')
    ax.set_title('Map coverage over time')
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close(fig)


def plot_boxes(metrics_by_base, out_path):
    keys = [('explore_time_s', 'Exploration time [s]'),
            ('path_length_m', 'Path length [m]'),
            ('efficiency_m2_per_m', 'Efficiency [m²/m]')]
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    for ax, (k, lab) in zip(axes, keys):
        data = []
        for base in ('pmb2', 'omni'):
            vals = np.asarray([m[k] for m in metrics_by_base[base]], float)
            data.append(vals[np.isfinite(vals)])
        ax.boxplot(data)
        ax.set_xticks([1, 2])
        ax.set_xticklabels(['PMB2', 'OmniBase'])
        for i, (vals, base) in enumerate(zip(data, ('pmb2', 'omni')), start=1):
            ax.scatter([i] * len(vals), vals, color=COLORS[base], zorder=3)
        ax.set_title(lab)
        ax.grid(True, alpha=0.3, axis='y')
    fig.suptitle('Per-run exploration metrics', fontsize=16)
    fig.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close(fig)


def plot_slam_error(runs_by_base, out_path):
    any_data = any(r['err'][0].size >= 2
                   for rs in runs_by_base.values() for r in rs)
    if not any_data:
        print('(!) No map->base_footprint TF data in the logs: '
              'SLAM error figure skipped.')
        return
    fig, ax = plt.subplots(figsize=(11, 5.5))
    for base, rs in runs_by_base.items():
        series = [r['err'] for r in rs if r['err'][0].size >= 2]
        for t, e in series:
            ax.plot(t, e, color=COLORS[base], alpha=0.3, linewidth=1)
        if series:
            tmax = max(s[0][-1] for s in series)
            grid = np.arange(0.0, tmax + 1.0, 1.0)
            arr = np.vstack([np.interp(grid, s[0], s[1]) for s in series])
            ax.plot(grid, arr.mean(axis=0), color=COLORS[base], linewidth=2.5,
                    label=f'{LABELS[base]} (mean)')
    ax.set_xlabel('Exploration time [s]')
    ax.set_ylabel(r'$|\mathrm{SLAM\ pose} - \mathrm{ground\ truth}|$ [m]')
    ax.set_title('SLAM localization error during exploration')
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close(fig)


def plot_recoveries(metrics_by_base, out_path):
    behaviors = [('spins', 'Spin'), ('backups', 'BackUp'), ('waits', 'Wait'),
                 ('costmap_clears', 'ClearCostmap'),
                 ('plan_failures', 'Plan failures')]
    x = np.arange(len(behaviors))
    width = 0.35
    fig, ax = plt.subplots(figsize=(10, 5))
    for off, base in [(-width / 2, 'pmb2'), (width / 2, 'omni')]:
        means, stds = [], []
        for k, _ in behaviors:
            vals = np.asarray([m.get(k, np.nan) for m in metrics_by_base[base]],
                              float)
            vals = vals[np.isfinite(vals)]
            means.append(vals.mean() if vals.size else np.nan)
            stds.append(vals.std() if vals.size else np.nan)
        ax.bar(x + off, means, width, yerr=stds, capsize=4,
               label=LABELS[base], color=COLORS[base])
    ax.set_xticks(x)
    ax.set_xticklabels([b for _, b in behaviors])
    ax.set_ylabel('Mean count per run')
    ax.set_title('Recovery behaviours and planning failures (exploration phase)')
    ax.grid(True, alpha=0.3, axis='y')
    ax.legend()
    fig.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close(fig)


def plot_trajectories(runs_by_base, out_path):
    fig, axes = plt.subplots(1, 2, figsize=(16, 8), sharex=True, sharey=True)
    for ax, base in zip(axes, ('pmb2', 'omni')):
        for i, r in enumerate(runs_by_base[base]):
            gx, gy = r['traj']
            if len(gx) == 0:
                continue
            ax.plot(gx, gy, alpha=0.8, linewidth=1.2, label=f'Run {i+1}')
            ax.scatter([gx[0]], [gy[0]], marker='*', s=180,
                       color='black', zorder=5)
        ax.set_title(LABELS[base])
        ax.set_xlabel(r'$x$ [m]')
        ax.set_ylabel(r'$y$ [m]')
        ax.set_aspect('equal', adjustable='box')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
    fig.suptitle(r'Exploration trajectories (ground truth, star = start)',
                 fontsize=16)
    fig.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close(fig)


# ---------------------------------------------------------------------- #
# Main                                                                    #
# ---------------------------------------------------------------------- #
def main():
    p = argparse.ArgumentParser()
    p.add_argument('--pmb2-run-dirs', nargs='+', required=True,
                   help='One or more PMB2 run folders (logs_explore/pmb2_...)')
    p.add_argument('--omni-run-dirs', nargs='+', required=True,
                   help='One or more OmniBase run folders')
    p.add_argument('--out-dir', default='./figures_explore')
    p.add_argument('--spawn-x', type=float, default=None,
                   help='Pose of the SLAM map origin in the Gazebo world '
                        'frame (T_world<-map). Default: estimated PER RUN '
                        'from the first valid SLAM/ground-truth sample pair '
                        '(ATE-style origin alignment). Pass explicit values '
                        'only to force it for ALL runs.')
    p.add_argument('--spawn-y', type=float, default=None)
    p.add_argument('--spawn-yaw', type=float, default=None)
    p.add_argument('--coverage-frac', type=float, default=0.95,
                   help='Fraction of the MINIMUM final area used as the '
                        'common target for time-to-coverage (default 0.95)')
    args = p.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    if args.spawn_x is None and args.spawn_y is None and args.spawn_yaw is None:
        spawn_override = None   # auto-detect per run
    else:
        spawn_override = (args.spawn_x or 0.0,
                          args.spawn_y or 0.0,
                          args.spawn_yaw or 0.0)

    runs = {'pmb2': [analyze_run(d, spawn_override) for d in args.pmb2_run_dirs],
            'omni': [analyze_run(d, spawn_override) for d in args.omni_run_dirs]}

    # Common target area: fraction of the minimum final area across ALL
    # runs, so every run reaches it and times are compared at equal coverage
    finals = [r['metrics']['final_area_m2'] for rs in runs.values() for r in rs]
    finals = [f for f in finals if np.isfinite(f)]
    target = args.coverage_frac * min(finals) if finals else float('nan')
    for rs in runs.values():
        for r in rs:
            r['metrics']['time_to_target_s'] = time_to_area(r['cov'], target)

    mtr = {b: [r['metrics'] for r in rs] for b, rs in runs.items()}

    # --- per-run table ---
    if np.isfinite(target):
        print(f"\nCommon target area for 'time_to_target': {target:.1f} m² "
              f"({args.coverage_frac * 100:.0f}% of the minimum final area)\n")
    show_cols = ['run', 'completed', 'explore_time_s', 'time_to_target_s',
                 'path_length_m', 'final_area_m2', 'efficiency_m2_per_m',
                 'slam_err_mean_m', 'spins', 'backups', 'replans',
                 'plan_failures']
    for base in ('pmb2', 'omni'):
        tab = pd.DataFrame(mtr[base])
        cols = [c for c in show_cols if c in tab.columns]
        print(f'=== {LABELS[base]} ===')
        with pd.option_context('display.width', 220):
            print(tab[cols].to_string(index=False))
        print()

    # --- aggregates mean ± std ---
    print('=== Aggregates (mean ± std across runs) ===')
    num_keys = ['explore_time_s', 'time_to_target_s', 'path_length_m',
                'final_area_m2', 'efficiency_m2_per_m', 'efficiency_m2_per_s',
                'smooth_ang', 'smooth_lin', 'lateral_usage',
                'slam_err_mean_m', 'slam_err_max_m',
                'spins', 'backups', 'waits', 'costmap_clears',
                'replans', 'plan_failures', 'plan_fail_rate']
    for k in num_keys:
        line = f'{k:24s}'
        for base in ('pmb2', 'omni'):
            vals = np.asarray([m.get(k, np.nan) for m in mtr[base]], float)
            vals = vals[np.isfinite(vals)]
            if vals.size:
                line += f'  {base.upper():5s}={vals.mean():9.3f} ± {vals.std():7.3f}'
            else:
                line += f'  {base.upper():5s}=      n/a          '
        print(line)

    # --- figures ---
    plot_coverage(runs, target, os.path.join(args.out_dir, 'coverage_vs_time.png'))
    plot_boxes(mtr, os.path.join(args.out_dir, 'exploration_metrics_box.png'))
    plot_slam_error(runs, os.path.join(args.out_dir, 'slam_error_vs_time.png'))
    plot_recoveries(mtr, os.path.join(args.out_dir, 'recovery_counts.png'))
    plot_trajectories(runs, os.path.join(args.out_dir, 'exploration_trajectories.png'))
    print(f'\nFigures saved in {args.out_dir}/')


if __name__ == '__main__':
    main()
