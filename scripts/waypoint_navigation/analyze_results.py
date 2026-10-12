#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze_results.py  (fixed + English figures)
=============================================
Offline analysis of ONE pair of waypoint-following runs (PMB2 vs OmniBase),
produced by metrics_logger.py + send_waypoints.py. Use it for the in-depth
qualitative figures on a representative (median) run pair; the mean/std
statistics over the 5 repetitions per base are produced by
aggregate_waypoints.py, which imports the metric functions defined here.

Usage:
    python3 analyze_results.py \
        --pmb2-run-dir logs/pmb2_001 \
        --omni-run-dir logs/omni_001 \
        --waypoints waypoints_hallway.yaml \
        --out-dir ./figures

Each run dir is the one created by metrics_logger.py / send_waypoints.py:
    <out-dir-logger>/<base_type>_<run_id>/
        log.csv                main time series
        run_summary.csv        written by send_waypoints.py (sim duration, rtf)
        plans/wp<N>.csv        last Nav2 /plan received while leg N was active
        bt_counts.csv          (optional) authoritative BT counters recovered
                               from the rosbag by bt_counts_from_bag.py

All metrics are computed over the RUN WINDOW: from the first row with
waypoint_idx >= 0 (first goal active) to the first row with waypoint_idx
== -1 (completion signal from send_waypoints.py). Rows logged before the
task starts or after it ends are excluded.

Recovery / planning counters: if bt_counts.csv exists it is the
authoritative source (recovered offline from /behavior_tree_log in the
rosbag, incl. Wait and planning failures); otherwise the counter columns
of log.csv are used, with a warning (the live logger had a status-string
bug, so pre-fix logs contain zeros there).

Dependencies: pandas, numpy, matplotlib, pyyaml
"""
import argparse
import math
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml

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
# Loading and run window                                                  #
# ---------------------------------------------------------------------- #
def load_log(path):
    df = pd.read_csv(path)
    df = df.sort_values('t').reset_index(drop=True)
    df['t'] = df['t'] - df['t'].iloc[0]   # time relative to logger start
    return df


def load_waypoints(path):
    with open(path) as f:
        data = yaml.safe_load(f)
    return data['waypoints']


def load_run_summary(run_dir):
    """run_summary.csv written by send_waypoints.py (or None)."""
    p = os.path.join(run_dir, 'run_summary.csv')
    if not os.path.exists(p):
        return None
    try:
        return pd.read_csv(p).iloc[0].to_dict()
    except Exception:
        return None


def run_window(df):
    """(t_start, t_end, completed): from the first active waypoint
    (waypoint_idx >= 0) to the completion signal (first waypoint_idx == -1),
    falling back to the end of the log if -1 was never seen."""
    started = df.index[df['waypoint_idx'] >= 0]
    t_start = float(df['t'].iloc[started[0]]) if len(started) else 0.0
    done = df.index[df['waypoint_idx'] == -1]
    if len(done):
        return t_start, float(df['t'].iloc[done[0]]), True
    return t_start, float(df['t'].iloc[-1]), False


def window_df(df, t0, t1):
    return df[(df['t'] >= t0) & (df['t'] <= t1)].reset_index(drop=True)


# ---------------------------------------------------------------------- #
# Recovery / planning counters                                            #
# ---------------------------------------------------------------------- #
def load_bt_counters(run_dir, d):
    """Counters for the run. Preferred source: bt_counts.csv (recovered
    from the rosbag, window-scoped, includes Wait and planning failures).
    Fallback: the counter columns of log.csv at the end of the window
    (unreliable for logs recorded with the pre-fix live logger)."""
    out = {'spins': np.nan, 'backups': np.nan, 'waits': np.nan,
           'costmap_clears': np.nan, 'replans': np.nan,
           'plan_failures': np.nan, 'plan_fail_rate': np.nan,
           'counters_source': 'none'}
    p = os.path.join(run_dir, 'bt_counts.csv')
    if os.path.exists(p):
        try:
            bt = pd.read_csv(p, comment='#')
            m = {(str(r['category']), str(r['transition'])): float(r['count_window'])
                 for _, r in bt.iterrows()}

            def act(cat):
                # activations (departures from IDLE) cover one-shot service
                # nodes that never report RUNNING; fall back to RUNNING for
                # files written by older versions of bt_counts_from_bag.py
                return m.get((cat, 'ACTIVATIONS'), m.get((cat, 'RUNNING'), 0.0))

            out.update({
                'spins': act('spin'), 'backups': act('backup'),
                'waits': act('wait'),
                'costmap_clears': act('clear_global') + act('clear_local')
                                  + act('clear_other'),
                'replans': act('planning'),
                'plan_failures': m.get(('planning', 'FAILURE'), 0.0),
                'counters_source': 'bag'})
        except Exception as e:
            print(f'(!) Could not parse {p}: {e}')
    if out['counters_source'] != 'bag':
        cols = [('spin_count', 'spins'), ('backup_count', 'backups'),
                ('clearcostmap_count', 'costmap_clears'),
                ('planning_attempts', 'replans')]
        if all(c in d.columns for c, _ in cols) and len(d):
            for c, k in cols:
                out[k] = float(d[c].iloc[-1])
            if 'wait_count' in d.columns:
                out['waits'] = float(d['wait_count'].iloc[-1])
            out['counters_source'] = 'log'
            if all(out[k] == 0 for _, k in cols):
                print(f'(!) [{os.path.basename(run_dir.rstrip("/"))}] all '
                      f'log.csv counters are zero and no bt_counts.csv was '
                      f'found: if this run predates the logger fix, recover '
                      f'the counters from its rosbag with '
                      f'bt_counts_from_bag.py before trusting them.')
    if np.isfinite(out['plan_failures']) and np.isfinite(out['replans']) \
            and out['replans'] > 0:
        out['plan_fail_rate'] = out['plan_failures'] / out['replans']
    return out


# ---------------------------------------------------------------------- #
# Scalar metrics (shared with aggregate_waypoints.py)                     #
# ---------------------------------------------------------------------- #
def path_length(df):
    dx = df['gt_x'].diff().fillna(0)
    dy = df['gt_y'].diff().fillna(0)
    return float(np.sqrt(dx**2 + dy**2).sum())


def straight_line_length(waypoints, start_xy):
    pts = [start_xy] + [(w['x'], w['y']) for w in waypoints]
    total = 0.0
    for (x0, y0), (x1, y1) in zip(pts[:-1], pts[1:]):
        total += math.hypot(x1 - x0, y1 - y0)
    return total


def waypoint_errors(df, waypoints):
    """For each waypoint, the CLOSEST APPROACH of the trajectory (minimum
    distance over all logged points) and the heading error at that closest
    point.

    Why closest approach, and not 'the last row while this waypoint_idx was
    active': with goThroughPoses a fast base can have Nav2 retire two poses
    between two feedback cycles, so an intermediate waypoint index is
    sometimes never published - the old rule then returned NaN for it (this
    is what produced the 'Mean of empty slice' warnings on the Angle path).
    Closest approach is robust to that, and for a pass-through / corner
    waypoint it is the honest measure of how near the robot actually passed
    (= the corner-cutting distance). For the final stop waypoint it
    coincides with the final positioning error. It is also free of the
    timing noise of the old rule (which measured wherever the robot happened
    to be at the instant Nav2 moved on, not the true nearest point)."""
    errors = []
    gx = df['gt_x'].to_numpy(float)
    gy = df['gt_y'].to_numpy(float)
    gyaw = df['gt_yaw'].to_numpy(float)
    logged = set(int(v) for v in df['waypoint_idx'].unique())
    for i, wp in enumerate(waypoints):
        if len(gx) == 0:
            errors.append({'waypoint_idx': i, 'pos_err': np.nan,
                           'yaw_err': np.nan, 'idx_logged': i in logged})
            continue
        dist = np.hypot(gx - wp['x'], gy - wp['y'])
        k = int(np.argmin(dist))
        yaw_err = abs(math.atan2(math.sin(gyaw[k] - wp['yaw']),
                                 math.cos(gyaw[k] - wp['yaw'])))
        errors.append({'waypoint_idx': i, 'pos_err': float(dist[k]),
                       'yaw_err': yaw_err, 'idx_logged': i in logged})
    out = pd.DataFrame(errors)
    missing = [i for i in range(len(waypoints)) if i not in logged]
    if missing:
        print(f'  (note: waypoint index/es {missing} were never published in '
              f'the log window - Nav2 goThroughPoses can retire two poses '
              f'between feedback cycles for a fast base; the closest-approach '
              f'error is still reported for them.)')
    # Frame-offset sentinel: under a map<->world offset (spawn pose / map
    # anchoring, cf. PMB2 exploration at y=-1.3 m) the robot stops at
    # waypoint+offset for EVERY leg, so the point-to-point errors are all
    # large and nearly identical. Real navigation inaccuracy is neither.
    pos = out['pos_err'].to_numpy(float)
    pos = pos[np.isfinite(pos)]
    if len(pos) >= 2 and pos.mean() > 0.5 and pos.std() < 0.2 * pos.mean():
        print(f'(!) per-waypoint position errors are all large and nearly '
              f'identical (mean {pos.mean():.2f} m, std {pos.std():.2f} m): '
              f'classic signature of a map<->world frame offset, NOT of '
              f'navigation accuracy. Diagnose the frames (spawn pose, map '
              f'anchoring) before using these numbers in the thesis.')
    return out


def smoothness_metric(df):
    """RMS of the commanded angular acceleration (lower = smoother).
    Samples with dt == 0 are skipped: the sim clock has coarser granularity
    than the logging rate, so consecutive rows can share a timestamp."""
    t = df['t'].values
    wz = df['cmd_wz'].values
    if len(t) < 3:
        return float('nan')
    dt = np.diff(t)
    ok = dt > 0
    if not ok.any():
        return float('nan')
    dwz = np.diff(wz)[ok] / dt[ok]
    return float(np.sqrt(np.nanmean(dwz**2)))


def linear_smoothness_metric(df):
    """RMS of the commanded linear acceleration (vx, vy). Complementary to
    the angular one: a robot can be smooth in rotation and jerky in
    translation, or vice versa."""
    t = df['t'].values
    if len(t) < 3:
        return float('nan')
    dt = np.diff(t)
    ok = dt > 0
    if not ok.any():
        return float('nan')
    dvx = np.diff(df['cmd_vx'].values)[ok] / dt[ok]
    dvy = np.diff(df['cmd_vy'].values)[ok] / dt[ok]
    return float(np.sqrt(np.nanmean(dvx**2 + dvy**2)))


def lateral_usage(df):
    """Fraction of the commanded speed attributable to vy (only meaningful
    for holonomic bases; ~0 for the differential drive)."""
    speed = np.sqrt(df['cmd_vx']**2 + df['cmd_vy']**2)
    nonzero = speed > 1e-3
    if nonzero.sum() == 0:
        return 0.0
    return float((df['cmd_vy'][nonzero].abs() / speed[nonzero]).mean())


# ---------------------------------------------------------------------- #
# Tracking error vs the Nav2 global plan                                  #
# ---------------------------------------------------------------------- #
def compute_cross_track_error(df, waypoints):
    """Distance from the STRAIGHT-LINE reference (leg start -> leg goal).
    Used only as a fallback when no /plan was saved for a leg: a straight
    line crossing an obstacle would report a 'deviation' that is actually
    a necessary avoidance, so prefer compute_plan_tracking_error.

    Returns (error, arclen), both aligned to df."""
    n = len(df)
    error = np.full(n, np.nan)
    leg_start_xy = (df['gt_x'].iloc[0], df['gt_y'].iloc[0])

    for i, wp in enumerate(waypoints):
        mask = (df['waypoint_idx'] == i).values
        if not mask.any():
            continue
        x0, y0 = leg_start_xy
        x1, y1 = wp['x'], wp['y']
        seg_len = math.hypot(x1 - x0, y1 - y0)
        xs = df['gt_x'].values[mask]
        ys = df['gt_y'].values[mask]
        if seg_len < 1e-6:
            dist = np.hypot(xs - x0, ys - y0)
        else:
            dist = np.abs((xs - x0) * (y1 - y0)
                          - (ys - y0) * (x1 - x0)) / seg_len
        error[mask] = dist
        leg_start_xy = (x1, y1)

    dx = df['gt_x'].diff().fillna(0)
    dy = df['gt_y'].diff().fillna(0)
    arclen = np.sqrt(dx**2 + dy**2).cumsum().values
    return error, arclen


def load_plan(plans_dir, wp_idx):
    """Global /plan saved by metrics_logger.py for leg wp_idx (or None)."""
    path = os.path.join(plans_dir, f'wp{wp_idx}.csv')
    if not os.path.exists(path):
        return None
    plan_df = pd.read_csv(path)
    if len(plan_df) < 2:
        return None
    return plan_df


def min_dist_to_polyline(pts, poly, chunk=2000):
    """Minimum distance of each point in pts (N,2) from the polyline poly
    (M,2). Vectorised and chunked to keep memory bounded."""
    a = poly[:-1]
    ab = poly[1:] - a
    ab2 = (ab ** 2).sum(axis=1)
    ab2[ab2 < 1e-12] = 1e-12
    out = np.empty(len(pts))
    for s in range(0, len(pts), chunk):
        p = pts[s:s + chunk]
        ap = p[:, None, :] - a[None, :, :]
        tt = np.clip((ap * ab[None, :, :]).sum(-1) / ab2[None, :], 0.0, 1.0)
        proj = a[None, :, :] + tt[..., None] * ab[None, :, :]
        d2 = ((p[:, None, :] - proj) ** 2).sum(-1)
        out[s:s + chunk] = np.sqrt(d2.min(axis=1))
    return out


def compute_plan_tracking_error(df, waypoints, plans_dir):
    """'True' cross-track error: for each point of the real trajectory, the
    minimum distance from the Nav2 global plan (/plan) of the leg active at
    that moment. Falls back to the straight-line reference (and says so)
    only for legs whose plan file is missing."""
    error, arclen = compute_cross_track_error(df, waypoints)  # base fallback
    used_fallback = []
    for i in range(len(waypoints)):
        mask = (df['waypoint_idx'] == i).values
        if not mask.any():
            continue
        plan_df = load_plan(plans_dir, i)
        if plan_df is None:
            used_fallback.append(i)
            continue
        pts = np.column_stack([df['gt_x'].values[mask],
                               df['gt_y'].values[mask]])
        error[mask] = min_dist_to_polyline(pts, plan_df[['x', 'y']].values)
    if used_fallback:
        print(f'  (note: legs {used_fallback} have no saved /plan in '
              f'{plans_dir} - straight-line fallback used for those legs)')
    # Soft sentinel: with a healthy Nav2 setup the cross-track error stays
    # well below 0.5 m. Note that a pure map<->world TRANSLATION does not
    # show up here as a constant error (legs parallel to the offset absorb
    # it): its clean signature is in the per-waypoint errors, checked in
    # waypoint_errors().
    med = float(np.nanmedian(error))
    if np.isfinite(med) and med > 0.5:
        run = os.path.basename(os.path.dirname(plans_dir.rstrip('/'))) \
            or plans_dir
        print(f'(!) [{run}] tracking error is unusually large '
              f'(median {med:.2f} m). Two known non-navigation causes: '
              f'(a) plans/wpN.csv holds only the LAST replan of the leg (a '
              f'short stub near the leg end, giving a distance-to-goal ramp '
              f'instead of a cross-track) -> re-extract the plans from the '
              f'rosbag with bt_counts_from_bag.py v5 (--patch-log); '
              f'(b) a map<->world frame offset -> check whether the '
              f'per-waypoint errors are also large and nearly identical.')
    return error, arclen


# ---------------------------------------------------------------------- #
# Wheel data (with the two documented simulation caveats)                 #
# ---------------------------------------------------------------------- #
def reconstruct_omni_wheel_data(df, wheel_radius=0.0762, a_plus_b=0.467575):
    """KINEMATIC reconstruction of the OmniBase's 4 mecanum wheels.

    WHY: in simulation the OmniBase is moved by gazebo_ros_planar_move,
    which imposes the body twist directly and does NOT actuate the wheels;
    the wheel joints do not appear in /joint_states (verified on real
    logs), so there is no measured data to plot. Wheel rates are therefore
    reconstructed by applying the inverse mecanum kinematics (thesis
    Chapter 2) to the commanded twist (order FL, FR, RL, RR):

        | w_FL |       | 1  -1  -(a+b) | | vx |
        | w_FR | = 1/R | 1   1   (a+b) | | vy |
        | w_RL |       | 1   1  -(a+b) | | wz |
        | w_RR |       | 1  -1   (a+b) |

    Defaults from base.urdf.xacro: R = 0.0762 m, a+b = 0.467575 m.

    THESIS NOTE: this is the kinematically IDEAL (no-slip) wheel rotation
    required by the trajectory - the simulation does not physically model
    the mecanum rollers. The differential drive wheels, instead, come from
    real simulated encoders. Position is obtained by trapezoidal
    integration of the rate over time (cumulative, like a real continuous
    joint). Returns (pos_series, vel_series, derived_joints) in the same
    format as parse_joint_series."""
    t = df['t'].values
    vx = df['cmd_vx'].values
    vy = df['cmd_vy'].values
    wz = df['cmd_wz'].values
    inv_r = 1.0 / wheel_radius
    k = a_plus_b
    vel_series = {
        'Front left':  inv_r * (vx - vy - k * wz),
        'Front right': inv_r * (vx + vy + k * wz),
        'Rear left':   inv_r * (vx + vy - k * wz),
        'Rear right':  inv_r * (vx - vy + k * wz),
    }
    pos_series = {}
    for jn, w in vel_series.items():
        pos = np.zeros_like(w)
        if len(t) > 1:
            dt = np.diff(t)
            pos[1:] = np.cumsum(0.5 * (w[1:] + w[:-1]) * dt)
        pos_series[jn] = pos
    return pos_series, vel_series, list(vel_series.keys())


def parse_joint_series(df, keyword='wheel'):
    """Extracts, for every joint whose name contains `keyword`, the
    position and velocity time series aligned to df['t']. Matching by name
    (not by fixed index) protects against joint reordering between
    /joint_states messages. Returns two dicts {joint: np.array(len(df))}."""
    n = len(df)
    pos_series, vel_series = {}, {}
    names_col = df['joint_names'].fillna('')
    pos_col = (df['joint_positions'].fillna('')
               if 'joint_positions' in df else pd.Series([''] * n))
    vel_col = df['joint_velocities'].fillna('')
    for row_i in range(n):
        names_str = names_col.iloc[row_i]
        if not names_str:
            continue
        names = names_str.split(';')
        positions = pos_col.iloc[row_i].split(';') if pos_col.iloc[row_i] else []
        velocities = vel_col.iloc[row_i].split(';') if vel_col.iloc[row_i] else []
        for j, jn in enumerate(names):
            if keyword.lower() not in jn.lower():
                continue
            if jn not in pos_series:
                pos_series[jn] = np.full(n, np.nan)
                vel_series[jn] = np.full(n, np.nan)
            if j < len(positions) and positions[j] != '':
                pos_series[jn][row_i] = float(positions[j])
            if j < len(velocities) and velocities[j] != '':
                vel_series[jn][row_i] = float(velocities[j])
    return pos_series, vel_series


def derive_velocity_from_position(t, position):
    """Velocity of a continuous joint by central finite differences on the
    position (np.gradient). Duplicated sim-clock timestamps are skipped
    (np.gradient would divide by zero there); those samples stay NaN."""
    pos = np.asarray(position, dtype=float)
    t = np.asarray(t, dtype=float)
    vel = np.full_like(pos, np.nan)
    valid = np.flatnonzero(~np.isnan(pos))
    if valid.size < 2:
        return vel
    keep = valid[np.concatenate(([True], np.diff(t[valid]) > 0))]
    if keep.size < 2:
        return vel
    vel[keep] = np.gradient(pos[keep], t[keep])
    return vel


def fix_wheel_velocities(df, pos_series, vel_series, label,
                         nan_frac_threshold=0.5):
    """CONFIRMED BUG (seen on real PMB2 data, 29/06): in TIAGo/Gazebo
    Classic simulation the `velocity` field of /joint_states for the drive
    wheel joints returns NaN - a known ros2_control plugin limitation, not
    a logging error. The `position` field remains valid. For every joint
    whose velocity is NaN for more than `nan_frac_threshold` of the
    samples, replace it with the numerical derivative of the position.
    Applied identically to both bases (independent of the planar_move
    caveat). Returns (fixed_vel_series, derived_joint_names)."""
    t = df['t'].values
    fixed, derived_joints = {}, []
    for jn, vel in vel_series.items():
        if np.isnan(vel).mean() > nan_frac_threshold:
            fixed[jn] = derive_velocity_from_position(t, pos_series[jn])
            derived_joints.append(jn)
        else:
            fixed[jn] = vel
    if derived_joints:
        print(f"[{label}] unreliable (NaN) 'velocity' field for: "
              f'{derived_joints} -> velocity derived numerically from '
              f"'position'. Cite it in the thesis as a known ros2_control "
              f'simulation limitation, not a pipeline anomaly.')
    return fixed, derived_joints


def check_wheel_data_validity(df, pos_series, vel_series, label):
    """Two-level check, in the right order:
    1) is the velocity field reliable? (otherwise fix_wheel_velocities
       derives it from position)
    2) even AFTER the fix, are the wheels consistent with cmd_vel? If they
       stay still while cmd_vel indicates motion, it is the
       gazebo_ros_planar_move plugin moving the body without physically
       animating the wheels (OmniBase caveat).
    Returns (fixed_vel_series, derived_joint_names)."""
    if not vel_series:
        print(f"[{label}] no 'wheel' joint found in the logs: check the "
              f'real names with: ros2 topic echo /joint_states --once')
        return vel_series, []
    fixed_vel, derived_joints = fix_wheel_velocities(df, pos_series,
                                                     vel_series, label)
    max_cmd = float(np.sqrt(df['cmd_vx']**2 + df['cmd_vy']**2
                            + df['cmd_wz']**2).max())
    wheel_maxes = [np.nanmax(np.abs(v)) for v in fixed_vel.values()]
    max_wheel = np.nanmax(wheel_maxes) if wheel_maxes else np.nan
    if max_cmd > 0.05 and (np.isnan(max_wheel) or max_wheel < 1e-3):
        print(f'!! WARNING [{label}]: cmd_vel indicates motion '
              f'(max={max_cmd:.2f}) but wheel joint velocities stay ~0 even '
              f'after the NaN fix. Likely cause: gazebo_ros_planar_move '
              f'moves the body without animating the wheels - the wheel '
              f'rotation figure for this base does NOT represent the real '
              f'kinematics and must be discussed as a known limitation.')
    return fixed_vel, derived_joints


# ---------------------------------------------------------------------- #
# Adaptive layout for metric x/y (trajectory-like) figures                 #
# ---------------------------------------------------------------------- #
def geo_layout(xy_parts, waypoints=None, n_panels=1, height=9.0,
               min_panel_w=3.8, max_panel_w=9.5, max_fig_w=19.0):
    """Figure size and axis limits for equal-aspect metric plots that stay
    readable for ANY path shape. matplotlib's default reaction to a long
    thin path (e.g. a 0.5 m x 13 m corridor) is to shrink the axes box to
    a needle; here instead the SHORTER data range is symmetrically expanded
    until the data aspect matches a sensibly-sized panel, so the box always
    fills the figure. Returns (xlim, ylim, figsize, narrow)."""
    xs = np.concatenate([np.asarray(p[0], float) for p in xy_parts])
    ys = np.concatenate([np.asarray(p[1], float) for p in xy_parts])
    if waypoints:
        xs = np.concatenate([xs, np.asarray([w['x'] for w in waypoints])])
        ys = np.concatenate([ys, np.asarray([w['y'] for w in waypoints])])
    x0, x1 = float(np.nanmin(xs)), float(np.nanmax(xs))
    y0, y1 = float(np.nanmin(ys)), float(np.nanmax(ys))
    pad = 0.05 * max(x1 - x0, y1 - y0, 1.0)
    x0, x1, y0, y1 = x0 - pad, x1 + pad, y0 - pad, y1 + pad
    xr, yr = x1 - x0, y1 - y0
    panel_w = float(np.clip(xr / yr * height, min_panel_w, max_panel_w))
    fig_w = min(max_fig_w, n_panels * panel_w + 1.6)
    panel_w = (fig_w - 1.6) / n_panels
    target = panel_w / height              # width/height of each panel box
    if xr / yr < target:                   # corridor-like: widen x
        cx, half = 0.5 * (x0 + x1), 0.5 * target * yr
        x0, x1 = cx - half, cx + half
    else:                                  # flat and wide: extend y
        cy, half = 0.5 * (y0 + y1), 0.5 * xr / target
        y0, y1 = cy - half, cy + half
    return (x0, x1), (y0, y1), (fig_w, height + 0.8), panel_w < 5.0


def geo_finish(ax, xlim, ylim):
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True, alpha=0.3)


def geo_legend(ax, narrow, **kw):
    """Inside the axes when there is room, below them for narrow panels
    (where an inner legend would cover the whole path)."""
    if narrow:
        opts = dict(loc='upper center', bbox_to_anchor=(0.5, -0.06),
                    ncol=2, fontsize=9, frameon=False)
        opts.update(kw)
        ax.legend(**opts)
    else:
        ax.legend(loc='best', **kw)


# ---------------------------------------------------------------------- #
# Figures                                                                 #
# ---------------------------------------------------------------------- #
def plot_trajectories(df_pmb2, df_omni, waypoints, out_path,
                      quiver_every=20, plans_dirs=None):
    xlim, ylim, figsize, narrow = geo_layout(
        [(df_pmb2['gt_x'], df_pmb2['gt_y']),
         (df_omni['gt_x'], df_omni['gt_y'])], waypoints, n_panels=1)
    fig, ax = plt.subplots(figsize=figsize)
    ax.plot(df_pmb2['gt_x'].to_numpy(), df_pmb2['gt_y'].to_numpy(),
            label=LABELS['pmb2'], color=COLORS['pmb2'])
    ax.plot(df_omni['gt_x'].to_numpy(), df_omni['gt_y'].to_numpy(),
            label=LABELS['omni'], color=COLORS['omni'])
    for df, color in [(df_pmb2, COLORS['pmb2']), (df_omni, COLORS['omni'])]:
        idx = np.arange(0, len(df), quiver_every)
        ax.quiver(df['gt_x'].values[idx], df['gt_y'].values[idx],
                  np.cos(df['gt_yaw'].values[idx]),
                  np.sin(df['gt_yaw'].values[idx]),
                  color=color, alpha=0.5, scale=20, width=0.003)
    if plans_dirs:
        for base in ('pmb2', 'omni'):
            shown = False
            for i in range(len(waypoints)):
                plan_df = load_plan(plans_dirs[base], i)
                if plan_df is None:
                    continue
                ax.plot(plan_df['x'].to_numpy(), plan_df['y'].to_numpy(),
                        '--', linewidth=1.1,
                        color=COLORS[base], alpha=0.75, zorder=1,
                        label=(f'{LABELS[base]} Nav2 plan'
                               if not shown else None))
                shown = True
    wx = [w['x'] for w in waypoints]
    wy = [w['y'] for w in waypoints]
    ax.scatter(wx, wy, marker='*', s=200, color='black', zorder=5,
               label='Waypoint target')
    for i, (x, y) in enumerate(zip(wx, wy)):
        ax.annotate(str(i), (x, y), textcoords='offset points', xytext=(5, 5))
    ax.set_xlabel(r'$x$ [m]')
    ax.set_ylabel(r'$y$ [m]')
    ax.set_title('Trajectory: PMB2 vs OmniBase')
    geo_finish(ax, xlim, ylim)
    geo_legend(ax, narrow)
    fig.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close(fig)


def plot_velocity_profiles(df_pmb2, df_omni, out_path):
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    for df, label, color in [(df_pmb2, 'PMB2', COLORS['pmb2']),
                             (df_omni, 'OmniBase', COLORS['omni'])]:
        t = df['t'].to_numpy()
        axes[0].plot(t, df['cmd_vx'].to_numpy(), label=label, color=color)
        axes[1].plot(t, df['cmd_vy'].to_numpy(), label=label, color=color)
        axes[2].plot(t, df['cmd_wz'].to_numpy(), label=label, color=color)
    axes[0].set_ylabel(r'$v_x$ [m/s]')
    axes[1].set_ylabel(r'$v_y$ [m/s]')
    axes[2].set_ylabel(r'$\omega_z$ [rad/s]')
    axes[2].set_xlabel('Time [s]')
    for ax in axes:
        ax.grid(True, alpha=0.3)
        ax.legend()
    axes[0].set_title('Commanded velocity profiles')
    fig.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close(fig)


def plot_cross_track_2d_compare(df_pmb2, df_omni, waypoints,
                                err_pmb2, err_omni, out_path):
    """Real trajectory coloured by the tracking error (vs the Nav2 /plan,
    straight-line fallback where needed), PMB2 and OmniBase side by side
    with the SAME colour scale (otherwise the visual comparison would be
    misleading)."""
    finite = [e for e in (err_pmb2, err_omni) if np.isfinite(e).any()]
    vmax = max(np.nanmax(e) for e in finite) if finite else 1.0
    xlim, ylim, figsize, narrow = geo_layout(
        [(df_pmb2['gt_x'], df_pmb2['gt_y']),
         (df_omni['gt_x'], df_omni['gt_y'])], waypoints, n_panels=2)
    fig, axes = plt.subplots(1, 2, figsize=(figsize[0] + 1.2, figsize[1]))
    sc = None
    for ax, df, err, label in [(axes[0], df_pmb2, err_pmb2, LABELS['pmb2']),
                               (axes[1], df_omni, err_omni, LABELS['omni'])]:
        ref_x = [df['gt_x'].iloc[0]] + [w['x'] for w in waypoints]
        ref_y = [df['gt_y'].iloc[0]] + [w['y'] for w in waypoints]
        ax.plot(ref_x, ref_y, '--', color='gray', linewidth=1.5,
                label='Straight-line reference (context)')
        sc = ax.scatter(df['gt_x'].to_numpy(), df['gt_y'].to_numpy(),
                        c=err, cmap='inferno', s=12, vmin=0, vmax=vmax)
        ax.scatter(ref_x[1:], ref_y[1:], marker='*', s=220, color='cyan',
                   edgecolor='black', zorder=5, label='Waypoint target')
        ax.set_title(label)
        ax.set_xlabel(r'$x$ [m]')
        ax.set_ylabel(r'$y$ [m]')
        geo_finish(ax, xlim, ylim)
        ax.grid(True, alpha=0.2)
        geo_legend(ax, narrow)
    cbar = fig.colorbar(sc, ax=axes, shrink=0.8)
    cbar.set_label('Tracking error vs Nav2 global plan [m]')
    fig.suptitle('In-plane trajectory error (2D)', fontsize=16)
    fig.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close(fig)


def plot_error_vs_arclength(arclen_pmb2, err_pmb2, arclen_omni, err_omni,
                            out_path):
    fig, ax = plt.subplots(figsize=(11, 5.5))
    ax.plot(arclen_pmb2, err_pmb2, label='PMB2', color=COLORS['pmb2'])
    ax.plot(arclen_omni, err_omni, label='OmniBase', color=COLORS['omni'])
    ax.set_xlabel('Travelled distance [m]')
    ax.set_ylabel('Tracking error [m]')
    ax.set_title('Tracking error (vs /plan) along the trajectory')
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close(fig)

def pretty_wheel_label(name):
    """Readable legend label for a wheel joint. Known joint names are mapped
    explicitly; already-clean names (e.g. the reconstructed OmniBase keys)
    pass through, and any other raw name is cleaned generically."""
    known = {
        'wheel_left_joint': 'Left',
        'wheel_right_joint': 'Right',
        'wheel_front_left_joint': 'Front left',
        'wheel_front_right_joint': 'Front right',
        'wheel_rear_left_joint': 'Rear left',
        'wheel_rear_right_joint': 'Rear right',
    }
    if name in known:
        return known[name]
    tokens = [w for w in name.replace('_', ' ').split()
              if w.lower() not in ('wheel', 'joint')]
    return ' '.join(tokens).capitalize() if tokens else name

def plot_wheel_rotation(df, pos_series, vel_series, base_label, out_path,
                        derived_joints=None, data_source_note=None,
                        vel_label_suffix=None):
    """Position (= cumulative rotation) and rate of every wheel joint over
    time. Curves in derived_joints are marked in the legend; a
    data_source_note becomes a subtitle (methodological transparency)."""
    if not pos_series:
        return
    derived_joints = derived_joints or []
    suffix = vel_label_suffix or 'derived from position'
    fig, axes = plt.subplots(2, 1, figsize=(11, 9), sharex=True)
    t = df['t'].to_numpy()
    for jn, series in pos_series.items():
        axes[0].plot(t, np.asarray(series), label=pretty_wheel_label(jn))
    for jn, series in vel_series.items():
        name = pretty_wheel_label(jn)
        label = f'{name} ({suffix})' if jn in derived_joints else name
        axes[1].plot(t, np.asarray(series), label=label)
    axes[0].set_ylabel('Joint position [rad]')
    axes[1].set_ylabel('Joint rate [rad/s]')
    axes[1].set_xlabel('Time [s]')
    title = f'Wheel rotation - {base_label}'
    if data_source_note:
        axes[0].set_title(title + f'\n{data_source_note}', fontsize=13)
    else:
        axes[0].set_title(title)
    for ax in axes:
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=9, ncol=2)
    fig.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close(fig)


def plot_waypoint_errors(err_pmb2, err_omni, out_path):
    fig, ax = plt.subplots(figsize=(8, 5))
    width = 0.35
    x = np.arange(len(err_pmb2))
    ax.bar(x - width / 2, err_pmb2['pos_err'].to_numpy(), width,
           label='PMB2', color=COLORS['pmb2'])
    ax.bar(x + width / 2, err_omni['pos_err'].to_numpy(), width,
           label='OmniBase', color=COLORS['omni'])
    ax.set_xticks(x)
    ax.set_xticklabels([f'WP{i}' for i in x])
    ax.set_ylabel('Position error [m]')
    ax.set_title('Positioning error per waypoint')
    ax.legend()
    ax.grid(True, alpha=0.3, axis='y')
    fig.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close(fig)


def plot_recoveries_map(df_pmb2, df_omni, waypoints, out_path):
    """Map of the points where recovery behaviours were triggered. Events
    are located via the increments of the counter columns, so it needs
    counters patched as TIME-RESOLVED step functions (bt_counts_from_bag.py
    v4+); constant counters produce no markers."""
    xlim, ylim, figsize, narrow = geo_layout(
        [(df_pmb2['gt_x'], df_pmb2['gt_y']),
         (df_omni['gt_x'], df_omni['gt_y'])], waypoints, n_panels=2)
    fig, axes = plt.subplots(1, 2, figsize=figsize)
    # ClearCostmap is the most frequent recovery around obstacles (and the
    # one triggered by a planning failure before the retry), so it must be
    # on the map; Spin/BackUp/Wait are the controller-side recoveries.
    markers = [('spin_count', 'Spin', 'red', 'X', 100),
               ('backup_count', 'BackUp', 'orange', 's', 80),
               ('wait_count', 'Wait', 'green', '^', 80),
               ('clearcostmap_count', 'ClearCostmap', 'purple', 'D', 70)]
    for ax, df, label in [(axes[0], df_pmb2, LABELS['pmb2']),
                          (axes[1], df_omni, LABELS['omni'])]:
        ax.plot(df['gt_x'].to_numpy(), df['gt_y'].to_numpy(),
                color='lightgray', label='Trajectory', zorder=1)
        any_marker = False
        for col, name, color, marker, size in markers:
            if col not in df.columns:
                continue
            starts = df[df[col].diff().fillna(0) > 0]
            if not starts.empty:
                any_marker = True
                ax.scatter(starts['gt_x'].to_numpy(),
                           starts['gt_y'].to_numpy(), color=color,
                           marker=marker, s=size,
                           label=f'{name} (x{len(starts)})', zorder=3)
        wx = [w['x'] for w in waypoints]
        wy = [w['y'] for w in waypoints]
        ax.scatter(wx, wy, marker='*', s=150, color='blue',
                   label='Waypoint', zorder=2)
        if not any_marker:
            ax.text(0.5, 0.5, 'no recovery interventions\nin this run',
                    transform=ax.transAxes, ha='center', va='center',
                    fontsize=12, color='dimgray',
                    bbox=dict(boxstyle='round', facecolor='white',
                              edgecolor='lightgray', alpha=0.8), zorder=4)
        ax.set_title(label)
        ax.set_xlabel(r'$x$ [m]')
        ax.set_ylabel(r'$y$ [m]')
        geo_finish(ax, xlim, ylim)
        geo_legend(ax, narrow)
    fig.suptitle('Map of recovery interventions', fontsize=16)
    fig.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close(fig)


def safe_ratio(a, b):
    try:
        return a / b
    except (ZeroDivisionError, TypeError):
        return float('nan')


# ---------------------------------------------------------------------- #
# Main                                                                    #
# ---------------------------------------------------------------------- #
def main():
    p = argparse.ArgumentParser()
    p.add_argument('--pmb2-run-dir', required=True,
                   help='PMB2 run folder (e.g. logs/pmb2_001, contains '
                        'log.csv and plans/)')
    p.add_argument('--omni-run-dir', required=True,
                   help='OmniBase run folder')
    p.add_argument('--waypoints', required=True)
    p.add_argument('--out-dir', default='./figures')
    args = p.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    waypoints = load_waypoints(args.waypoints)

    runs = {}
    for base, run_dir in [('pmb2', args.pmb2_run_dir),
                          ('omni', args.omni_run_dir)]:
        df = load_log(os.path.join(run_dir, 'log.csv'))
        t0, t1, completed = run_window(df)
        d = window_df(df, t0, t1)
        summary = load_run_summary(run_dir)
        runs[base] = {'dir': run_dir, 'd': d, 'completed': completed,
                      'duration': t1 - t0, 'summary': summary,
                      'plans': os.path.join(run_dir, 'plans')}
        line = (f'[{base}] window: {t1 - t0:.1f} s (sim), '
                f'completed={completed}')
        if summary:
            line += (f' | run_summary: duration_sim={summary.get("duration_sim_s", float("nan")):.1f} s, '
                     f'succeeded={summary.get("succeeded", "?")}, '
                     f'rtf={summary.get("rtf", float("nan")):.2f}')
        print(line)

    d_pmb2, d_omni = runs['pmb2']['d'], runs['omni']['d']
    start_pmb2 = (d_pmb2['gt_x'].iloc[0], d_pmb2['gt_y'].iloc[0])
    start_omni = (d_omni['gt_x'].iloc[0], d_omni['gt_y'].iloc[0])

    metrics = {
        'path_length_m': {'pmb2': path_length(d_pmb2),
                          'omni': path_length(d_omni)},
        'straight_line_ref_m': {
            'pmb2': straight_line_length(waypoints, start_pmb2),
            'omni': straight_line_length(waypoints, start_omni)},
        'completion_time_s': {'pmb2': runs['pmb2']['duration'],
                              'omni': runs['omni']['duration']},
        'smoothness_rms_angaccel': {'pmb2': smoothness_metric(d_pmb2),
                                    'omni': smoothness_metric(d_omni)},
        'smoothness_rms_linaccel': {
            'pmb2': linear_smoothness_metric(d_pmb2),
            'omni': linear_smoothness_metric(d_omni)},
        'lateral_usage_frac': {'pmb2': lateral_usage(d_pmb2),
                               'omni': lateral_usage(d_omni)},
    }
    print('\n=== Run metrics (run window) ===')
    for k, v in metrics.items():
        print(f"{k:28s} PMB2={v['pmb2']:.3f}  Omni={v['omni']:.3f}  "
              f"ratio(PMB2/Omni)={safe_ratio(v['pmb2'], v['omni']):.3f}")

    print('\n=== Robustness (recovery & planning) ===')
    for base in ('pmb2', 'omni'):
        c = load_bt_counters(runs[base]['dir'], runs[base]['d'])
        print(f"{LABELS[base]:24s} | Spin: {c['spins']:.0f} | "
              f"BackUp: {c['backups']:.0f} | Wait: {c['waits']:.0f} | "
              f"ClearCostmap: {c['costmap_clears']:.0f} | "
              f"Plan attempts: {c['replans']:.0f} | "
              f"Plan failures: {c['plan_failures']:.0f} "
              f"[source: {c['counters_source']}]")

    err_pmb2 = waypoint_errors(d_pmb2, waypoints)
    err_omni = waypoint_errors(d_omni, waypoints)
    print('\n=== Positioning error per waypoint [m] ===')
    print(pd.DataFrame({'PMB2': err_pmb2['pos_err'],
                        'OmniBase': err_omni['pos_err']}))

    print('\n=== Tracking error vs Nav2 /plan ===')
    cte_pmb2, arclen_pmb2 = compute_plan_tracking_error(
        d_pmb2, waypoints, runs['pmb2']['plans'])
    cte_omni, arclen_omni = compute_plan_tracking_error(
        d_omni, waypoints, runs['omni']['plans'])
    print('=== Tracking error stats [m] (mean / max / std) ===')
    for label, err in [('PMB2', cte_pmb2), ('OmniBase', cte_omni)]:
        print(f'{label:10s} mean={np.nanmean(err):.4f}  '
              f'max={np.nanmax(err):.4f}  std={np.nanstd(err):.4f}')

    print('\n=== Wheel data check and correction ===')
    pos_pmb2, vel_pmb2_raw = parse_joint_series(d_pmb2)
    pos_omni, vel_omni_raw = parse_joint_series(d_omni)
    vel_pmb2, derived_pmb2 = check_wheel_data_validity(
        d_pmb2, pos_pmb2, vel_pmb2_raw, 'PMB2')
    vel_omni, derived_omni = check_wheel_data_validity(
        d_omni, pos_omni, vel_omni_raw, 'OmniBase')

    plot_trajectories(d_pmb2, d_omni, waypoints,
                      os.path.join(args.out_dir, 'trajectory_comparison.png'),
                      plans_dirs={'pmb2': runs['pmb2']['plans'],
                                  'omni': runs['omni']['plans']})
    plot_velocity_profiles(d_pmb2, d_omni,
                           os.path.join(args.out_dir, 'velocity_profiles.png'))
    plot_waypoint_errors(err_pmb2, err_omni,
                         os.path.join(args.out_dir, 'waypoint_errors.png'))
    plot_cross_track_2d_compare(
        d_pmb2, d_omni, waypoints, cte_pmb2, cte_omni,
        os.path.join(args.out_dir, 'cross_track_error_2d.png'))
    plot_error_vs_arclength(
        arclen_pmb2, cte_pmb2, arclen_omni, cte_omni,
        os.path.join(args.out_dir, 'cross_track_error_vs_distance.png'))
    plot_recoveries_map(d_pmb2, d_omni, waypoints,
                        os.path.join(args.out_dir, 'recovery_map.png'))

    # PMB2: wheels from real simulated encoders (velocity derived from
    # position where the native field is NaN).
    plot_wheel_rotation(d_pmb2, pos_pmb2, vel_pmb2, LABELS['pmb2'],
                        os.path.join(args.out_dir, 'wheel_rotation_pmb2.png'),
                        derived_joints=derived_pmb2)
    # OmniBase: if the 4 wheel joints are absent from /joint_states
    # (planar_move plugin), reconstruct them from the inverse mecanum
    # kinematics (thesis Ch. 2) applied to cmd_vel.
    if pos_omni:
        plot_wheel_rotation(d_omni, pos_omni, vel_omni, LABELS['omni'],
                            os.path.join(args.out_dir,
                                         'wheel_rotation_omni.png'),
                            derived_joints=derived_omni)
    else:
        print('[OmniBase] wheel joints absent from /joint_states '
              '(planar_move plugin): reconstructing the 4 wheels from the '
              'inverse kinematics (cmd_vel -> wheels).')
        pos_rec, vel_rec, derived_rec = reconstruct_omni_wheel_data(d_omni)
        plot_wheel_rotation(
            d_omni, pos_rec, vel_rec, LABELS['omni'],
            os.path.join(args.out_dir, 'wheel_rotation_omni.png'),
            derived_joints=derived_rec,
            vel_label_suffix='reconstructed from kinematics',
            data_source_note='wheels reconstructed from the inverse '
                             'kinematic model (Ch. 2), not measured: '
                             'ideal no-slip rotation')
    print(f'\nFigures saved in {args.out_dir}/')


if __name__ == '__main__':
    main()
