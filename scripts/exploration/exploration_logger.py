#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
exploration_logger.py
=====================
Logger per le run di ESPLORAZIONE AUTONOMA (PMB2 vs OmniBase): variante di
metrics_logger.py pensata per lavorare con autonomous_explorer_thesis.py.

Cosa cambia rispetto al logger dei waypoint:
  - /current_waypoint_idx non esiste piu': al suo posto si ascolta
    /exploration_status (std_msgs/Int32, pubblicato dall'explorer):
        n >= 0 -> indice del goal di frontiera attivo (0, 1, 2, ...)
        -1     -> esplorazione dichiarata COMPLETATA (frontiere esaurite);
                  da qui in poi il robot sta solo tornando alla base
  - ci si iscrive a /map (OccupancyGrid pubblicata da slam_toolbox) e ad ogni
    riga si logga l'AREA ESPLORATA [m^2] e i conteggi delle celle
    (libere/occupate/sconosciute): e' la metrica cardine coverage-vs-tempo
  - si logga anche la posa STIMATA dallo SLAM (TF map -> base_footprint)
    accanto alla ground truth di Gazebo: offline (analyze_exploration.py)
    se ne ricava l'errore di localizzazione
  - i /plan vengono salvati per goal di frontiera: plans/goal<N>.csv

NOTA FRAME (importante per l'analisi): slam_toolbox aggancia il frame `map`
alla posa INIZIALE del robot. Se il robot parte dallo spawn di default
(0,0,0, nessun teletrasporto nel launch), frame map e frame world di Gazebo
coincidono (a meno della deriva SLAM) e gt/slam sono confrontabili
direttamente. Se spawni altrove, passa la posa di spawn ad
analyze_exploration.py con --spawn-x/y/yaw: sara' lui a riportare la ground
truth nel frame map prima del confronto.

REQUISITI identici a metrics_logger.py:
  - plugin gazebo_ros_state nel world (altrimenti /gazebo/model_states non
    esiste e il log resta vuoto)
  - use_sim_time passato via --ros-args (vedi Uso)
Valgono anche gli stessi caveat gia' documentati la' (velocity NaN dei
giunti ruota, plugin planar_move per l'OmniBase): questo logger, come
l'altro, registra i dati grezzi cosi' come arrivano.

Uso:
    python3 exploration_logger.py --base-type pmb2 --run-id expl_rep1 \
        --model-name tiago --out-dir ./logs_explore \
        --ros-args -p use_sim_time:=true
"""
import argparse
import csv
import math
import os

import numpy as np
import rclpy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from nav2_msgs.msg import BehaviorTreeLog
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


def yaw_from_quat(q):
    """Yaw da quaternione, assumendo roll/pitch ~ 0 (robot a base mobile)."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ExplorationLogger(Node):
    def __init__(self, base_type, run_id, model_name, out_dir):
        super().__init__('exploration_logger')
        self.model_name = model_name
        self._warned_missing_model = False
        self._warned_no_tf = False
        self._recovery_summary_saved = False

        self.run_dir = os.path.join(out_dir, f'{base_type}_{run_id}')
        self.plans_dir = os.path.join(self.run_dir, 'plans')
        os.makedirs(self.plans_dir, exist_ok=True)

        self.csv_path = os.path.join(self.run_dir, 'log.csv')
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            't', 'goal_idx',
            'gt_x', 'gt_y', 'gt_yaw',
            'slam_x', 'slam_y', 'slam_yaw',
            'cmd_vx', 'cmd_vy', 'cmd_wz',
            'explored_area_m2', 'free_cells', 'occupied_cells', 'unknown_cells',
            'map_res',
            'joint_names', 'joint_positions', 'joint_velocities',
            'spin_count', 'backup_count', 'clearcostmap_count', 'planning_attempts',
        ])

        # --- ultimi valori disponibili (sample-and-hold, come nel logger waypoint) ---
        self.latest_cmd = (0.0, 0.0, 0.0)
        self.latest_joint_names = ''
        self.latest_joint_pos = ''
        self.latest_joint_vel = ''
        self.current_goal = -2  # -2 = explorer non ancora partito
        # (area_m2, free, occupied, unknown, resolution)
        self.map_stats = (float('nan'), 0, 0, 0, float('nan'))

        self.bt_counts = {
            'Spin': 0,
            'Backup': 0,
            'ClearCostmap': 0,
            'PlanningAttempts': 0,
        }

        # TF listener per la posa stimata dallo SLAM (map -> base_footprint)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # /map di slam_toolbox e' pubblicata con durability TRANSIENT_LOCAL:
        # con questo QoS riceviamo subito anche l'ultima mappa gia' pubblicata.
        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(ModelStates, '/gazebo/model_states', self.cb_model_states, 50)
        self.create_subscription(Twist, '/cmd_vel', self.cb_cmd_vel, 50)
        self.create_subscription(JointState, '/joint_states', self.cb_joints, 50)
        self.create_subscription(Path, '/plan', self.cb_plan, 10)
        self.create_subscription(Int32, '/exploration_status', self.cb_status, 10)
        self.create_subscription(OccupancyGrid, '/map', self.cb_map, map_qos)
        self.create_subscription(BehaviorTreeLog, '/behavior_tree_log', self.cb_bt_log, 100)

        self.get_logger().info(f'Logging esplorazione su {self.run_dir}/')

    # ------------------------------------------------------------------ #
    # Callbacks                                                          #
    # ------------------------------------------------------------------ #
    def cb_bt_log(self, msg: BehaviorTreeLog):
        # IDLE = 1, RUNNING = 2, SUCCESS = 3, FAILURE = 4
        # Contiamo un'azione SOLO nell'istante in cui passa a RUNNING.
        for event in msg.event_log:
            if event.current_status == 2 and event.previous_status != 2:
                name = event.node_name
                if 'Spin' in name:
                    self.bt_counts['Spin'] += 1
                elif 'Backup' in name:
                    self.bt_counts['Backup'] += 1
                elif ('ClearEntireCostmap' in name or 'ClearLocalCostmap' in name
                        or 'ClearGlobalCostmap' in name):
                    self.bt_counts['ClearCostmap'] += 1
                elif 'ComputePathToPose' in name or 'ComputePathThroughPoses' in name:
                    self.bt_counts['PlanningAttempts'] += 1

    def cb_cmd_vel(self, msg: Twist):
        self.latest_cmd = (msg.linear.x, msg.linear.y, msg.angular.z)

    def cb_joints(self, msg: JointState):
        self.latest_joint_names = ';'.join(msg.name)
        self.latest_joint_pos = ';'.join(f'{v:.5f}' for v in msg.position)
        self.latest_joint_vel = ';'.join(f'{v:.4f}' for v in msg.velocity)

    def cb_map(self, msg: OccupancyGrid):
        """Statistiche di copertura dalla mappa SLAM. Convenzione OccupancyGrid:
        -1 = sconosciuta, 0..100 = probabilita' di occupazione (slam_toolbox
        pubblica tipicamente 0 libero e 100 occupato; usiamo soglia 50)."""
        data = np.asarray(msg.data, dtype=np.int16)
        unknown = int((data == -1).sum())
        occupied = int((data >= 50).sum())
        free = int(((data >= 0) & (data < 50)).sum())
        res = float(msg.info.resolution)
        area = (free + occupied) * res * res  # area "conosciuta" in m^2
        self.map_stats = (area, free, occupied, unknown, res)

    def cb_plan(self, msg: Path):
        if self.current_goal < 0 or not msg.poses:
            return
        path_file = os.path.join(self.plans_dir, f'goal{self.current_goal}.csv')
        with open(path_file, 'w', newline='') as pf:
            w = csv.writer(pf)
            w.writerow(['x', 'y'])
            for ps in msg.poses:
                w.writerow([f'{ps.pose.position.x:.5f}', f'{ps.pose.position.y:.5f}'])

    def cb_status(self, msg: Int32):
        self.current_goal = msg.data
        # -1 = esplorazione completata: congela i conteggi di recovery della
        # FASE DI ESPLORAZIONE (il ritorno a casa resta fuori dal riepilogo).
        if msg.data == -1 and not self._recovery_summary_saved:
            self.save_recovery_summary()
            self._recovery_summary_saved = True

    def get_slam_pose(self):
        """Posa stimata dallo SLAM: TF map -> base_footprint. NaN finche' lo
        SLAM non pubblica (normale nei primi istanti della run)."""
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            return (t.transform.translation.x,
                    t.transform.translation.y,
                    yaw_from_quat(t.transform.rotation))
        except Exception:
            if not self._warned_no_tf:
                self.get_logger().warn(
                    "TF map->base_footprint non ancora disponibile "
                    "(normale finche' lo SLAM non parte).")
                self._warned_no_tf = True
            return (float('nan'), float('nan'), float('nan'))

    def save_recovery_summary(self):
        summary_file = os.path.join(self.run_dir, 'recovery_summary.csv')
        with open(summary_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Behavior', 'Count'])
            for key, value in self.bt_counts.items():
                writer.writerow([key, value])
        self.get_logger().info('--- RIEPILOGO ROBUSTEZZA (fase di esplorazione) SALVATO ---')
        self.get_logger().info(f'Conteggi: {self.bt_counts}')
        self.get_logger().info(f'File: {summary_file}')

    def cb_model_states(self, msg: ModelStates):
        try:
            idx = msg.name.index(self.model_name)
        except ValueError:
            if not self._warned_missing_model:
                self.get_logger().warn(
                    f"Modello '{self.model_name}' non trovato in /gazebo/model_states. "
                    f"Nomi disponibili: {list(msg.name)}")
                self._warned_missing_model = True
            return

        pose = msg.pose[idx]
        yaw = yaw_from_quat(pose.orientation)
        sx, sy, syaw = self.get_slam_pose()
        area, free, occ, unk, res = self.map_stats

        t = self.get_clock().now().nanoseconds / 1e9

        self.writer.writerow([
            f'{t:.4f}', self.current_goal,
            f'{pose.position.x:.5f}', f'{pose.position.y:.5f}', f'{yaw:.5f}',
            f'{sx:.5f}', f'{sy:.5f}', f'{syaw:.5f}',
            f'{self.latest_cmd[0]:.4f}', f'{self.latest_cmd[1]:.4f}', f'{self.latest_cmd[2]:.4f}',
            f'{area:.4f}', free, occ, unk, f'{res:.4f}',
            self.latest_joint_names, self.latest_joint_pos, self.latest_joint_vel,
            self.bt_counts['Spin'],
            self.bt_counts['Backup'],
            self.bt_counts['ClearCostmap'],
            self.bt_counts['PlanningAttempts'],
        ])
        self.csv_file.flush()

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--base-type', required=True, choices=['pmb2', 'omni'])
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--model-name', required=True,
                        help="Nome del modello in /gazebo/model_states (es. 'tiago')")
    parser.add_argument('--out-dir', default='./logs_explore',
                        help='Cartella padre; la run finisce in <out-dir>/<base-type>_<run-id>/')
    # parse_known_args per non crashare quando riceve --ros-args
    args, unknown = parser.parse_known_args()

    rclpy.init()
    node = ExplorationLogger(args.base_type, args.run_id, args.model_name, args.out_dir)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
