#!/usr/bin/env python3
"""
metrics_logger.py
==================
Logger sincronizzato per la comparazione PMB2 vs OmniBase.

Si iscrive a:
  - /gazebo/model_states   -> pose "ground truth" (nessun errore di
                               localizzazione, presa direttamente dal
                               motore fisico di Gazebo)
  - /cmd_vel                -> velocita' comandata da Nav2 (vx, vy, wz)
  - /joint_states            -> posizione E velocita' delle ruote (vedi CAVEAT sotto)
  - /plan                   -> percorso globale calcolato da Nav2 per la
                               leg attiva: serve come riferimento "vero"
                               per l'errore di tracking, molto meglio di
                               una retta quando la leg deve evitare un
                               ostacolo (vedi compute_plan_tracking_error
                               in analyze_results.py)
  - /current_waypoint_idx   -> indice del waypoint attivo (pubblicato da
                               send_waypoints.py)

Scrive una riga di CSV ogni volta che arriva un nuovo /gazebo/model_states
(che tipicamente arriva alla frequenza fisica, es. 100Hz), con gli ultimi
valori disponibili delle altre fonti ("sample and hold" - adeguato perche'
cmd_vel e joint_states tipicamente arrivano anche piu' spesso).

STRUTTURA DI OUTPUT (una cartella per run, cosi' tutto sta insieme):
    <out_dir>/<base_type>_<run_id>/
        log.csv              <- la serie temporale principale
        plans/wp<N>.csv       <- ultimo /plan ricevuto mentre la leg N era attiva

REQUISITO: /gazebo/model_states esiste solo se nel world e' caricato il
plugin di stato:
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so"/>
Controlla il tuo file .world; se non c'e' aggiungilo prima di lanciare
questo logger, altrimenti il topic non esiste e il logger resta vuoto.

CAVEAT CONFERMATO (visto sui tuoi dati reali, base PMB2, 29/06): il campo
`velocity` di /joint_states per i giunti ruota di trazione (wheel_left_joint,
wheel_right_joint) torna NaN - limite noto del plugin ros2_control in
simulazione Gazebo Classic, non un errore della pipeline. Il campo
`position` resta invece valido e fisicamente significativo. NON serve
modificare questo logger: registriamo entrambi i campi grezzi esattamente
come arrivano (NaN incluso), e la correzione (derivare la velocita' da
position quando velocity e' NaN) avviene offline in analyze_results.py
(funzione fix_wheel_velocities), che gestisce automaticamente sia questo
bug sia il caveat seguente sull'OmniBase - sono due problemi diversi e
indipendenti, entrambi gestiti.

CAVEAT IMPORTANTE per OmniBase:
Se la base usa il plugin gazebo_ros_planar_move (coerente con la necessita'
del bridge TF odom->base_footprint), il moto della piattaforma NON deriva
dalla rotazione fisica delle ruote simulate: il plugin impone direttamente
vx, vy, wz al corpo rigido. In questo caso /joint_states puo' essere
puramente cosmetico (animazione visiva) e NON rappresentativo della vera
cinematica. Prima di costruire il grafico "per-ruota" per l'OmniBase,
confronta i joint_states con cmd_vel: se le ruote "girano" in modo
incoerente quando vy e' diverso da zero, e' il plugin planar_move a
muovere la base e i dati ruota non sono significativi per quel robot.
In tal caso, sostituisci il grafico con velocita' derivate per via
cinematica inversa da cmd_vel invece che lette da joint_states.

Nota: per il grafico "rotazione ruota" usiamo la POSIZIONE del giunto
(non la velocita' integrata a mano), perche' un giunto continuo in
Gazebo accumula posizione senza limiti - e' quindi la lettura diretta
piu' pulita di "quanto ha ruotato la ruota nel tempo".

Uso:
    python3 metrics_logger.py --base-type omni --run-id 001 \\
        --model-name tiago_omni --out-dir ./logs

L'argomento --model-name deve corrispondere esattamente al nome del
modello come appare in /gazebo/model_states. Per scoprirlo:
    ros2 topic echo /gazebo/model_states --once

Logger sincronizzato per la comparazione PMB2 vs OmniBase.
(Versione corretta per uso del ROS Clock e parse_known_args)
"""
import argparse
import csv
import math
import os
import time

import rclpy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav2_msgs.msg import BehaviorTreeLog
from std_msgs.msg import Int32


def yaw_from_quat(q):
    """Yaw da quaternione, assumendo roll/pitch ~ 0 (robot a base mobile)."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                       1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class MetricsLogger(Node):
    def __init__(self, base_type, run_id, model_name, out_dir):
        super().__init__('metrics_logger')
        self.model_name = model_name
        self._warned_missing_model = False

        self.run_dir = os.path.join(out_dir, f'{base_type}_{run_id}')
        self.plans_dir = os.path.join(self.run_dir, 'plans')
        os.makedirs(self.plans_dir, exist_ok=True)

        self.csv_path = os.path.join(self.run_dir, 'log.csv')
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            't', 'waypoint_idx',
            'gt_x', 'gt_y', 'gt_yaw',
            'cmd_vx', 'cmd_vy', 'cmd_wz',
            'joint_names', 'joint_positions', 'joint_velocities',
            'spin_count', 'backup_count', 'clearcostmap_count', 'planning_attempts'
        ])

        self.latest_cmd = (0.0, 0.0, 0.0)
        self.latest_joint_names = ''
        self.latest_joint_pos = ''
        self.latest_joint_vel = ''
        self.current_wp = -2  

        # --- NUOVO: Dizionario per contare le azioni del Behavior Tree ---
        self.bt_counts = {
            'Spin': 0,
            'Backup': 0,
            'ClearCostmap': 0,
            'PlanningAttempts': 0
        }

        self.create_subscription(ModelStates, '/gazebo/model_states', self.cb_model_states, 50)
        self.create_subscription(Twist, '/cmd_vel', self.cb_cmd_vel, 50)
        self.create_subscription(JointState, '/joint_states', self.cb_joints, 50)
        self.create_subscription(Path, '/plan', self.cb_plan, 10)
        self.create_subscription(Int32, '/current_waypoint_idx', self.cb_wp_idx, 10)
        
        # --- NUOVO: Sottoscrizione al log del Behavior Tree ---
        self.create_subscription(BehaviorTreeLog, '/behavior_tree_log', self.cb_bt_log, 100)

        self.get_logger().info(f'Logging su {self.run_dir}/')

    # --- NUOVO: Callback per analizzare il Behavior Tree in tempo reale ---
    def cb_bt_log(self, msg: BehaviorTreeLog):
        # I codici di stato di nav2_msgs/msg/BehaviorTreeStatusChange sono:
        # IDLE = 1, RUNNING = 2, SUCCESS = 3, FAILURE = 4
        for event in msg.event_log:
            # Contiamo un'azione SOLO nell'istante in cui passa allo stato RUNNING
            # (evita di contare la stessa azione decine di volte mentre è in esecuzione)
            if event.current_status == 2 and event.previous_status != 2:
                name = event.node_name
                
                # Match parziale sulle stringhe per non dipendere dai nomi esatti dell'XML
                if 'Spin' in name:
                    self.bt_counts['Spin'] += 1
                elif 'Backup' in name:
                    self.bt_counts['Backup'] += 1
                elif 'ClearEntireCostmap' in name or 'ClearLocalCostmap' in name or 'ClearGlobalCostmap' in name:
                    self.bt_counts['ClearCostmap'] += 1
                elif 'ComputePathToPose' in name or 'ComputePathThroughPoses' in name:
                    self.bt_counts['PlanningAttempts'] += 1

    def cb_cmd_vel(self, msg: Twist):
        self.latest_cmd = (msg.linear.x, msg.linear.y, msg.angular.z)

    def cb_joints(self, msg: JointState):
        self.latest_joint_names = ';'.join(msg.name)
        self.latest_joint_pos = ';'.join(f'{v:.5f}' for v in msg.position)
        self.latest_joint_vel = ';'.join(f'{v:.4f}' for v in msg.velocity)

    def cb_plan(self, msg: Path):
        if self.current_wp < 0 or not msg.poses:
            return
        path_file = os.path.join(self.plans_dir, f'wp{self.current_wp}.csv')
        with open(path_file, 'w', newline='') as pf:
            w = csv.writer(pf)
            w.writerow(['x', 'y'])
            for ps in msg.poses:
                w.writerow([f'{ps.pose.position.x:.5f}', f'{ps.pose.position.y:.5f}'])

    def cb_wp_idx(self, msg: Int32):
        self.current_wp = msg.data
        
        # --- NUOVO: Salva i conteggi delle recovery quando la run finisce (-1) ---
        if self.current_wp == -1:
            self.save_recovery_summary()

    # --- NUOVO: Funzione per salvare il file CSV dei comportamenti ---
    def save_recovery_summary(self):
        summary_file = os.path.join(self.run_dir, 'recovery_summary.csv')
        with open(summary_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Behavior', 'Count'])
            for key, value in self.bt_counts.items():
                writer.writerow([key, value])
        self.get_logger().info(f"--- RIEPILOGO ROBUSTEZZA SALVATO ---")
        self.get_logger().info(f"Conteggi finali: {self.bt_counts}")
        self.get_logger().info(f"File: {summary_file}")

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
        
        t = self.get_clock().now().nanoseconds / 1e9

        self.writer.writerow([
            f'{t:.4f}', self.current_wp,
            f'{pose.position.x:.5f}', f'{pose.position.y:.5f}', f'{yaw:.5f}',
            f'{self.latest_cmd[0]:.4f}', f'{self.latest_cmd[1]:.4f}', f'{self.latest_cmd[2]:.4f}',
            self.latest_joint_names, self.latest_joint_pos, self.latest_joint_vel,
            self.bt_counts['Spin'], 
            self.bt_counts['Backup'], 
            self.bt_counts['ClearCostmap'], 
            self.bt_counts['PlanningAttempts']
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
                         help="Nome del modello in /gazebo/model_states (es. 'tiago', 'tiago_omni')")
    parser.add_argument('--out-dir', default='./logs',
                         help="Cartella padre; la run finisce in <out-dir>/<base-type>_<run-id>/")
    
    # FIX: Usiamo parse_known_args per non crashare quando riceve --ros-args
    args, unknown = parser.parse_known_args()

    rclpy.init()
    node = MetricsLogger(args.base_type, args.run_id, args.model_name, args.out_dir)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
