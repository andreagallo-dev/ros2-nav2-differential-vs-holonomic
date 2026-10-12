#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
autonomous_explorer_thesis.py
=============================
Versione "da tesi" di autonomous_explorer.py (tirocinio): la logica di
esplorazione a frontiere (detection, blacklist, timeout, ritorno a casa) e'
IDENTICA all'originale; cambia solo la strumentazione, per rendere le run
comparabili e ripetibili tra PMB2 e OmniBase:

  1. Argomenti CLI (--base-type, --run-id, --out-dir, --seed, --policy, ...)
     cosi' ogni run finisce nella stessa struttura di cartelle dei test a
     waypoint:   <out-dir>/<base_type>_<run_id>/

  2. Pubblica /exploration_status (std_msgs/Int32) per exploration_logger.py:
         n >= 0 -> indice del goal di frontiera appena inviato
         -1     -> esplorazione COMPLETATA (frontiere esaurite); da qui in
                   poi il robot torna solo alla base

  3. Scrive exploration_summary.csv nella cartella della run, con tempi in
     SIM-TIME e WALL-TIME + RTF (stessa filosofia di send_waypoints.py),
     conteggio goal inviati/raggiunti/rifiutati/falliti, timeout ed esito.
     Il summary viene scritto anche se interrompi con Ctrl-C (run parziale).

  4. La mappa esplorata viene salvata DENTRO la cartella della run
     (explored_map.pgm/.yaml), non piu' nella working directory.

  5. --seed fissa il generatore casuale (documentabile in tesi).
     --policy permette una selezione DETERMINISTICA della frontiera:
        random_far -> comportamento ORIGINALE del tirocinio (random tra il
                      30% di frontiere piu' lontane)
        farthest   -> sempre la frontiera valida piu' lontana (consigliata
                      per la comparazione: zero varianza di politica)
        nearest    -> sempre la piu' vicina
     Qualunque sia la scelta, USA LA STESSA per entrambe le basi.

Nota metodologica: anche a parita' di seed le traiettorie di PMB2 e OmniBase
divergono comunque (cinematiche diverse -> mappe parziali diverse ->
frontiere diverse), quindi servono comunque N repliche per base (>= 5, come
per i test a waypoint) e statistiche aggregate.

Uso:
    python3 autonomous_explorer_thesis.py --base-type pmb2 --run-id expl_rep1 \
        --out-dir ./logs_explore --policy farthest \
        --ros-args -p use_sim_time:=true
"""
import argparse
import csv
import math
import os
import random
import time

import numpy as np
import rclpy
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Int32
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class ThesisExplorer(Node):
    def __init__(self, args):
        super().__init__('thesis_explorer')

        # --- FORZA SIM TIME (come nell'originale: cruciale in Gazebo) ---
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        # --- Parametri della run ---
        self.base_type = args.base_type
        self.run_id = args.run_id
        self.run_dir = os.path.join(args.out_dir, f'{args.base_type}_{args.run_id}')
        os.makedirs(self.run_dir, exist_ok=True)

        self.policy = args.policy
        self.seed = args.seed
        if args.seed is not None:
            random.seed(args.seed)

        self.frontier_threshold = args.frontier_threshold
        self.MAX_NAV_TIME = args.max_nav_time
        self.max_nav_time_home = args.max_nav_time_home
        # NUOVO: filtro "distanza dai muri" per scartare i goal irraggiungibili
        # (frontiere libere nella griglia /map ma dentro l'inflation di Nav2)
        self.goal_clearance = args.goal_clearance       # [m] distanza minima da un muro
        self.wall_thresh = args.wall_value_threshold    # soglia "cella = muro" (0-100)
        self.blacklist_radius = args.blacklist_radius   # [m] raggio di esclusione di un goal fallito
        self.goal_retract = args.goal_retract           # [m] arretramento del goal dal bordo dell'ignoto

        # --- Componenti ROS2 ---
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.map_sub = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.status_pub = self.create_publisher(Int32, '/exploration_status', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Stato interno (identico al tirocinio) ---
        self.latest_map = None
        self.blacklist_zones = []
        self.is_navigating = False
        self.nav_start_time = None
        self.current_goal_handle = None
        self.current_target = None
        self.home_pose = None
        self.returning_home = False

        # --- Contatori e tempi per il summary di tesi ---
        self.goals_sent = 0        # goal di frontiera inviati (HOME esclusa)
        self.goals_reached = 0
        self.goals_rejected = 0
        self.goals_failed = 0      # ABORTED / CANCELED
        self.timeouts = 0
        self.frontiers_rejected_wall_total = 0   # diagnostica del filtro clearance
        self.t_sim_start = None
        self.t_wall_start = None
        self.t_sim_explore_end = None   # istante di "frontiere esaurite"
        self.exploration_completed = False
        self.returned_home = False
        self._summary_written = False

        self.get_logger().info('Attendo il server Nav2...')
        self.nav_client.wait_for_server()
        self.get_logger().info(
            f'Nav2 pronto! Esplorazione [{self.base_type}] run {self.run_id} '
            f'(policy={self.policy}, seed={self.seed}, '
            f'frontier_threshold={self.frontier_threshold}).')

        # Loop di pianificazione ogni 1.0 s (come nell'originale)
        self.timer = self.create_timer(1.0, self.planning_loop)

    # ------------------------------------------------------------------ #
    # Callbacks base                                                     #
    # ------------------------------------------------------------------ #
    def map_callback(self, msg):
        self.latest_map = msg

    def get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return x, y, yaw
        except Exception:
            return None, None, None

    def add_to_blacklist(self, target):
        """Come nel tirocinio, ma evita i duplicati (il timeout + il result
        CANCELED possono blacklistare due volte lo stesso punto)."""
        if target is not None and target not in self.blacklist_zones:
            self.blacklist_zones.append(target)

    # ------------------------------------------------------------------ #
    # Loop principale                                                    #
    # ------------------------------------------------------------------ #
    def planning_loop(self):
        robot_x, robot_y, robot_yaw = self.get_robot_pose()
        if robot_x is None or self.latest_map is None:
            return

        if self.home_pose is None:
            self.home_pose = (robot_x, robot_y, robot_yaw)
            # inizio ufficiale della run: prima posa valida disponibile
            self.t_sim_start = self.get_clock().now().nanoseconds / 1e9
            self.t_wall_start = time.time()
            self.get_logger().info(
                f'HOME memorizzata: X={robot_x:.2f}, Y={robot_y:.2f} '
                f'(t_sim_start={self.t_sim_start:.1f}s)')

        # 1. CHECK TIMEOUT (in tempo simulato, identico al tirocinio)
        if self.is_navigating:
            current_time = self.get_clock().now().nanoseconds / 1e9
            if current_time - self.nav_start_time > self.MAX_NAV_TIME:
                self.get_logger().warn('TIMEOUT! Il robot sembra bloccato. Annullo il goal.')
                self.timeouts += 1
                if self.current_goal_handle is not None:
                    self.current_goal_handle.cancel_goal_async()
                if self.returning_home:
                    self.get_logger().warn(
                        'Impossibile raggiungere esattamente HOME. Salvo la mappa qui.')
                    self.save_map_and_exit()
                else:
                    self.add_to_blacklist(self.current_target)
                    self.is_navigating = False
            return

        if self.returning_home:
            return

        # 2. FRONTIER DETECTION (identica al tirocinio)
        width = self.latest_map.info.width
        height = self.latest_map.info.height
        resolution = self.latest_map.info.resolution
        origin_x = self.latest_map.info.origin.position.x
        origin_y = self.latest_map.info.origin.position.y

        map_data = np.array(self.latest_map.data).reshape((height, width))

        free_space = (map_data == 0)
        unknown_space = (map_data == -1)
        occupied_space = (map_data >= self.wall_thresh)   # muri/ostacoli nella griglia

        shift_up = np.roll(unknown_space, 1, axis=0)
        shift_down = np.roll(unknown_space, -1, axis=0)
        shift_left = np.roll(unknown_space, 1, axis=1)
        shift_right = np.roll(unknown_space, -1, axis=1)

        is_frontier = free_space & (shift_up | shift_down | shift_left | shift_right)
        fy, fx = np.where(is_frontier)

        # raggio di sicurezza dai muri, convertito in celle (dipende dalla
        # risoluzione corrente della mappa). 0 => filtro disattivato.
        clearance_cells = (int(round(self.goal_clearance / resolution))
                           if (self.goal_clearance > 0 and resolution > 0) else 0)

        valid_frontiers = []
        rejected_wall = 0
        for i in range(len(fx)):
            wx = (fx[i] * resolution) + origin_x
            wy = (fy[i] * resolution) + origin_y
            dist = math.hypot(wx - robot_x, wy - robot_y)
            if dist < 1.0:
                continue
            in_blacklist = any(math.hypot(wx - bx, wy - by) < self.blacklist_radius
                               for bx, by in self.blacklist_zones)
            if in_blacklist:
                continue
            # --- NUOVO: scarta le frontiere troppo vicine a un muro ---
            # Una cella "libera" (==0) nella griglia /map puo' comunque cadere
            # dentro l'inflation layer di Nav2: in quel caso il planner accetta
            # il goal ma il controller non riesce a portarci il footprint del
            # robot -> robot fermo -> timeout -> blacklist. Lo evitiamo a monte
            # controllando che nessuna cella-muro cada in una finestra di lato
            # 2*clearance_cells attorno alla frontiera (vicinato di Chebyshev,
            # leggermente conservativo agli angoli: va bene, e' lato sicurezza).
            if clearance_cells > 0:
                y0 = max(0, fy[i] - clearance_cells)
                y1 = min(height, fy[i] + clearance_cells + 1)
                x0 = max(0, fx[i] - clearance_cells)
                x1 = min(width, fx[i] + clearance_cells + 1)
                if occupied_space[y0:y1, x0:x1].any():
                    rejected_wall += 1
                    continue
            valid_frontiers.append((wx, wy, dist))

        if rejected_wall:
            self.frontiers_rejected_wall_total += rejected_wall
            self.get_logger().info(
                f'{rejected_wall} frontiere scartate (troppo vicine a un muro, '
                f'clearance={self.goal_clearance:.2f} m) -> {len(valid_frontiers)} valide.')

        # 3. CRITERIO DI COMPLETAMENTO (identico: stesso threshold per le due basi)
        if len(valid_frontiers) < self.frontier_threshold:
            self.get_logger().info(
                f'ESPLORAZIONE COMPLETATA! ({len(valid_frontiers)} frontiere valide '
                f'< soglia {self.frontier_threshold}). Ritorno alla base...')
            self.exploration_completed = True
            self.t_sim_explore_end = self.get_clock().now().nanoseconds / 1e9
            self.status_pub.publish(Int32(data=-1))   # segnale per il logger
            self.returning_home = True
            self.MAX_NAV_TIME = self.max_nav_time_home
            self.send_goal(self.home_pose[0], self.home_pose[1], self.home_pose[2])
            return

        # 4. SELEZIONE DELLA FRONTIERA (politica configurabile)
        valid_frontiers.sort(key=lambda f: f[2])   # distanza crescente
        if self.policy == 'nearest':
            best_goal = valid_frontiers[0]
        elif self.policy == 'farthest':
            best_goal = valid_frontiers[-1]
        else:
            # 'random_far' = comportamento ORIGINALE del tirocinio:
            # random tra il 30% di frontiere piu' lontane
            top_far = valid_frontiers[int(len(valid_frontiers) * 0.7):]
            if not top_far:
                top_far = valid_frontiers
            best_goal = random.choice(top_far)

        # Orientamento del goal = direzione robot->frontiera. Cosi' il robot
        # arriva gia' rivolto verso la zona inesplorata (il laser la scansiona
        # subito) e NON deve ruotare sul posto a fine goal per soddisfare la
        # yaw_goal_tolerance: quella rotazione terminale sembra una "recovery
        # inutile" ma e' solo l'orientamento imposto (prima era fisso a 0.0).
        bx, by = best_goal[0], best_goal[1]
        goal_yaw = math.atan2(by - robot_y, bx - robot_x)

        # ARRETRAMENTO del goal dal bordo dell'ignoto verso il robot.
        # PERCHE': una cella di frontiera sta ESATTAMENTE sul confine
        # libero/ignoto. Mandarci il goal sopra e' diverso da un Nav2 Goal
        # piazzato a mano in spazio libero confermato (quello funziona sempre):
        # mentre il robot si avvicina, il laser rivela cio' che c'e' oltre - e
        # spesso e' un muro appena oltre il vecchio orizzonte sensore. Quel muro
        # (con la sua inflation) cade sul goal, che era valido e diventa
        # irraggiungibile -> ComputePathToPose fallisce -> il behavior server
        # parte con le recovery, anche se "a occhio" il punto era libero.
        # Arretrando il goal di goal_retract metri lungo la retta frontiera->
        # robot, il target cade in spazio GIA' libero: il robot avanza comunque
        # verso la frontiera e la scansiona, ma il goal non e' piu' sul bordo.
        gx = bx - self.goal_retract * math.cos(goal_yaw)
        gy = by - self.goal_retract * math.sin(goal_yaw)
        # sicurezza: usa il punto arretrato solo se e' effettivamente libero
        # (==0) nella griglia; altrimenti (es. frontiera dietro un angolo, retta
        # che taglia un muro) ripiega sulla frontiera originale.
        gx_idx = int((gx - origin_x) / resolution)
        gy_idx = int((gy - origin_y) / resolution)
        if (0 <= gy_idx < height and 0 <= gx_idx < width
                and map_data[gy_idx, gx_idx] == 0):
            tx, ty = gx, gy
        else:
            tx, ty = bx, by
        self.send_goal(tx, ty, goal_yaw)

    # ------------------------------------------------------------------ #
    # Invio goal e gestione risultati                                    #
    # ------------------------------------------------------------------ #
    def send_goal(self, x, y, yaw):
        self.is_navigating = True
        self.current_target = (x, y)
        self.nav_start_time = self.get_clock().now().nanoseconds / 1e9

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        if self.returning_home:
            self.get_logger().info(f'Navigo verso HOME: X={x:.2f}, Y={y:.2f}')
        else:
            idx = self.goals_sent
            self.status_pub.publish(Int32(data=idx))   # per exploration_logger
            self.goals_sent += 1
            self.get_logger().info(f'Goal di frontiera #{idx}: X={x:.2f}, Y={y:.2f}')

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.current_goal_handle = future.result()
        if not self.current_goal_handle.accepted:
            self.get_logger().warn('Goal RIFIUTATO dal planner. Punto in blacklist.')
            self.goals_rejected += 1
            if self.returning_home:
                self.save_map_and_exit()
            else:
                self.add_to_blacklist(self.current_target)
                self.is_navigating = False
            return
        self._get_result_future = self.current_goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        self.is_navigating = False

        if status == 4:  # SUCCEEDED
            if self.returning_home:
                self.get_logger().info('Arrivato alla base!')
                self.returned_home = True
                self.save_map_and_exit()
            else:
                self.goals_reached += 1
                self.get_logger().info('Frontiera raggiunta!')
        else:
            # QUALSIASI esito non-SUCCESS (ABORTED=6, CANCELED=5, ecc.) ->
            # blacklist. Il tirocinio NON blacklistava gli ABORTED, ma con una
            # policy deterministica (nearest/farthest) questo crea un LOOP
            # INFINITO: Nav2 esaurisce le sue recovery e aborta il goal, il
            # loop di pianificazione ripesca la STESSA frontiera piu' vicina
            # (non blacklistata), la rimanda, Nav2 ri-aborta... e il behavior
            # server non smette mai. Blacklistando l'ABORTED, la frontiera che
            # Nav2 ha dichiarato irraggiungibile viene esclusa e alla prossima
            # iterazione se ne sceglie un'altra.
            self.goals_failed += 1
            self.get_logger().warn(
                f'Goal fallito (status={status}: 6=ABORTED, 5=CANCELED). Blacklist.')
            if self.returning_home:
                self.save_map_and_exit()
            else:
                self.add_to_blacklist(self.current_target)

    # ------------------------------------------------------------------ #
    # Chiusura run: mappa + summary                                      #
    # ------------------------------------------------------------------ #
    def save_map_and_exit(self):
        map_path = os.path.join(self.run_dir, 'explored_map')
        self.get_logger().info(f'SALVATAGGIO MAPPA in {map_path} ...')
        os.system(f'ros2 run nav2_map_server map_saver_cli -f {map_path}')
        self.get_logger().info('Mappa salvata.')
        self.write_summary()
        self.get_logger().info('Nodo in chiusura.')
        raise SystemExit

    def write_summary(self):
        """Scrive exploration_summary.csv (una sola volta). Chiamata sia alla
        fine regolare della run sia dal finally di main (Ctrl-C)."""
        if self._summary_written:
            return
        self._summary_written = True

        t_sim_end = self.get_clock().now().nanoseconds / 1e9
        t_wall_end = time.time()
        sim_total = (t_sim_end - self.t_sim_start) if self.t_sim_start else float('nan')
        wall_total = (t_wall_end - self.t_wall_start) if self.t_wall_start else float('nan')
        explore_sim = ((self.t_sim_explore_end - self.t_sim_start)
                       if (self.t_sim_explore_end and self.t_sim_start) else float('nan'))
        rtf = (sim_total / wall_total) if (wall_total and wall_total > 0) else float('nan')

        row = {
            'base_type': self.base_type,
            'run_id': self.run_id,
            'policy': self.policy,
            'seed': self.seed if self.seed is not None else '',
            'frontier_threshold': self.frontier_threshold,
            'exploration_completed': self.exploration_completed,
            'returned_home': self.returned_home,
            # tempi (la metrica ufficiale e' quella in tempo SIMULATO)
            'explore_duration_sim_s': explore_sim,
            'total_duration_sim_s': sim_total,
            'total_duration_wall_s': wall_total,
            'rtf': rtf,
            't_sim_start_s': self.t_sim_start if self.t_sim_start else float('nan'),
            't_sim_explore_end_s': (self.t_sim_explore_end
                                    if self.t_sim_explore_end else float('nan')),
            't_sim_end_s': t_sim_end,
            # conteggi (solo goal di FRONTIERA: la HOME e' esclusa)
            'goals_sent': self.goals_sent,
            'goals_reached': self.goals_reached,
            'goals_rejected': self.goals_rejected,
            'goals_failed': self.goals_failed,
            'timeouts': self.timeouts,
            'goal_clearance_m': self.goal_clearance,
            'blacklist_radius_m': self.blacklist_radius,
            'goal_retract_m': self.goal_retract,
            'frontiers_rejected_wall': self.frontiers_rejected_wall_total,
            'blacklist_size': len(self.blacklist_zones),
        }

        summary_path = os.path.join(self.run_dir, 'exploration_summary.csv')
        with open(summary_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=row.keys())
            writer.writeheader()
            writer.writerow(row)

        self.get_logger().info(
            f'Summary salvato in {summary_path} | '
            f'esplorazione: {explore_sim:.1f}s sim, goal inviati: {self.goals_sent}, '
            f'raggiunti: {self.goals_reached}, falliti: {self.goals_failed}, '
            f'timeout: {self.timeouts}, RTF: {rtf:.2f}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--base-type', required=True, choices=['pmb2', 'omni'])
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--out-dir', default='./logs_explore',
                        help='STESSA cartella passata a exploration_logger.py')
    parser.add_argument('--seed', type=int, default=None,
                        help='Seed del generatore casuale (per la policy random_far)')
    parser.add_argument('--policy', default='random_far',
                        choices=['random_far', 'farthest', 'nearest'],
                        help="Selezione della frontiera: 'random_far' = originale "
                             "tirocinio; 'farthest' consigliata per la comparazione")
    parser.add_argument('--frontier-threshold', type=int, default=25,
                        help='Sotto questo numero di celle di frontiera valide '
                             'l esplorazione e dichiarata completa (uguale per le due basi!)')
    parser.add_argument('--max-nav-time', type=float, default=55.0,
                        help='Timeout [s sim] per raggiungere un goal di frontiera')
    parser.add_argument('--max-nav-time-home', type=float, default=90.0,
                        help='Timeout [s sim] per il ritorno alla base')
    parser.add_argument('--goal-clearance', type=float, default=0.35,
                        help='Distanza minima [m] che una frontiera deve avere da '
                             'qualsiasi muro/ostacolo per essere un goal valido. '
                             'Scarta i goal che cadrebbero nell inflation di Nav2 '
                             '(robot fermo + timeout). 0 = disattiva il filtro. '
                             'Alza (0.40-0.45) se il robot resta ancora bloccato; '
                             'abbassa (0.25-0.30) se dichiara completo troppo presto.')
    parser.add_argument('--wall-value-threshold', type=int, default=65,
                        help='Valore nella griglia /map sopra il quale una cella e '
                             'considerata muro/ostacolo (slam_toolbox: 0-100, '
                             '-1=ignoto). Default 65, coerente con occupied_thresh '
                             'di Nav2.')
    parser.add_argument('--blacklist-radius', type=float, default=1.0,
                        help='Raggio [m] entro cui un goal fallito/andato in timeout '
                             'esclude le frontiere successive. 0.6 (originale) e '
                             'troppo piccolo: il robot ripesca punti sulla stessa '
                             'frontiera d angolo e si ri-blocca. 1.0-1.5 esclude '
                             'tutta la zona problematica dopo un solo fallimento.')
    parser.add_argument('--goal-retract', type=float, default=0.40,
                        help='Arretramento [m] del goal dal bordo dell ignoto verso '
                             'il robot, cosi il target cade in spazio libero '
                             'confermato invece che sul confine libero/ignoto '
                             '(dove un muro appena rivelato lo rende irraggiungibile '
                             'e fa partire le recovery). 0 = manda il goal sulla '
                             'frontiera esatta (comportamento originale).')
    # parse_known_args per non crashare quando riceve --ros-args
    args, unknown = parser.parse_known_args()

    rclpy.init()
    node = ThesisExplorer(args)
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        # garantisce il summary anche su run interrotta a meta'
        node.write_summary()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
