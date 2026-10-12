#!/usr/bin/env python3
"""
send_waypoints.py
=================
Invia la STESSA sequenza di waypoint a Nav2 usando goThroughPoses, garantendo
un movimento CONTINUO e SMOOTH senza che il robot si fermi ai punti intermedi,
identica per PMB2 e OmniBase.
La differenza tra le due basi e' gia' gestita a monte dal tuo launch file
(argomento base_type) e dalla configurazione Nav2 corrispondente; questo
script non sa nulla della cinematica del robot, manda solo dei goal.

Va eseguito DOPO che:
  - il bringup di Nav2 per la base scelta e' attivo
  - la localizzazione ha una initial pose corretta (se non viene settata
    automaticamente dal tuo launch, scommenta la chiamata a setInitialPose
    sotto)

NOTE SUI TEMPI (importante per la comparazione):
  - Il nodo BasicNavigator NON ha use_sim_time:=true di default: senza il fix
    qui sotto, navigator.get_clock().now() restituirebbe il tempo di PARETE del
    PC, non /clock di Gazebo. Lo forziamo a true in __main__ subito dopo aver
    creato il navigator. Cosi':
      1) la DURATA della run e' in tempo SIMULATO, coerente col metrics_logger
         (che deve girare anch'esso in use_sim_time) e con tutto lo stack Nav2;
      2) le PoseStamped inviate a Nav2 hanno timestamp in sim-time, evitando
         errori di estrapolazione TF ("message too old / future") dovuti al
         mismatch wall-time vs /clock.
  - Il WALL-CLOCK (time.time()) resta usato SOLO per il timeout di sicurezza:
    se Gazebo si blocca, /clock si ferma e un timeout in sim-time non
    scatterebbe mai (deadlock). Il wall-clock ci protegge comunque.
  - Il run_summary MANTIENE le colonne originali (total_waypoints, t_start,
    t_end, duration_s, result, succeeded) con gli stessi valori di prima
    (wall-clock), e AGGIUNGE in coda: duration_sim_s (durata in tempo
    simulato, la metrica corretta), t_sim_start_s, t_sim_end_s e rtf. Cosi'
    non rompe analyze_results.py, e in piu' hai il tempo simulato + un RTF
    utile da citare in tesi (quanto la sim girava vicino al tempo reale, e se
    una base caricava il simulatore piu' dell'altra, es. l'omni con 2 laser).

Uso:
    python3 send_waypoints.py --waypoints waypoints.yaml --base-type pmb2 --run-id 001 --out-dir ./logs
    python3 send_waypoints.py --waypoints waypoints.yaml --base-type omni --run-id 001 --out-dir ./logs

Il riepilogo run_summary.csv finisce nella STESSA cartella di run usata da
metrics_logger.py (<out-dir>/<base-type>_<run-id>/), cosi' tutto quello
che riguarda una run sta in un unico posto.
"""
import argparse
import csv
import math
import os
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.parameter import Parameter
from std_msgs.msg import Int32


def load_waypoints(path):
    with open(path, 'r') as f:
        data = yaml.safe_load(f)
    return data['waypoints']


def make_pose(navigator, x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    # Con use_sim_time:=true (forzato in main) questo stamp e' in tempo simulato,
    # coerente con l'intero stack Nav2.
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--waypoints', required=True, help='File YAML con la sequenza di waypoint')
    parser.add_argument('--base-type', required=True, choices=['pmb2', 'omni'])
    parser.add_argument('--run-id', default=None,
                        help='Identificatore della run (per i nomi dei file). Default: timestamp.')
    parser.add_argument('--out-dir', default='./logs',
                        help='Cartella padre, STESSA passata a metrics_logger.py')
    args, unknown = parser.parse_known_args()

    rclpy.init()
    navigator = BasicNavigator()

    # ------------------------------------------------------------------------ #
    # SIM TIME (fix cardine per la sincronizzazione dei tempi)                  #
    # Senza questo il TimeSource del nodo usa il clock di sistema. Forzando     #
    # use_sim_time:=true, il nodo si aggancia a /clock di Gazebo e             #
    # get_clock().now() restituisce il tempo simulato. Facciamo declare solo   #
    # se serve, poi set esplicito a True per garantire il valore E far scattare #
    # la callback del TimeSource che effettua lo switch a ROS_TIME.            #
    # ------------------------------------------------------------------------ #
    if not navigator.has_parameter('use_sim_time'):
        navigator.declare_parameter('use_sim_time', True)
    navigator.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

    # Publisher ausiliario per il logger
    idx_pub = navigator.create_publisher(Int32, '/current_waypoint_idx', 10)

    wps = load_waypoints(args.waypoints)
    run_id = args.run_id or time.strftime('%Y%m%d_%H%M%S')
    run_dir = os.path.join(args.out_dir, f'{args.base_type}_{run_id}')
    os.makedirs(run_dir, exist_ok=True)
    summary_path = os.path.join(run_dir, 'run_summary.csv')

    navigator.waitUntilNav2Active()
    # Da qui in poi /clock e' sicuramente attivo (lo stack e' up), quindi
    # get_clock().now() e' un tempo simulato valido.

    # Creazione della lista completa delle pose (stamp gia' in sim-time)
    poses = []
    for wp in wps:
        poses.append(make_pose(navigator, wp['x'], wp['y'], wp['yaw']))

    print(f"[{args.base_type}] Invio traiettoria fluida con {len(poses)} waypoint a Nav2...")

    # ------------------------------------------------------------------------ #
    # Due orologi, due scopi diversi                                            #
    #   - WALL-CLOCK: SOLO timeout di sicurezza (protegge da /clock congelato). #
    #   - SIM-CLOCK : DURATA UFFICIALE della run (coerente col metrics_logger). #
    # ------------------------------------------------------------------------ #
    t_wall_start = time.time()
    t_sim_start = navigator.get_clock().now()

    current_idx = 0
    idx_pub.publish(Int32(data=current_idx))

    # QUI AVVIENE LA MAGIA: Il robot calcola il percorso completo e parte!
    navigator.goThroughPoses(poses)

    # Monitoraggio dell'esecuzione in modo sicuro
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()

        # Aggiornamento indice waypoint basato su quelli rimanenti
        if feedback and hasattr(feedback, 'number_of_poses_remaining'):
            poses_remaining = feedback.number_of_poses_remaining
            active_idx = len(poses) - poses_remaining

            if active_idx != current_idx and active_idx < len(poses):
                current_idx = active_idx
                idx_pub.publish(Int32(data=current_idx))
                print(f"[{args.base_type}] Passaggio al waypoint intermedio {current_idx}")

        # Timeout di sicurezza in WALL-CLOCK (vedi nota in cima al file).
        if (time.time() - t_wall_start) > 300.0:
            print(f'[{args.base_type}] TIMEOUT GLOBALE SUPERATO (5 min wall), annullo il task.')
            navigator.cancelTask()
            break

        time.sleep(0.1)  # Previene il sovraccarico della CPU nel loop

    t_sim_end = navigator.get_clock().now()
    t_wall_end = time.time()

    result = navigator.getResult()
    ok = (result == TaskResult.SUCCEEDED)

    # Durata ufficiale = tempo SIMULATO trascorso.
    duration_sim_s = (t_sim_end - t_sim_start).nanoseconds / 1e9
    # Durata di parete = diagnostica per stimare il Real-Time Factor.
    duration_wall_s = t_wall_end - t_wall_start
    # RTF osservato (1.0 = tempo reale). NON e' una metrica del robot, ma dice
    # quanto e' affidabile qualsiasi confronto wall-clock e se una base carica
    # il simulatore piu' dell'altra.
    rtf = (duration_sim_s / duration_wall_s) if duration_wall_s > 0 else float('nan')

    print(f"[{args.base_type}] Traiettoria completata! Risultato: {result} | "
          f"durata simulata: {duration_sim_s:.2f}s "
          f"(wall: {duration_wall_s:.2f}s, RTF: {rtf:.2f})")

    # Salvataggio del riepilogo
    idx_pub.publish(Int32(data=-1))  # -1 = run terminata, segnale per il logger

    row = {
        # --- Colonne ORIGINALI: valori IDENTICI allo script di partenza -------
        # t_start / t_end / duration_s restano in WALL-CLOCK, esattamente come
        # prima. Cosi' qualsiasi cosa in analyze_results.py che legga queste
        # colonne (per nome O per posizione) continua a funzionare invariata.
        'total_waypoints': len(poses),
        't_start': t_wall_start,
        't_end': t_wall_end,
        'duration_s': duration_wall_s,
        'result': str(result),
        'succeeded': ok,
        # --- Colonne AGGIUNTE in coda: tempo SIMULATO + diagnostica RTF -------
        # Puramente additive: non spostano ne' rinominano le colonne sopra.
        'duration_sim_s': duration_sim_s,        # durata in tempo simulato (metrica corretta)
        't_sim_start_s': t_sim_start.nanoseconds / 1e9,
        't_sim_end_s': t_sim_end.nanoseconds / 1e9,
        'rtf': rtf,                              # Real-Time Factor osservato (sim / wall)
    }

    with open(summary_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=row.keys())
        writer.writeheader()
        writer.writerow(row)

    print(f"Riepilogo run salvato in: {summary_path}")

    # NON chiamare navigator.lifecycleShutdown() qui: spegnerebbe anche
    # lifecycle_manager_localization (quindi AMCL), facendo sparire tutta
    # la TF map->odom e quindi il robot da RViz pochi secondi dopo la fine
    # della run. Vogliamo solo chiudere il nodo di QUESTO script, lasciando
    # l'intero stack Nav2 (bringup) attivo per la prossima replica.
    navigator.destroyNode()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
