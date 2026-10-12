#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bt_counts_from_bag.py  (v3)
===========================
Ricostruisce i comportamenti del behavior tree di Nav2 (Spin, BackUp,
ClearCostmap, Wait, ComputePathToPose) dal topic /behavior_tree_log
registrato in un rosbag2, limitandoli alla FINESTRA DI ESPLORAZIONE
definita da /exploration_status (dal primo goal >= 0 al primo -1):
la stessa finestra usata da analyze_exploration.py.

Novita' v3 - conteggio delle ATTIVAZIONI (uscite dallo stato IDLE):
i nodi di servizio "one-shot" come le pulizie costmap saltano direttamente
da IDLE a SUCCESS senza mai riportare RUNNING (verificato sui bag reali:
clear_global RUNNING=0 SUCCESS=32). Contare le transizioni a RUNNING li
perderebbe. Le colonne canoniche usano quindi le ATTIVAZIONI, definite come
transizioni con previous_status == IDLE e current_status != IDLE.

Per ogni categoria vengono contate:
  ACTIVATIONS -> attivazioni (uscite da IDLE)
  RUNNING     -> transizioni a RUNNING (per confronto/retrocompatibilita')
  SUCCESS     -> completamenti riusciti
  FAILURE     -> fallimenti (es. per la pianificazione i
                 "GridBased: failed to create plan" a livello BT)

Uso:
  1) Solo ispezione (contatori + node_name/status realmente visti nel BT):
       python3 bt_counts_from_bag.py --bag bags/pmb2_expl_rep1

  2) Contare, salvare il dettaglio in <run_dir>/bt_counts.csv e riscrivere
     le 4 colonne contatore canoniche nel log.csv della run:
       python3 bt_counts_from_bag.py --bag bags/pmb2_expl_rep1 \
           --patch-log logs_explore/pmb2_expl_rep1/log.csv

Note:
  - analyze_exploration.py legge SOLO il valore dei contatori all'ultima
    riga della finestra (d[col].iloc[-1]): il patch scrive quindi il
    totale-di-finestra (ATTIVAZIONI) su tutte le righe. Backup automatico
    in log.csv.bak alla prima scrittura.
  - Nel BT di default di Nav2 il nodo wrapper RecoveryNode ha lo STESSO
    display name del nodo foglia ComputePathToPose: i conteggi di
    pianificazione a livello BT includono quindi wrapper+foglia e NON
    coincidono con il grep dei log testuali del planner_server. E' una
    differenza di definizione, non un errore: usare la stessa definizione
    (questa) per tutte le run rende il confronto tra basi omogeneo.
Dipendenze: ambiente ROS 2 sourcato (rosbag2_py, rclpy, nav2_msgs).
"""
import argparse
import csv
import os
import shutil
from collections import Counter

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

TERMINAL = ('RUNNING', 'SUCCESS', 'FAILURE')
CSV_TRANSITIONS = ('ACTIVATIONS', 'RUNNING', 'SUCCESS', 'FAILURE')
CATEGORIES = ('spin', 'backup', 'clear_global', 'clear_local',
              'clear_other', 'wait', 'planning')

# categoria -> (colonna log.csv, etichetta nella tabella di analyze)
# I clear vengono sommati in un'unica colonna canonica.
CANONICAL = {
    'spin': ('spin_count', 'spins'),
    'backup': ('backup_count', 'backups'),
    'clear': ('clearcostmap_count', 'costmap_clears'),
    'planning': ('planning_attempts', 'replans'),
}


def classify(node_name):
    low = node_name.lower()
    if 'spin' in low:
        return 'spin'
    if 'backup' in low or 'back_up' in low:
        return 'backup'
    if 'clear' in low and 'costmap' in low:
        if 'global' in low:
            return 'clear_global'
        if 'local' in low:
            return 'clear_local'
        return 'clear_other'
    if 'computepath' in low:
        return 'planning'
    if 'wait' in low:
        return 'wait'
    return None


def open_reader(bag_dir, storage_id):
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_dir, storage_id=storage_id),
        ConverterOptions(input_serialization_format='cdr',
                         output_serialization_format='cdr'))
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    return reader, type_map


def scan_bag(bag_dir, storage_id):
    reader, type_map = open_reader(bag_dir, storage_id)

    if '/behavior_tree_log' not in type_map:
        raise SystemExit(
            "Nel bag non c'e' /behavior_tree_log: niente da recuperare.")
    has_status = '/exploration_status' in type_map
    if not has_status:
        print("(!) /exploration_status assente dal bag: conto su TUTTO il "
              "bag (finestra di esplorazione non ricostruibile).")

    msg_classes = {}

    def msg_class(topic):
        if topic not in msg_classes:
            msg_classes[topic] = get_message(type_map[topic])
        return msg_classes[topic]

    counts = {'window': Counter(), 'total': Counter()}  # (categoria, trans)
    act_by_name = Counter()       # node_name -> attivazioni (intero bag)
    statuses_seen = set()
    t_start = t_end = None        # tempi bag [ns] della finestra
    t_first = t_last = None

    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        if t_first is None:
            t_first = t_ns
        t_last = t_ns

        if topic == '/exploration_status':
            msg = deserialize_message(raw, msg_class(topic))
            if msg.data >= 0 and t_start is None:
                t_start = t_ns
            elif msg.data == -1 and t_end is None:
                t_end = t_ns

        elif topic == '/behavior_tree_log':
            msg = deserialize_message(raw, msg_class(topic))
            # messaggi in ordine temporale: se t_end e' gia' noto siamo
            # oltre il completamento (fase di homing) -> fuori finestra.
            in_window = ((not has_status)
                         or (t_start is not None
                             and (t_end is None or t_ns <= t_end)))
            for ev in msg.event_log:
                cur = str(ev.current_status).upper()
                prev = str(ev.previous_status).upper()
                statuses_seen.add(str(ev.current_status))
                cat = classify(ev.node_name)
                if cat is None:
                    continue
                hits = []
                if cur in TERMINAL and prev != cur:
                    hits.append(cur)
                if prev == 'IDLE' and cur != 'IDLE':
                    hits.append('ACTIVATIONS')
                    act_by_name[ev.node_name] += 1
                for h in hits:
                    counts['total'][(cat, h)] += 1
                    if in_window:
                        counts['window'][(cat, h)] += 1

    return {
        'counts': counts,
        'act_by_name': act_by_name,
        'statuses_seen': statuses_seen,
        'completed': t_end is not None,
        'start_offset_s': (t_start - t_first) / 1e9
                          if (t_start is not None and t_first is not None)
                          else float('nan'),
        'window_s': ((t_end if t_end is not None else t_last) - t_start) / 1e9
                    if t_start is not None else float('nan'),
        'bag_s': (t_last - t_first) / 1e9 if t_first is not None else 0.0,
    }


def canonical_window_counts(counts_window):
    """Le 4 colonne canoniche di log.csv = ATTIVAZIONI in finestra
    (i clear global/local/other vengono sommati)."""
    clear = sum(counts_window[(c, 'ACTIVATIONS')]
                for c in ('clear_global', 'clear_local', 'clear_other'))
    return {
        'spin_count': counts_window[('spin', 'ACTIVATIONS')],
        'backup_count': counts_window[('backup', 'ACTIVATIONS')],
        'clearcostmap_count': clear,
        'planning_attempts': counts_window[('planning', 'ACTIVATIONS')],
    }


def write_detail_csv(path, res):
    with open(path, 'w', newline='') as f:
        f.write(f"# bag completato(-1 visto)={res['completed']} "
                f"finestra_s={res['window_s']:.1f} "
                f"inizio_finestra_dal_bag_s={res['start_offset_s']:.1f}\n")
        w = csv.writer(f)
        w.writerow(['category', 'transition', 'count_window', 'count_total'])
        for cat in CATEGORIES:
            for tr in CSV_TRANSITIONS:
                w.writerow([cat, tr,
                            res['counts']['window'][(cat, tr)],
                            res['counts']['total'][(cat, tr)]])
    print(f'Dettaglio salvato in {path}')


def patch_log(log_path, canon):
    if not os.path.exists(log_path):
        raise SystemExit(f'log.csv non trovato: {log_path}')
    bak = log_path + '.bak'
    if not os.path.exists(bak):
        shutil.copy2(log_path, bak)
        print(f'Backup creato: {bak}')

    with open(log_path, newline='') as f:
        rows = list(csv.reader(f))
    if not rows:
        raise SystemExit(f'{log_path} e\' vuoto.')
    header = rows[0]
    try:
        idx = {c: header.index(c) for c in canon}
    except ValueError as e:
        raise SystemExit(f'Colonna contatore mancante nel log: {e}')

    for r in rows[1:]:
        for c, i in idx.items():
            if i < len(r):
                r[i] = str(int(canon[c]))

    with open(log_path, 'w', newline='') as f:
        csv.writer(f).writerows(rows)
    print(f'Patchato {log_path}: '
          + ', '.join(f'{c}={int(v)}' for c, v in canon.items()))


def main():
    ap = argparse.ArgumentParser(
        description='Ricostruisce i comportamenti BT (recovery, planning) '
                    'dal bag di una run di esplorazione.')
    ap.add_argument('--bag', required=True,
                    help='Cartella del rosbag2 (es. bags/pmb2_expl_rep1)')
    ap.add_argument('--patch-log', default=None,
                    help='log.csv della stessa run: riscrive le 4 colonne '
                         'contatore canoniche')
    ap.add_argument('--out-csv', default=None,
                    help='Dove salvare il dettaglio (default: bt_counts.csv '
                         'accanto a --patch-log, se dato)')
    ap.add_argument('--storage-id', default='sqlite3',
                    help="Storage del bag (default 'sqlite3' per i .db3)")
    args = ap.parse_args()

    res = scan_bag(args.bag, args.storage_id)
    cw = res['counts']['window']
    canon = canonical_window_counts(cw)

    print(f"\nBag: {args.bag}  (durata {res['bag_s']:.1f} s, tempo bag)")
    print(f"Esplorazione completata (visto -1): {res['completed']}   "
          f"finestra: {res['window_s']:.1f} s "
          f"(inizia {res['start_offset_s']:.1f} s dopo l'inizio del bag)")
    print(f"Status BT visti nel bag: {sorted(res['statuses_seen'])}")

    print('\nColonne canoniche (finestra, ATTIVAZIONI = uscite da IDLE):')
    for cat, (col, label) in CANONICAL.items():
        print(f'  {label:15s} {int(canon[col]):5d}')

    print('\nDettaglio finestra (attivazioni / successi / fallimenti):')
    for cat in CATEGORIES:
        a = cw[(cat, 'ACTIVATIONS')]
        s = cw[(cat, 'SUCCESS')]
        fl = cw[(cat, 'FAILURE')]
        if a or s or fl:
            print(f'  {cat:13s} ACT={a:4d}  SUCCESS={s:4d}  FAILURE={fl:4d}')
    pf = cw[('planning', 'FAILURE')]
    pa = cw[('planning', 'ACTIVATIONS')]
    if pa:
        print(f'\n  -> fallimenti di pianificazione (livello BT): {pf} su '
              f'{pa} attivazioni ({100.0 * pf / pa:.1f}%)')

    print('\nNode_name che hanno contribuito (attivazioni, INTERO bag):')
    for name, n in res['act_by_name'].most_common(20):
        print(f'  {n:5d}  {name}   -> {classify(name)}')

    out_csv = args.out_csv
    if out_csv is None and args.patch_log:
        out_csv = os.path.join(os.path.dirname(args.patch_log) or '.',
                               'bt_counts.csv')
    if out_csv:
        print()
        write_detail_csv(out_csv, res)
    if args.patch_log:
        patch_log(args.patch_log, canon)


if __name__ == '__main__':
    main()
