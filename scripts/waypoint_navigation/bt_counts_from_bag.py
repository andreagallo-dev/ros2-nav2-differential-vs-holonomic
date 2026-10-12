#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bt_counts_from_bag.py  (v5 - esplorazione E waypoint, + piani dal bag)
=======================================================================
Ricostruisce i comportamenti del behavior tree di Nav2 (Spin, BackUp,
ClearCostmap, Wait, ComputePathToPose/ThroughPoses) dal topic
/behavior_tree_log di un rosbag2, finestrati sulla fase attiva della run.

Novita' v5 - piani globali dal bag: se il bag contiene /plan, per ogni leg
(regola: piano ATTIVO all'inizio della leg, altrimenti il primo durante)
viene estratto il PRIMO piano ricevuto mentre quella leg era attiva e
riscritto in plans/wp<leg>.csv accanto al log (backup .bak dei file del
logger). Motivo: il logger live salva l'ULTIMO replan della leg, che con
replanning ~1 Hz e' un moncherino calcolato a ridosso del traguardo; usato
come riferimento trasforma l'errore di tracking in una distanza-al-goal
(rampa lineare). Il PRIMO piano della leg e' il percorso inteso.

Supporta entrambe le pipeline della tesi (topic di stato auto-rilevato):
  - esplorazione : /exploration_status    <-> colonna log 'goal_idx'
  - waypoint     : /current_waypoint_idx  <-> colonna log 'waypoint_idx'
Semantica identica: primo valore >= 0 = inizio, primo -1 = fine run.

Novita' v4 - patch TEMPORIZZATO dei contatori nel log.csv:
gli eventi BT vengono riportati sull'asse dei tempi del log (tempo
simulato) ancorando i cambi di indice visti nel bag (tempo di parete) ai
cambi della colonna indice nel log, e i contatori vengono scritti come
FUNZIONI A GRADINO. Cosi':
  - l'ultima riga della finestra contiene i totali (usati dalle analisi
    aggregate), come prima;
  - gli incrementi riga-per-riga localizzano gli eventi nel tempo/spazio,
    il che serve alla figura recovery_map di analyze_results.py (marker
    sugli istanti di Spin/BackUp/Wait).
Se il log manca o gli ancoraggi sono insufficienti, ricade sul patch a
totali costanti (come v3) con un avviso. Viene inoltre aggiunta la colonna
'wait_count' se assente (in coda: non sposta le colonne esistenti).

ATTIVAZIONI = transizioni con previous_status == IDLE e current != IDLE:
coprono anche i nodi di servizio one-shot (es. pulizie costmap) che
saltano da IDLE a SUCCESS senza mai riportare RUNNING.

Uso:
  1) Ispezione:  python3 bt_counts_from_bag.py --bag bags/<run>
  2) Patch:      python3 bt_counts_from_bag.py --bag bags/<run> \
                     --patch-log logs/<run>/log.csv
     (scrive anche <run>/bt_counts.csv; backup log.csv.bak alla prima volta)

Nota definizione: nel BT di default il wrapper RecoveryNode condivide il
display name col nodo foglia di pianificazione, quindi i conteggi a
livello BT non coincidono col grep dei log testuali del planner. Stessa
definizione per tutte le run = confronto omogeneo.
Dipendenze: ROS 2 sourcato (rosbag2_py, rclpy, nav2_msgs) solo per la
lettura del bag; il resto e' numpy/stdlib.
"""
import argparse
import csv
import os
import shutil
from collections import Counter

import numpy as np

TERMINAL = ('RUNNING', 'SUCCESS', 'FAILURE')
CSV_TRANSITIONS = ('ACTIVATIONS', 'RUNNING', 'SUCCESS', 'FAILURE')
CATEGORIES = ('spin', 'backup', 'clear_global', 'clear_local',
              'clear_other', 'wait', 'planning')

# topic di stato -> colonna indice corrispondente nel log.csv
STATUS_CANDIDATES = [('/exploration_status', 'goal_idx'),
                     ('/current_waypoint_idx', 'waypoint_idx')]

# colonna log.csv <- categorie i cui eventi la incrementano
PATCH_COLS = {
    'spin_count': ('spin',),
    'backup_count': ('backup',),
    'wait_count': ('wait',),
    'clearcostmap_count': ('clear_global', 'clear_local', 'clear_other'),
    'planning_attempts': ('planning',),
}
CANONICAL_LABELS = {'spin_count': 'spins', 'backup_count': 'backups',
                    'wait_count': 'waits',
                    'clearcostmap_count': 'costmap_clears',
                    'planning_attempts': 'replans'}


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
    # import locali: cosi' il modulo resta importabile (e testabile) anche
    # senza un ambiente ROS attivo
    from rclpy.serialization import deserialize_message  # noqa: F401
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_dir, storage_id=storage_id),
                ConverterOptions(input_serialization_format='cdr',
                                 output_serialization_format='cdr'))
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    return reader, type_map


def scan_bag(bag_dir, storage_id, status_topic=None):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader, type_map = open_reader(bag_dir, storage_id)
    if '/behavior_tree_log' not in type_map:
        raise SystemExit(
            "Nel bag non c'e' /behavior_tree_log: niente da recuperare.")

    if status_topic is None:
        for topic, _ in STATUS_CANDIDATES:
            if topic in type_map:
                status_topic = topic
                break
    if status_topic is None or status_topic not in type_map:
        print('(!) Nessun topic di stato nel bag '
              f'({[t for t, _ in STATUS_CANDIDATES]}): conto su TUTTO il '
              'bag, finestra e patch temporizzato non disponibili.')
        status_topic = None

    msg_classes = {}

    def msg_class(topic):
        if topic not in msg_classes:
            msg_classes[topic] = get_message(type_map[topic])
        return msg_classes[topic]

    counts = {'window': Counter(), 'total': Counter()}
    act_by_name = Counter()
    events_window = {cat: [] for cat in CATEGORIES}   # attivazioni [ns]
    status_first = {}                                  # valore -> t_ns
    statuses_seen = set()
    t_start = t_end = None
    t_first = t_last = None
    has_plan_topic = '/plan' in type_map
    plans_first = {}          # leg -> [(x, y), ...]  (PRIMO /plan della leg)
    plans_at_start = {}       # leg -> piano ATTIVO all'inizio della leg
    last_plan = None          # /plan piu' recente visto nel bag
    plan_msgs = Counter()     # leg -> n. messaggi /plan visti
    cur_status = None

    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        if t_first is None:
            t_first = t_ns
        t_last = t_ns

        if status_topic is not None and topic == status_topic:
            msg = deserialize_message(raw, msg_class(topic))
            v = int(msg.data)
            cur_status = v
            if v not in status_first:
                status_first[v] = t_ns
                if v >= 0 and last_plan is not None:
                    plans_at_start[v] = last_plan
            if v >= 0 and t_start is None:
                t_start = t_ns
            elif v == -1 and t_end is None:
                t_end = t_ns

        elif has_plan_topic and topic == '/plan':
            plan_msgs[cur_status] += 1
            msg = deserialize_message(raw, msg_class(topic))
            last_plan = [(ps.pose.position.x, ps.pose.position.y)
                         for ps in msg.poses]
            if cur_status is not None and cur_status >= 0 \
                    and cur_status not in plans_first:
                plans_first[cur_status] = last_plan

        elif topic == '/behavior_tree_log':
            msg = deserialize_message(raw, msg_class(topic))
            in_window = ((status_topic is None)
                         or (t_start is not None
                             and (t_end is None or t_ns <= t_end)))
            for ev in msg.event_log:
                cur = str(ev.current_status).upper()
                prev = str(ev.previous_status).upper()
                statuses_seen.add(str(ev.current_status))
                cat = classify(ev.node_name)
                if cat is None:
                    continue
                if cur in TERMINAL and prev != cur:
                    counts['total'][(cat, cur)] += 1
                    if in_window:
                        counts['window'][(cat, cur)] += 1
                if prev == 'IDLE' and cur != 'IDLE':
                    act_by_name[ev.node_name] += 1
                    counts['total'][(cat, 'ACTIVATIONS')] += 1
                    if in_window:
                        counts['window'][(cat, 'ACTIVATIONS')] += 1
                        events_window[cat].append(t_ns)

    return {
        'counts': counts, 'act_by_name': act_by_name,
        'events_window': events_window, 'status_first': status_first,
        'status_topic': status_topic, 'statuses_seen': statuses_seen,
        'has_plan_topic': has_plan_topic, 'plans_first': plans_first,
        'plans_at_start': plans_at_start,
        'plan_msgs': dict(plan_msgs),
        'completed': t_end is not None,
        'start_offset_s': (t_start - t_first) / 1e9
                          if (t_start is not None and t_first is not None)
                          else float('nan'),
        'window_s': ((t_end if t_end is not None else t_last) - t_start) / 1e9
                    if t_start is not None else float('nan'),
        'bag_s': (t_last - t_first) / 1e9 if t_first is not None else 0.0,
    }


def canonical_window_counts(cw):
    return {col: int(sum(cw[(c, 'ACTIVATIONS')] for c in cats))
            for col, cats in PATCH_COLS.items()}


def write_detail_csv(path, res):
    with open(path, 'w', newline='') as f:
        f.write(f"# bag completato(-1 visto)={res['completed']} "
                f"finestra_s={res['window_s']:.1f} "
                f"inizio_finestra_dal_bag_s={res['start_offset_s']:.1f}\n")
        w = csv.writer(f)
        w.writerow(['category', 'transition', 'count_window', 'count_total'])
        for cat in CATEGORIES:
            for tr in CSV_TRANSITIONS:
                w.writerow([cat, tr, res['counts']['window'][(cat, tr)],
                            res['counts']['total'][(cat, tr)]])
    print(f'Dettaglio salvato in {path}')


# ---------------------------------------------------------------------- #
# Patch del log.csv                                                       #
# ---------------------------------------------------------------------- #
def read_log_rows(log_path):
    with open(log_path, newline='') as f:
        rows = list(csv.reader(f))
    if len(rows) < 2:
        raise SystemExit(f'{log_path} e\' vuoto.')
    return rows


def log_anchor_times(rows, idx_col_name):
    """Prima occorrenza (tempo sim) di ogni valore della colonna indice."""
    header = rows[0]
    if idx_col_name not in header or 't' not in header:
        return None, None
    i_t = header.index('t')
    i_v = header.index(idx_col_name)
    anchors, ts = {}, np.empty(len(rows) - 1)
    for k, r in enumerate(rows[1:]):
        try:
            t = float(r[i_t])
            v = int(float(r[i_v]))
        except (ValueError, IndexError):
            ts[k] = np.nan
            continue
        ts[k] = t
        if v not in anchors:
            anchors[v] = t
    return anchors, ts


def build_wall_to_sim(status_first, log_anchors):
    """Coppie (wall_s, sim_s) dai valori indice visti in ENTRAMBE le fonti,
    in ordine cronologico bag, tenendo solo coppie strettamente crescenti."""
    wall, sim = [], []
    for v, t_ns in sorted(status_first.items(), key=lambda kv: kv[1]):
        if v in log_anchors:
            w, s = t_ns / 1e9, log_anchors[v]
            if not wall or (w > wall[-1] and s > sim[-1]):
                wall.append(w)
                sim.append(s)
    return np.asarray(wall), np.asarray(sim)


def build_step_counts(row_ts, event_sim_times):
    """Contatore cumulativo per riga: numero di eventi con tempo <= t_riga."""
    ev = np.sort(np.asarray(event_sim_times, float))
    safe_ts = np.where(np.isfinite(row_ts), row_ts, -np.inf)
    return np.searchsorted(ev, safe_ts, side='right')


def choose_leg_plans(plans_at_start, plans_first):
    """Per ogni leg sceglie il piano di riferimento: quello ATTIVO all'inizio
    della leg (l'ultimo /plan pubblicato prima del cambio indice, che copre
    la leg dal primo metro); in mancanza, il primo pubblicato durante la
    leg. Ritorna (plans_leg, src_leg)."""
    plans_leg, src_leg = {}, {}
    for leg in sorted(set(plans_at_start) | set(plans_first)):
        pts = plans_at_start.get(leg)
        if pts:
            plans_leg[leg] = pts
            src_leg[leg] = 'attivo a inizio leg'
        elif plans_first.get(leg):
            plans_leg[leg] = plans_first[leg]
            src_leg[leg] = 'primo durante la leg'
    return plans_leg, src_leg


def write_plans(run_dir, plans_leg, src_leg=None):
    """Riscrive plans/wp<leg>.csv col PRIMO /plan di ogni leg estratto dal
    bag (= il percorso inteso della leg). Backup una tantum dei file
    preesistenti del logger (ultimo replan) in wp<leg>.csv.bak."""
    plans_dir = os.path.join(run_dir, 'plans')
    os.makedirs(plans_dir, exist_ok=True)
    for leg in sorted(plans_leg):
        pts = plans_leg[leg]
        if len(pts) < 2:
            print(f'  (leg {leg}: primo /plan con {len(pts)} punti, salto)')
            continue
        path = os.path.join(plans_dir, f'wp{leg}.csv')
        bak = path + '.bak'
        if os.path.exists(path) and not os.path.exists(bak):
            shutil.copy2(path, bak)
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['x', 'y'])
            for x, y in pts:
                w.writerow([f'{x:.5f}', f'{y:.5f}'])
        src = (src_leg or {}).get(leg, 'primo durante la leg')
        print(f'  plans/wp{leg}.csv <- /plan {src} ({len(pts)} punti)')


def patch_log(log_path, res, idx_col_name):
    canon = canonical_window_counts(res['counts']['window'])
    rows = read_log_rows(log_path)
    header = rows[0]

    bak = log_path + '.bak'
    if not os.path.exists(bak):
        shutil.copy2(log_path, bak)
        print(f'Backup creato: {bak}')

    # colonna wait_count aggiunta in coda se assente (non sposta le altre)
    if 'wait_count' not in header:
        header.append('wait_count')
        for r in rows[1:]:
            r.append('0')

    missing = [c for c in PATCH_COLS if c not in header]
    if missing:
        raise SystemExit(f'Colonne contatore mancanti nel log: {missing}')
    col_idx = {c: header.index(c) for c in PATCH_COLS}

    # tentativo di patch temporizzato
    anchors, row_ts = (None, None)
    if idx_col_name is not None:
        anchors, row_ts = log_anchor_times(rows, idx_col_name)
    mode = 'costante (fallback)'
    per_col_counts = None
    if anchors:
        wall, sim = build_wall_to_sim(res['status_first'], anchors)
        if wall.size >= 2:
            per_col_counts = {}
            for col, cats in PATCH_COLS.items():
                ev_wall = [t / 1e9 for c in cats
                           for t in res['events_window'][c]]
                ev_sim = np.interp(np.asarray(ev_wall), wall, sim) \
                    if ev_wall else np.asarray([])
                per_col_counts[col] = build_step_counts(row_ts, ev_sim)
            mode = f'temporizzato ({wall.size} ancoraggi indice)'
    if per_col_counts is None:
        print('(!) Ancoraggi insufficienti: patch a totali costanti '
              '(la recovery_map non potra\' localizzare gli eventi).')

    for k, r in enumerate(rows[1:]):
        for col, i in col_idx.items():
            val = (per_col_counts[col][k] if per_col_counts is not None
                   else canon[col])
            if i < len(r):
                r[i] = str(int(val))

    with open(log_path, 'w', newline='') as f:
        csv.writer(f).writerows(rows)
    print(f'Patchato {log_path} [{mode}]: '
          + ', '.join(f'{c}={canon[c]}' for c in PATCH_COLS))


# ---------------------------------------------------------------------- #
def main():
    ap = argparse.ArgumentParser(
        description='Ricostruisce i comportamenti BT (recovery, planning) '
                    'dal bag di una run (esplorazione o waypoint).')
    ap.add_argument('--bag', required=True)
    ap.add_argument('--patch-log', default=None,
                    help='log.csv della stessa run da patchare')
    ap.add_argument('--out-csv', default=None,
                    help='Dettaglio (default: bt_counts.csv accanto al log)')
    ap.add_argument('--status-topic', default=None,
                    help='Override del topic di stato (default: auto)')
    ap.add_argument('--idx-column', default=None,
                    help='Override della colonna indice nel log '
                         '(default: dedotta dal topic di stato)')
    ap.add_argument('--storage-id', default='sqlite3')
    ap.add_argument('--no-plans', action='store_true',
                    help='Non riscrivere plans/wp<N>.csv dal bag')
    args = ap.parse_args()

    res = scan_bag(args.bag, args.storage_id, args.status_topic)
    cw = res['counts']['window']
    canon = canonical_window_counts(cw)

    idx_col = args.idx_column
    if idx_col is None and res['status_topic'] is not None:
        idx_col = dict(STATUS_CANDIDATES).get(res['status_topic'])

    print(f"\nBag: {args.bag}  (durata {res['bag_s']:.1f} s, tempo bag)")
    print(f"Topic di stato: {res['status_topic']}  (colonna log: {idx_col})")
    print(f"Run completata (visto -1): {res['completed']}   "
          f"finestra: {res['window_s']:.1f} s "
          f"(inizia {res['start_offset_s']:.1f} s dopo l'inizio del bag)")
    print(f"Status BT visti nel bag: {sorted(res['statuses_seen'])}")

    print('\nColonne canoniche (finestra, ATTIVAZIONI = uscite da IDLE):')
    for col, label in CANONICAL_LABELS.items():
        print(f'  {label:15s} {canon[col]:5d}')

    print('\nDettaglio finestra (attivazioni / successi / fallimenti):')
    for cat in CATEGORIES:
        a, s, fl = (cw[(cat, tr)] for tr in
                    ('ACTIVATIONS', 'SUCCESS', 'FAILURE'))
        if a or s or fl:
            print(f'  {cat:13s} ACT={a:4d}  SUCCESS={s:4d}  FAILURE={fl:4d}')
    pf, pa = cw[('planning', 'FAILURE')], cw[('planning', 'ACTIVATIONS')]
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
        patch_log(args.patch_log, res, idx_col)
        if not args.no_plans:
            run_dir = os.path.dirname(args.patch_log) or '.'
            plans_leg, src_leg = choose_leg_plans(
                res.get('plans_at_start', {}), res['plans_first'])
            if plans_leg:
                print('Piani di riferimento per leg estratti dal bag:')
                write_plans(run_dir, plans_leg, src_leg)
            elif res['has_plan_topic']:
                print("(!) '/plan' presente nel bag ma nessun messaggio "
                      "durante una leg attiva: plans/ non riscritti.")
            else:
                print("(nota: '/plan' non registrato in questo bag: "
                      "plans/ lasciati come scritti dal logger)")


if __name__ == '__main__':
    main()
