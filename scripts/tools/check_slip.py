import os
import sys
import sqlite3
import math
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def get_db3_path(bag_dir):
    for root, dirs, files in os.walk(bag_dir):
        for file in files:
            if file.endswith('.db3'):
                return os.path.join(root, file)
    return None

def main():
    if len(sys.argv) < 2:
        print("Uso: python3 check_slip.py <percorso_della_cartella_rosbag>")
        sys.exit(1)
        
    bag_dir = sys.argv[1]
    db3_path = get_db3_path(bag_dir)
    
    if not db3_path:
        print(f"File .db3 non trovato in '{bag_dir}'.")
        sys.exit(1)
        
    conn = sqlite3.connect(db3_path)
    cursor = conn.cursor()
    
    cursor.execute("SELECT id, name, type FROM topics")
    topics = {name: (t_id, t_type) for t_id, name, t_type in cursor.fetchall()}
    
    id_js, type_js = topics.get('/joint_states', (None, None))
    id_ms, type_ms = topics.get('/gazebo/model_states', (None, None))
    id_cv, type_cv = topics.get('/cmd_vel', (None, None))
    
    if not id_js or not id_ms:
        print("Errore: I topic base mancano nella bag.")
        sys.exit(1)
        
    JS_Msg = get_message(type_js)
    MS_Msg = get_message(type_ms)
    CV_Msg = get_message(type_cv) if id_cv else None
    
    time_js, vel_js = [], []
    time_gt, vel_gt = [], []
    time_cv, vel_cv = [], []
    
    WHEEL_RADIUS = 0.0985 
    robot_name_in_gazebo = None

    cursor.execute(
        "SELECT timestamp, topic_id, data FROM messages ORDER BY timestamp"
    )
    
    print(f"[*] Lettura del database: {db3_path}")
    print("[*] Elaborazione dei dati in corso...")
    
    for timestamp, topic_id, data in cursor.fetchall():
        if topic_id == id_js:
            msg = deserialize_message(data, JS_Msg)
            try:
                idx_l = msg.name.index('wheel_left_joint')
                idx_r = msg.name.index('wheel_right_joint')
                w_l = msg.velocity[idx_l]
                w_r = msg.velocity[idx_r]
                
                v_wheels = (w_l + w_r) * WHEEL_RADIUS / 2.0
                time_js.append(timestamp)
                vel_js.append(v_wheels)
            except (ValueError, IndexError):
                pass
                
        elif topic_id == id_cv and CV_Msg:
            msg = deserialize_message(data, CV_Msg)
            time_cv.append(timestamp)
            vel_cv.append(msg.linear.x)
                
        elif topic_id == id_ms:
            msg = deserialize_message(data, MS_Msg)
            
            # Cerca SPECIFICAMENTE "tiago" o "pmb2"
            if robot_name_in_gazebo is None:
                for name in msg.name:
                    nome_min = name.lower()
                    if 'tiago' in nome_min or 'pmb2' in nome_min:
                        robot_name_in_gazebo = name
                        print(f"[*] Bingo! Robot identificato su Gazebo come: '{robot_name_in_gazebo}'")
                        break
                        
            if robot_name_in_gazebo and robot_name_in_gazebo in msg.name:
                idx = msg.name.index(robot_name_in_gazebo)
                v_gt_x = msg.twist[idx].linear.x
                v_gt_y = msg.twist[idx].linear.y
                
                qw, qx, qy, qz = msg.pose[idx].orientation.w, msg.pose[idx].orientation.x, msg.pose[idx].orientation.y, msg.pose[idx].orientation.z
                yaw = math.atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))
                
                v_forward = v_gt_x * math.cos(yaw) + v_gt_y * math.sin(yaw)
                
                time_gt.append(timestamp)
                vel_gt.append(v_forward)

    if not time_js or not time_gt:
        print("Dati insufficienti per generare il grafico.")
        sys.exit(1)

    # Allinea i tempi partendo da zero
    t0 = min(time_js[0] if time_js else float('inf'), 
             time_gt[0] if time_gt else float('inf'), 
             time_cv[0] if time_cv else float('inf'))
             
    time_js = [(t - t0) / 1e9 for t in time_js]
    time_gt = [(t - t0) / 1e9 for t in time_gt]
    time_cv = [(t - t0) / 1e9 for t in time_cv]

    # Stampa i valori massimi
    print(f"\n--- RISULTATI ---")
    if vel_cv: print(f"Max Velocità Comandata (Nav2): {max(vel_cv):.4f} m/s")
    if vel_js: print(f"Max Velocità Teorica (Ruote):  {max(vel_js):.4f} m/s")
    if vel_gt: print(f"Max Velocità Reale (Gazebo):   {max(vel_gt):.4f} m/s")

    # Plot
    plt.figure(figsize=(10, 6))
    
    if time_cv:
        plt.plot(time_cv, vel_cv, label="Comando Inviato (/cmd_vel)", color='limegreen', linewidth=1.5, alpha=0.8)
    
    plt.plot(time_js, vel_js, label="Velocità ruote (Teorica)", color='royalblue', linewidth=4.5, alpha=0.9)
    plt.plot(time_gt, vel_gt, label="Velocità Ground-Truth (Reale)", color='darkred', linestyle='--', linewidth=2)
    
    plt.title("Test Slittamento: TIAGo / PMB2 (Differenziale)")
    plt.xlabel("Tempo di esecuzione (secondi)")
    plt.ylabel("Velocità Lineare (m/s)")
    plt.legend()
    plt.grid(True, linestyle=':', alpha=0.7)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
