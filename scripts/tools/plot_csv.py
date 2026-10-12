import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import sys

def parse_ros_array(array_str):
    if pd.isna(array_str):
        return []
    s = str(array_str).strip('[]')
    if ';' in s:
        return [x.strip(" '\"") for x in s.split(';')]
    else:
        return [x.strip(" '\"") for x in s.split(',')]

def main():
    if len(sys.argv) < 2:
        print("Uso: python3 plot_csv.py <percorso_del_file.csv>")
        sys.exit(1)
        
    csv_file = sys.argv[1]
    print(f"[*] Lettura del file: {csv_file}...")
    
    try:
        df = pd.read_csv(csv_file)
    except Exception as e:
        print(f"Errore nella lettura: {e}")
        sys.exit(1)
        
    df['t_norm'] = df['t'] - df['t'].iloc[0]
    t_norm = df['t_norm'].values 
    
    R = 0.0985      # Raggio ruota
    D = 0.4044      # Distanza tra le ruote (carreggiata PMB2/TIAGo)
    
    pos_l_list, pos_r_list = [], []
    
    for i in range(len(df)):
        row = df.iloc[i]
        j_names = parse_ros_array(row['joint_names'])
        j_pos_str = parse_ros_array(row['joint_positions']) 
        
        try:
            idx_l = next(idx for idx, n in enumerate(j_names) if 'wheel' in n and 'left' in n)
            idx_r = next(idx for idx, n in enumerate(j_names) if 'wheel' in n and 'right' in n)
            
            pl, pr = float(j_pos_str[idx_l]), float(j_pos_str[idx_r])
            
            if np.isnan(pl) or np.isnan(pr):
                pos_l_list.append(np.nan)
                pos_r_list.append(np.nan)
            else:
                pos_l_list.append(pl)
                pos_r_list.append(pr)
        except Exception:
            pos_l_list.append(np.nan)
            pos_r_list.append(np.nan)
            
    df['pos_l'] = pos_l_list
    df['pos_r'] = pos_r_list

    # TEMPO
    df['dt'] = df['t'].diff()
    df.loc[df['dt'] == 0, 'dt'] = 1e-6
    
    # DERIVATE POSIZIONE RUOTE
    df['dp_l'] = df['pos_l'].diff()
    df['dp_r'] = df['pos_r'].diff()
    df.loc[df['dp_l'] > np.pi, 'dp_l'] -= 2 * np.pi
    df.loc[df['dp_l'] < -np.pi, 'dp_l'] += 2 * np.pi
    df.loc[df['dp_r'] > np.pi, 'dp_r'] -= 2 * np.pi
    df.loc[df['dp_r'] < -np.pi, 'dp_r'] += 2 * np.pi

    df['w_l'] = df['dp_l'] / df['dt']
    df['w_r'] = df['dp_r'] / df['dt']
    
    # 1. CALCOLO LINEARE (uguale a prima)
    raw_wheel_v = (df['w_l'] + df['w_r']) * R / 2.0
    df['wheel_v_smooth'] = raw_wheel_v.rolling(window=10, min_periods=1).mean().fillna(0)
    
    df['dx'] = df['gt_x'].diff()
    df['dy'] = df['gt_y'].diff()
    raw_v = (df['dx']/df['dt']) * np.cos(df['gt_yaw']) + (df['dy']/df['dt']) * np.sin(df['gt_yaw'])
    raw_v = raw_v.clip(lower=-0.5, upper=1.0) 
    df['gt_v_smooth'] = raw_v.rolling(window=10, min_periods=1).mean().fillna(0)

    # 2. CALCOLO ANGOLARE (Nuovo!)
    # w_robot = R/D * (w_destra - w_sinistra)
    raw_wheel_w = (df['w_r'] - df['w_l']) * R / D
    df['wheel_w_smooth'] = raw_wheel_w.rolling(window=10, min_periods=1).mean().fillna(0)
    
    # Derivata imbardata (Ground Truth)
    df['dyaw'] = df['gt_yaw'].diff()
    df.loc[df['dyaw'] > np.pi, 'dyaw'] -= 2 * np.pi
    df.loc[df['dyaw'] < -np.pi, 'dyaw'] += 2 * np.pi
    raw_w = df['dyaw'] / df['dt']
    raw_w = raw_w.clip(lower=-1.5, upper=1.5)
    df['gt_w_smooth'] = raw_w.rolling(window=10, min_periods=1).mean().fillna(0)

    # DATI PER IL PLOT
    cmd_v = df['cmd_vx'].fillna(0).values
    cmd_w = df['cmd_wz'].fillna(0).values
    
    wv_v = df['wheel_v_smooth'].values
    gt_v = df['gt_v_smooth'].values
    
    wv_w = df['wheel_w_smooth'].values
    gt_w = df['gt_w_smooth'].values
    
    # SETUP GRAFICO DOPPIO
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
    
    # Plot 1: Lineare
    ax1.plot(t_norm, cmd_v, label="Comando Nav2 (cmd_vx)", color='limegreen', linewidth=2, alpha=0.9)
    ax1.plot(t_norm, wv_v, label="Velocità calcolata dalle Ruote", color='royalblue', linewidth=5, alpha=0.8)
    ax1.plot(t_norm, gt_v, label="Velocità Reale (Ground Truth)", color='darkred', linestyle='--', linewidth=2)
    ax1.set_title("Analisi Slittamento - Velocità Lineare (Avanti/Indietro)")
    ax1.set_ylabel("m/s")
    ax1.legend()
    ax1.grid(True, linestyle=':', alpha=0.7)
    
    # Plot 2: Angolare
    ax2.plot(t_norm, cmd_w, label="Comando Nav2 (cmd_wz)", color='limegreen', linewidth=2, alpha=0.9)
    ax2.plot(t_norm, wv_w, label="Rotazione calcolata dalle Ruote", color='royalblue', linewidth=5, alpha=0.8)
    ax2.plot(t_norm, gt_w, label="Rotazione Reale (Ground Truth)", color='darkred', linestyle='--', linewidth=2)
    ax2.set_title("Analisi Slittamento - Velocità Angolare (Rotazione)")
    ax2.set_xlabel("Tempo (s)")
    ax2.set_ylabel("rad/s")
    ax2.legend()
    ax2.grid(True, linestyle=':', alpha=0.7)
    
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()
