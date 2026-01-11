import pandas as pd
import matplotlib.pyplot as plt
import json
import pybullet as p
import pybullet_data
import numpy as np
import os
import matplotlib.gridspec as gridspec

# --- CONFIGURACIÓN ---
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RESULTADOS_DIR = os.path.join(SCRIPT_DIR, "..", "resultados")
ASSETS_DIR = os.path.join(SCRIPT_DIR, "..", "assets")
URDF_PATH = os.path.join(ASSETS_DIR, "segwayRobot.urdf")
CSV_PATH = os.path.join(RESULTADOS_DIR, "historial_fitness.csv")
JSON_PATH = os.path.join(RESULTADOS_DIR, "mejores_parametros.json")

# Estilo para reporte académico
plt.style.use('seaborn-v0_8-paper')
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Times New Roman'] + plt.rcParams['font.serif']

def plot_convergence():
    """GRÁFICO 1: Historia Evolutiva (Español)"""
    print("Generando gráfico de convergencia...")
    try:
        df = pd.read_csv(CSV_PATH)
    except FileNotFoundError:
        print("Error: No se encuentra historial_fitness.csv")
        return

    fig, ax = plt.subplots(figsize=(6, 4))
    
    generations = df['Gen']
    mean_fit = df['Avg_Fit']
    best_fit = df['Best_Fit']
    std_dev = df['Std_Dev_Fit']

    ax.fill_between(generations, mean_fit - std_dev, mean_fit + std_dev, 
                    color='orange', alpha=0.2, label='Desviación Estándar')
    
    ax.plot(generations, mean_fit, color='orange', linestyle='--', label='Fitness Promedio')
    ax.plot(generations, best_fit, color='blue', linewidth=1.5, label='Mejor Fitness')

    ax.set_title("Análisis de Convergencia Evolutiva")
    ax.set_xlabel("Generación")
    ax.set_ylabel("Puntaje de Aptitud (Fitness)")
    ax.legend(loc='lower right')
    ax.grid(True, linestyle=':', alpha=0.6)
    
    plt.tight_layout()
    plt.savefig(os.path.join(RESULTADOS_DIR, "plot_convergence.png"), dpi=300)
    print(" -> Guardado: plot_convergence.png")
    plt.close()

def run_validation_and_plot_dynamics():
    """GRÁFICO 2: Test de Impacto (Español)"""
    print("Ejecutando validación de impacto...")
    
    if not os.path.exists(JSON_PATH):
        print(f"Error: No se encuentra {JSON_PATH}")
        return

    with open(JSON_PATH, 'r') as f:
        data = json.load(f)
        p_vals = data["parameters"]

    kp_bal = p_vals["KP_BALANCE"]
    kp_rec = p_vals["KP_RECOVERY"]
    rec_thresh = p_vals["RECOVERY_THRESHOLD"]
    lerp_alpha = p_vals.get("LERP_ALPHA", 0.1)
    
    if p.isConnected(): p.disconnect()
    p.connect(p.DIRECT) 
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1./240.)
    p.loadURDF("plane.urdf")
    
    start_pos = [0,0,0.23]
    try:
        robotId = p.loadURDF(URDF_PATH, start_pos, [0,0,0,1])
    except:
        print("Error cargando URDF")
        p.disconnect()
        return
    
    log_time, log_tilt, log_kp = [], [], []
    max_steps = int(240 * 5.0) 
    kick_step = int(240 * 2.0)
    
    in_recovery = False
    current_kp = kp_bal 
    
    for step in range(max_steps):
        t = step * (1./240.)
        _, orn = p.getBasePositionAndOrientation(robotId)
        tilt = p.getEulerFromQuaternion(orn)[1]
        
        # Simular Impacto
        if step == kick_step:
            p.applyExternalForce(robotId, -1, [500.0, 0, 0], [0, 0, 0], p.WORLD_FRAME)
            
        # Lógica de Recuperación (State Machine)
        if not in_recovery and abs(tilt) > rec_thresh:
            in_recovery = True
        elif in_recovery and abs(tilt) < (rec_thresh * 0.5):
            in_recovery = False
            
        target_kp = kp_rec if in_recovery else kp_bal
        current_kp += (target_kp - current_kp) * lerp_alpha
            
        if step % 2 == 0:
            log_time.append(t)
            log_tilt.append(np.degrees(tilt))
            log_kp.append(current_kp)
            
        # Control simple PD para visualización
        torque = current_kp * (0 - tilt) 
        torque += (current_kp * 0.05) * (0 - p.getBaseVelocity(robotId)[1][0]) * -1

        p.setJointMotorControl2(robotId, 0, p.TORQUE_CONTROL, force=torque)
        p.setJointMotorControl2(robotId, 1, p.TORQUE_CONTROL, force=torque)
        p.stepSimulation()
        
    p.disconnect()
    
    # --- Graficado ---
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(6, 6), sharex=True)
    
    ax1.plot(log_time, log_tilt, color='#003366', label='Ángulo Inclinación')
    ax1.axvline(x=2.0, color='red', linestyle='--', label='Impacto (500N)')
    ax1.axhline(y=np.degrees(rec_thresh), color='orange', linestyle=':', label='Umbral')
    ax1.axhline(y=-np.degrees(rec_thresh), color='orange', linestyle=':')
    ax1.set_ylabel("Ángulo (grados)")
    ax1.set_title("Respuesta al Impulso Externo")
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    
    ax2.plot(log_time, log_kp, color='#8B0000', linewidth=2, label=r'Respuesta $K_p$')
    ax2.set_ylabel(r"Ganancia ($K_p$)")
    ax2.set_xlabel("Tiempo (s)")
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(os.path.join(RESULTADOS_DIR, "plot_impulse_response_heavy.png"), dpi=300)
    print(" -> Guardado: plot_impulse_response_heavy.png")
    plt.close()    

def run_full_phases_plot():
    """GRÁFICO: Trayectoria Compleja + Diagnóstico (Español)"""
    print("Generando gráfico de trayectoria compleja...")

    if not os.path.exists(JSON_PATH):
        return

    with open(JSON_PATH, 'r') as f:
        data = json.load(f)
        p_vals = data["parameters"]

    # Parámetros 
    kp_bal, kd_bal = p_vals["KP_BALANCE"], p_vals["KD_BALANCE"]
    kp_fwd, kd_fwd = p_vals["KP_FORWARD"], p_vals["KD_FORWARD"]
    kp_turn, kd_turn = p_vals["KP_TURNING"], p_vals["KD_TURNING"]
    kp_rec, kd_rec = p_vals["KP_RECOVERY"], p_vals["KD_RECOVERY"]
    
    kp_pos = p_vals.get("KP_POSITION", 0.1) 
    tgt_tilt = p_vals["TARGET_TILT"]
    rec_thresh = p_vals["RECOVERY_THRESHOLD"]
    lerp_alpha = p_vals.get("LERP_ALPHA", 0.2)
    kp_yaw = p_vals.get("KP_YAW", p_vals.get("TURNING_SPEED", 20.0))
    
    # Simulación
    if p.isConnected(): p.disconnect()
    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1./240.)
    p.loadURDF("plane.urdf")
    
    try:
        start_pos = [0,0,0.23]
        robotId = p.loadURDF(URDF_PATH, start_pos, [0,0,0,1])
    except:
        p.disconnect()
        return

    # Secuencia de Fases
    phase_seq = [
        0, 1,           # Calentamiento
        0, 2, 0, 1,     # Lado 1
        0, 2, 0, 1,     # Lado 2
        0, 2, 0, 1,     # Lado 3
        0               # Final
    ]
    
    LOG_START_PHASE_IDX = 2 
    phase_names = {0: "Equilibrio", 1: "Avance", 2: "Giro"} # Traducción
    phase_colors_map = {0: "#1f77b4", 1: "#2ca02c", 2: "#ff7f0e"} 
    bg_colors = {0: "#e6f2ff", 1: "#e6ffe6", 2: "#fff0e6"}
    
    PHASE_DURATION = 4.0 
    total_time = len(phase_seq) * PHASE_DURATION
    max_steps = int(240 * total_time)

    current_kp, current_kd = kp_bal, kd_bal
    in_recovery = False
    phase_idx = 0
    next_phase_time = PHASE_DURATION
    
    initial_pos, _ = p.getBasePositionAndOrientation(robotId)
    target_y = initial_pos[1]
    
    _, start_orn = p.getBasePositionAndOrientation(robotId)
    start_yaw = p.getEulerFromQuaternion(start_orn)[2]
    target_yaw_abs = start_yaw 
    TARGET_YAW_PER_PHASE = np.radians(90.0) 
    
    log_t, log_tilt, log_kp, log_phase, log_x, log_y, log_yaw = [], [], [], [], [], [], []

    for step in range(max_steps):
        t = step * (1./240.)
        
        # Gestión de Fases
        if t >= next_phase_time and phase_idx < len(phase_seq) - 1:
            phase_idx += 1
            next_phase_time += PHASE_DURATION
            
            if phase_seq[phase_idx] == 0: # Entrar en Equilibrio
                pos_temp, _ = p.getBasePositionAndOrientation(robotId)
                target_y = pos_temp[1]
            
            if phase_seq[phase_idx] == 2: # Entrar en Giro
                _, orn_temp = p.getBasePositionAndOrientation(robotId)
                yaw_curr = p.getEulerFromQuaternion(orn_temp)[2]
                target_yaw_abs = yaw_curr + TARGET_YAW_PER_PHASE

        cur_phase = phase_seq[phase_idx]
        pos, orn = p.getBasePositionAndOrientation(robotId)
        euler = p.getEulerFromQuaternion(orn)
        tilt, yaw = euler[1], euler[2]
        _, ang_vel = p.getBaseVelocity(robotId)
        
        # Check Recuperación
        if not in_recovery and abs(tilt) > rec_thresh: in_recovery = True
        elif in_recovery and abs(tilt) < (rec_thresh * 0.5): in_recovery = False

        target_kp, target_kd = kp_bal, kd_bal
        des_tilt, turn_val = 0.0, 0.0

        if in_recovery:
            target_kp, target_kd = kp_rec, kd_rec
        elif cur_phase == 0: # Equilibrio
             target_kp, target_kd = kp_bal, kd_bal
             err_y = target_y - pos[1]
             des_tilt = np.clip(kp_pos * err_y, -0.15, 0.15)
        elif cur_phase == 1: # Avance
            target_kp, target_kd = kp_fwd, kd_fwd
            des_tilt = tgt_tilt 
        elif cur_phase == 2: # Giro
            target_kp, target_kd = kp_turn, kd_turn
            yaw_err = target_yaw_abs - yaw
            while yaw_err > np.pi: yaw_err -= 2*np.pi
            while yaw_err < -np.pi: yaw_err += 2*np.pi
            turn_val = np.clip(kp_yaw * yaw_err, -3.0, 3.0)

        current_kp += (target_kp - current_kp) * lerp_alpha
        current_kd += (target_kd - current_kd) * lerp_alpha

        tilt_err = tilt - des_tilt
        out = (current_kp * tilt_err) + (current_kd * ang_vel[0])
        
        v_l = np.clip(out + turn_val, -25, 25)
        v_r = np.clip(out - turn_val, -25, 25)
        
        p.setJointMotorControl2(robotId, 0, p.VELOCITY_CONTROL, targetVelocity=v_l, force=2.5)
        p.setJointMotorControl2(robotId, 1, p.VELOCITY_CONTROL, targetVelocity=v_r, force=2.5)
        p.stepSimulation()

        # Logging
        if step % 5 == 0 and phase_idx >= LOG_START_PHASE_IDX:
            log_t.append(t)
            log_tilt.append(np.degrees(tilt))
            log_kp.append(current_kp)
            log_yaw.append(np.degrees(yaw))
            log_phase.append(cur_phase)
            log_x.append(pos[0])
            log_y.append(pos[1])

    p.disconnect()

    if not log_t: return

    # Normalización para Gráficas
    start_time = log_t[0]
    start_x, start_y = log_x[0], log_y[0]
    start_yaw_plot = log_yaw[0]
    
    norm_t = [t - start_time for t in log_t]
    norm_x = [x - start_x for x in log_x]
    norm_y = [y - start_y for y in log_y]
    
    norm_yaw = []
    for y in log_yaw:
        diff = y - start_yaw_plot
        if diff > 180: diff -= 360
        elif diff < -180: diff += 360
        norm_yaw.append(diff)

    # --- Graficado Complejo ---
    fig = plt.figure(figsize=(12, 8))
    gs = gridspec.GridSpec(3, 2, width_ratios=[1.3, 1], height_ratios=[1, 1, 1]) 

    ax_tilt = plt.subplot(gs[0, 0])
    ax_yaw  = plt.subplot(gs[1, 0], sharex=ax_tilt)
    ax_kp   = plt.subplot(gs[2, 0], sharex=ax_tilt)
    ax_map  = plt.subplot(gs[:, 1]) 

    # 1. TILT
    ax_tilt.plot(norm_t, log_tilt, color='#003366', linewidth=1)
    ax_tilt.set_ylabel("Inclinación (deg)")
    ax_tilt.set_title("1. Respuesta de Equilibrio")
    ax_tilt.grid(True, alpha=0.3)

    # 2. YAW
    ax_yaw.plot(norm_t, norm_yaw, color='#800080', linewidth=1.5)
    ax_yaw.set_ylabel("Rumbo / Yaw (deg)")
    ax_yaw.set_title("2. Respuesta de Giro")
    ax_yaw.grid(True, alpha=0.3)
    ax_yaw.axhline(90, color='gray', linestyle=':', alpha=0.5)

    # 3. KP GAIN
    ax_kp.plot(norm_t, log_kp, color='#8B0000', linewidth=1.5)
    ax_kp.set_ylabel(r"Ganancia $K_p$")
    ax_kp.set_xlabel("Tiempo (s)")
    ax_kp.set_title("3. Adaptación del Controlador")
    ax_kp.grid(True, alpha=0.3)

    # Fondos de Fase
    change_points = [0] + [i for i in range(1, len(log_phase)) if log_phase[i] != log_phase[i-1]] + [len(log_phase)-1]
    
    for i in range(len(change_points)-1):
        idx0, idx1 = change_points[i], change_points[i+1]
        ph = log_phase[idx0]
        t0, t1 = norm_t[idx0], norm_t[idx1]
        
        c_bg = bg_colors.get(ph, "white")
        for ax in [ax_tilt, ax_yaw, ax_kp]:
            ax.axvspan(t0, t1, color=c_bg, alpha=0.5, lw=0)
        
        if (t1 - t0) > 1.0: 
             ax_tilt.text(t0 + 0.2, ax_tilt.get_ylim()[1]*0.85, phase_names[ph], fontsize=7, color='#444')

    # 4. MAPA XY
    ax_map.set_title("4. Trayectoria del Robot")
    ax_map.set_xlabel("Posición X (m)")
    ax_map.set_ylabel("Posición Y (m)")
    ax_map.axis('equal') 
    ax_map.grid(True, linestyle=':', alpha=0.5)

    for i in range(len(change_points)-1):
        idx0, idx1 = change_points[i], change_points[i+1]
        ph = log_phase[idx0]
        segment_x = norm_x[idx0 : min(idx1+1, len(norm_x))]
        segment_y = norm_y[idx0 : min(idx1+1, len(norm_y))]
        label_txt = phase_names[ph] if i < 5 else "" 
        
        ax_map.plot(segment_x, segment_y, 
                    color=phase_colors_map[ph], 
                    linewidth=2.5 if ph == 1 else 1.5,
                    label=label_txt)

    ax_map.scatter(0, 0, c='black', s=60, marker='o', label='Inicio', zorder=5)
    ax_map.scatter(norm_x[-1], norm_y[-1], c='red', s=60, marker='X', label='Fin', zorder=5)
    
    handles, labels = ax_map.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax_map.legend(by_label.values(), by_label.keys(), loc='best', fontsize='small')

    plt.tight_layout()
    plt.savefig(os.path.join(RESULTADOS_DIR, "plot_complex_trajectory.png"), dpi=300)
    print(f" -> Guardado: plot_complex_trajectory.png")
    plt.close()

if __name__ == "__main__":
    plot_convergence()
    run_validation_and_plot_dynamics()
    run_full_phases_plot()
    print("\n¡Todos los gráficos generados!")