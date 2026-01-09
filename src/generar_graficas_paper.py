import pandas as pd
import matplotlib.pyplot as plt
import json
import pybullet as p
import pybullet_data
import numpy as np
import os
import time

# --- CONFIGURACIÓN ---
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RESULTADOS_DIR = os.path.join(SCRIPT_DIR, "..", "resultados")
ASSETS_DIR = os.path.join(SCRIPT_DIR, "..", "assets")
URDF_PATH = os.path.join(ASSETS_DIR, "segwayRobot.urdf")
CSV_PATH = os.path.join(RESULTADOS_DIR, "historial_fitness.csv")
JSON_PATH = os.path.join(RESULTADOS_DIR, "mejores_parametros.json")

# Configurar Matplotlib para estilo académico (IEEE)
plt.style.use('seaborn-v0_8-paper')
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Times New Roman'] + plt.rcParams['font.serif']

def plot_convergence():
    """GRÁFICO 1: Historia Evolutiva (desde el CSV)"""
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

    # Región de desviación estándar
    ax.fill_between(generations, mean_fit - std_dev, mean_fit + std_dev, 
                    color='orange', alpha=0.2, label='Std Dev (Diversity)')
    
    # Líneas principales
    ax.plot(generations, mean_fit, color='orange', linestyle='--', label='Average Fitness')
    ax.plot(generations, best_fit, color='blue', linewidth=1.5, label='Best Fitness')

    ax.set_title("Evolutionary Convergence Analysis")
    ax.set_xlabel("Generation")
    ax.set_ylabel("Fitness Score")
    ax.legend(loc='lower right')
    ax.grid(True, linestyle=':', alpha=0.6)
    
    plt.tight_layout()
    plt.savefig(os.path.join(RESULTADOS_DIR, "plot_convergence.png"), dpi=300)
    print(" -> Guardado: plot_convergence.png")
    plt.close()

def run_validation_and_plot_dynamics():
    """GRÁFICOS 2, 3 y 4: Simulación con los MEJORES GENES"""
    print("Ejecutando simulación de validación...")
    
    # 1. Cargar Mejores Parámetros
    with open(JSON_PATH, 'r') as f:
        data = json.load(f)
        p_vals = data["parameters"]

    # Extraer valores necesarios
    kp_bal = p_vals["KP_BALANCE"]
    kp_rec = p_vals["KP_RECOVERY"]
    rec_thresh = p_vals["RECOVERY_THRESHOLD"]
    lerp_alpha = p_vals.get("LERP_ALPHA", 0.1) # Cargar alpha, default 0.1 si falla
    
    # 2. Configurar Simulación
    if p.isConnected(): p.disconnect() # Seguridad por si acaso
    p.connect(p.DIRECT) 
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1./240.)
    p.loadURDF("plane.urdf")
    
    start_pos = [0,0,0.23]
    robotId = p.loadURDF(URDF_PATH, start_pos, [0,0,0,1])
    
    # Listas para guardar datos
    log_time = []
    log_tilt = []
    log_kp = []
    
    max_steps = int(240 * 6.0) # 6 segundos es suficiente para ver la reacción
    kick_step = int(240 * 2.0) # Patada a los 2 segundos
    
    in_recovery = False
    
    # Inicializamos current_kp con el valor base
    current_kp = kp_bal 
    
    print(f" -> Umbral de recuperación: {np.degrees(rec_thresh):.2f} grados")
    print(f" -> Kp Base: {kp_bal:.2f} | Kp Recuperación: {kp_rec:.2f}")

    for step in range(max_steps):
        t = step * (1./240.)
        
        pos, orn = p.getBasePositionAndOrientation(robotId)
        euler = p.getEulerFromQuaternion(orn)
        tilt = euler[1]
        
        # --- APLICAR PATADA MÁS FUERTE ---
        # Subimos a 80N o 100N para asegurar que cruce el umbral
        if step == kick_step:
            print(f" [!] Aplicando patada en t={t:.2f}s")
            p.applyExternalForce(robotId, -1, [90.0, 0, 0], [0, 0, 0], p.WORLD_FRAME)
            
        # --- Lógica de Máquina de Estados ---
        # Hysteresis para evitar parpadeo
        if not in_recovery and abs(tilt) > rec_thresh:
            in_recovery = True
            print(f" [!] ACTIVANDO MODO RECUPERACIÓN en t={t:.2f}s (Tilt: {np.degrees(tilt):.2f}°)")
        elif in_recovery and abs(tilt) < (rec_thresh * 0.5):
            in_recovery = False
            # No imprimimos desactivación para no saturar consola
            
        # --- PLANIFICACIÓN DE GANANCIAS (CON LERP) ---
        target_kp = kp_rec if in_recovery else kp_bal
        
        # Fórmula de Interpolación Lineal (Suavizado)
        # Esto hace que la gráfica no sea un escalón cuadrado, sino una curva suave
        current_kp = current_kp + (target_kp - current_kp) * lerp_alpha
            
        # Logging
        if step % 2 == 0: # Mayor resolución para la gráfica
            log_time.append(t)
            log_tilt.append(np.degrees(tilt))
            log_kp.append(current_kp)
            
        # Control Simple de Torque
        torque = current_kp * (0 - tilt) 
        # Añadimos un poco de D (derivativo) simple para que no oscile eternamente en la gráfica
        # Asumimos un Kd proporcional al Kp (aprox 10%) solo para la visualización
        torque += (current_kp * 0.05) * (0 - p.getBaseVelocity(robotId)[1][0]) * -1

        p.setJointMotorControl2(robotId, 0, p.VELOCITY_CONTROL, force=0)
        p.setJointMotorControl2(robotId, 0, p.TORQUE_CONTROL, force=torque)
        p.setJointMotorControl2(robotId, 1, p.TORQUE_CONTROL, force=torque)
        
        p.stepSimulation()
        
    p.disconnect()
    
    # --- GRÁFICO 3: GAIN SCHEDULING (Kp vs Time) ---
    plt.figure(figsize=(6, 3))
    plt.plot(log_time, log_kp, color='#8B0000', linewidth=2, label=r'$K_p$ Response')
    
    # Dibujar línea vertical del impacto
    plt.axvline(x=2.0, color='black', linestyle='--', alpha=0.5, label='Impact')
    
    plt.title("Gain Scheduling Activation (Adaptive Response)")
    plt.xlabel("Time (s)")
    plt.ylabel("Proportional Gain ($K_p$)")
    plt.legend()
    plt.grid(True, alpha=0.5)
    plt.tight_layout()
    plt.savefig(os.path.join(RESULTADOS_DIR, "plot_gain_scheduling.png"), dpi=300)
    print(" -> Guardado: plot_gain_scheduling.png")
    plt.close()
    
    # (Opcional) Regenerar también el de Tilt si quieres
    plt.figure(figsize=(6, 4))
    plt.plot(log_time, log_tilt, color='navy')
    plt.axvline(x=2.0, color='red', linestyle='--')
    plt.axhline(y=np.degrees(rec_thresh), color='green', linestyle=':', alpha=0.7)
    plt.axhline(y=-np.degrees(rec_thresh), color='green', linestyle=':', alpha=0.7)
    plt.title("System Response: Tilt Angle")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (deg)")
    plt.tight_layout()
    plt.savefig(os.path.join(RESULTADOS_DIR, "plot_impulse_response.png"), dpi=300)
    plt.close()
    
if __name__ == "__main__":
    plot_convergence()
    run_validation_and_plot_dynamics()
    print("\n¡Gráficos generados exitosamente en la carpeta 'resultados'!")