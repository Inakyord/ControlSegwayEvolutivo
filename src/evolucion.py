import pybullet as p
import pybullet_data
import numpy as np
import os
import csv
import time
import signal
import sys
import json 
from concurrent.futures import ProcessPoolExecutor
from scipy.stats import qmc

# Configuración para evitar spam de PyBullet
os.environ['PYBULLET_DISABLE_WELCOME_MESSAGE'] = '1'

# --- RUTAS ---
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RESULTADOS_DIR = os.path.join(SCRIPT_DIR, "..", "resultados")
ASSETS_DIR = os.path.join(SCRIPT_DIR, "..", "assets")
URDF_PATH = os.path.join(ASSETS_DIR, "segwayRobot.urdf")
JSON_PATH = os.path.join(RESULTADOS_DIR, "mejores_parametros.json")

os.makedirs(RESULTADOS_DIR, exist_ok=True)

# --- HIPERPARÁMETROS ---
MAX_TORQUE = 2.5
POPULATION_SIZE = 50
NUM_GENERATIONS = 2000
MUTATION_RATE = 0.15 
TOURNAMENT_SIZE = 6
ELITISM_COUNT = 8

# --- RANGOS LINEALES (Optimizados) ---
RANGES = [
    (1, 600.0),  # 0: KP_BALANCE
    (1, 400.0),    # 1: KD_BALANCE
    (1, 600.0),   # 2: KP_FORWARD 
    (1, 400.0),    # 3: KD_FORWARD 
    (1, 600.0),   # 4: KP_TURNING 
    (1, 400.0),    # 5: KD_TURNING 
    (0.1, 1.5),     # 6: IMPULSE_DURATION_SEC
    (-5.0, 5.0),    # 7: IMPULSE_MAGNITUDE
    (1, 3.0),     # 8: TURNING_SPEED
    (100.0, 800.0), # 9: KP_RECOVERY
    (1, 400.0),  # 10: KD_RECOVERY
    (0.1, 2.0),     # 11: KP_POSITION
    (-0.20, -0.02), # 12: TARGET_TILT
    (0.02, 0.12),   # 13: RECOVERY_THRESHOLD
    (0.05, 0.60),   # 14: LERP_ALPHA
    (0.0, 5.0)      # 15: KI_BALANCE
]

GENE_NAMES = [
    "KP_BALANCE", "KD_BALANCE", "KP_FORWARD", "KD_FORWARD", 
    "KP_TURNING", "KD_TURNING", "IMPULSE_DURATION_SEC", "IMPULSE_MAGNITUDE", 
    "TURNING_SPEED", "KP_RECOVERY", "KD_RECOVERY", "KP_POSITION", "TARGET_TILT",
    "RECOVERY_THRESHOLD", "LERP_ALPHA", "KI_BALANCE"
]

# --- CONFIGURACIÓN DE TIEMPO (TIGHT LOOP) ---
PHYSICS_FREQ = 240.0
PHYSICS_DT = 1.0 / PHYSICS_FREQ

CONTROL_FREQ = 240.0  
CONTROL_DT = 1.0 / CONTROL_FREQ
PHYSICS_STEPS_PER_CONTROL = 1 # No decimation

PHASE_DURATION_SEC = 6.0
MAX_SIM_SECONDS = 36.0 
MAX_SIM_STEPS = int(PHYSICS_FREQ * MAX_SIM_SECONDS)

TILT_FAIL_ANGLE_RAD = np.radians(25.0) 

# --- GLOBALS ---
interrupted = False

def signal_handler(sig, frame):
    global interrupted
    if not interrupted:
        print("\n\n[!] Interrupción detectada. Guardando progreso...")
        interrupted = True

def init_worker():
    signal.signal(signal.SIGINT, signal.SIG_IGN)

def run_scenario(genes, scenario_type):
    # Desempaquetado
    kp_bal, kd_bal, kp_fwd, kd_fwd, kp_turn, kd_turn, \
    imp_dur_sec, imp_mag, turn_spd, \
    kp_rec, kd_rec, kp_pos, tgt_tilt, \
    rec_thresh, lerp_alpha, ki_bal = genes

    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(PHYSICS_DT)
    p.loadURDF("plane.urdf")

    start_pos = [0, 0, 0.23]
    start_tilt = 0.0
    
    # --- ESCENARIO 1: INCLINACIÓN ALEATORIA ---
    if scenario_type == "random_tilt":
        # Iniciar ligeramente inclinado (prueba de robustez inicial)
        start_tilt = np.radians(np.random.uniform(-5.0, 5.0)) 
    
    start_orn = p.getQuaternionFromEuler([start_tilt, 0, 0])

    try:
        robotId = p.loadURDF(URDF_PATH, start_pos, start_orn)
    except:
        p.disconnect()
        return -1e6, 0

    for j in [0, 1]:
        p.changeDynamics(robotId, j, lateralFriction=1.0, rollingFriction=0.02, jointDamping=0.05) 
        p.setJointMotorControl2(robotId, j, p.VELOCITY_CONTROL, force=0)

    # --- SECUENCIA DE FASES ---
    phase_seq = [0, 1, 0, 2, 0, 1] 
    phase_idx = 0
    
    current_time = 0.0
    next_phase_time = PHASE_DURATION_SEC
    
    target_y = 0.0
    in_recovery = False
    smooth_kp, smooth_kd = kp_bal, kd_bal
    integral_error = 0.0
    
    total_score = 0.0
    prev_pos = start_pos
    prev_yaw = 0.0
    phase_start_time = 0.0
    total_forward_dist = 0.0 

    v_l, v_r = 0.0, 0.0
    
    # Calcular el paso exacto del empujón (Segundo 2.0)
    kick_step = int(PHYSICS_FREQ * 2.0)

    # --- SIMULACIÓN (TIGHT LOOP) ---
    for step in range(MAX_SIM_STEPS):
        
        # 1. READ SENSORS (Every Step)
        pos, orn = p.getBasePositionAndOrientation(robotId)
        euler = p.getEulerFromQuaternion(orn)
        tilt = euler[1] 
        yaw = euler[2]
        lin_vel, ang_vel = p.getBaseVelocity(robotId)
        tilt_vel = ang_vel[0]
        
        # 2. FAIL CHECK
        if abs(tilt) > TILT_FAIL_ANGLE_RAD:
            p.disconnect()
            if total_forward_dist < 0.5: 
                return -2000.0, step
            return total_score, step

        # 3. EXTERNAL FORCE (Scenario)
        if scenario_type == "heavy_kick" and step == kick_step:
            p.applyExternalForce(robotId, -1, [40.0, 0, 0], [0, 0, 0], p.WORLD_FRAME)

        # 4. CONTROL LOGIC (EVERY STEP - NO DECIMATION)
        current_time = step * PHYSICS_DT
        
        # Phase Management
        if current_time >= next_phase_time:
            if phase_idx < len(phase_seq) - 1:
                phase_idx += 1
                next_phase_time = current_time + PHASE_DURATION_SEC
                phase_start_time = current_time
                if phase_seq[phase_idx] == 0: target_y = pos[1]

        cur_phase = phase_seq[phase_idx]

        # --- FITNESS CALCULATION ---
        dist_step = np.linalg.norm(np.array(pos[:2]) - np.array(prev_pos[:2]))
        yaw_diff = abs(yaw - prev_yaw)
        if yaw_diff > np.pi: yaw_diff = (2*np.pi) - yaw_diff
        
        if cur_phase == 0: # BALANCE
            move_penalty = dist_step * 200.0
            total_score += max(0, 0.5 - move_penalty)
            
        elif cur_phase == 1: # FORWARD
            if in_recovery:
                total_score -= 0.5 
            else:
                total_score += (dist_step * 250.0) 
                total_score -= (yaw_diff * 10.0)
                total_forward_dist += dist_step

        elif cur_phase == 2: # TURN
            target_yaw_change = turn_spd * CONTROL_DT
            spin_error = abs(yaw_diff - target_yaw_change)
            total_score += max(0, 1.0 - (spin_error * 80.0))

        prev_pos = pos
        prev_yaw = yaw

        # --- PID CONTROLLER ---
        if not in_recovery and abs(tilt) > rec_thresh: in_recovery = True
        elif in_recovery and abs(tilt) < (rec_thresh * 0.5): in_recovery = False

        des_tilt = 0.0
        turn_val = 0.0
        kp_t, kd_t = kp_bal, kd_bal

        if in_recovery:
            kp_t, kd_t = kp_rec, kd_rec
        elif cur_phase == 0: 
            kp_t, kd_t = kp_bal, kd_bal
            err_y = target_y - pos[1]
            des_tilt = np.clip(kp_pos * err_y, -0.15, 0.15)
        elif cur_phase == 1: 
            kp_t, kd_t = kp_fwd, kd_fwd
            des_tilt = tgt_tilt 
        elif cur_phase == 2: 
            kp_t, kd_t = kp_turn, kd_turn
            turn_val = turn_spd

        smooth_kp += (kp_t - smooth_kp) * lerp_alpha
        smooth_kd += (kd_t - smooth_kd) * lerp_alpha

        tilt_err = tilt - des_tilt

        if cur_phase == 0 and abs(tilt) < 0.1:
            integral_error += tilt_err
            integral_error = np.clip(integral_error, -5.0, 5.0) 
        else:
            integral_error = 0.0 
            
        out = (smooth_kp * tilt_err) + (smooth_kd * tilt_vel) + (ki_bal * integral_error)

        # --- MOTORS ---
        time_in_phase = current_time - phase_start_time
        
        if cur_phase == 1 and time_in_phase < imp_dur_sec:
            v_l = v_r = imp_mag
        else:
            if abs(out) > 30.0: total_score -= 0.1 
            v_l = np.clip(out + turn_val, -25, 25)
            v_r = np.clip(out - turn_val, -25, 25)
    
        p.setJointMotorControl2(robotId, 0, p.VELOCITY_CONTROL, targetVelocity=v_l, force=MAX_TORQUE)
        p.setJointMotorControl2(robotId, 1, p.VELOCITY_CONTROL, targetVelocity=v_r, force=MAX_TORQUE)

        # 5. STEP PHYSICS
        p.stepSimulation()

    p.disconnect()
    
    TARGET_DIST = 2.0
    
    # Si no llega a la meta, penalizamos proporcionalmente a lo que le faltó.
    if total_forward_dist < TARGET_DIST:
        missing_dist = TARGET_DIST - total_forward_dist
        total_score -= (missing_dist * 500.0)
    
    return total_score, MAX_SIM_STEPS

def evaluate_genome_robust(genes):
    scores = []
    steps_total = 0
    # AHORA SÍ probamos los 3 escenarios
    for s_type in ["standard", "random_tilt", "heavy_kick"]:
        s, st = run_scenario(genes, s_type)
        scores.append(s)
        steps_total += st
    
    min_score = min(scores)
    avg_score = sum(scores) / len(scores)
    
    # Ponderamos robustez (caso peor) y desempeño promedio
    final_fitness = (min_score * 0.2) + (avg_score * 0.8)
    avg_steps = steps_total / 3.0
    return final_fitness, avg_steps

def save_best_params_json(best_genes, fitness):
    data = { "metadata": { "fitness": float(fitness), "timestamp": time.strftime("%Y-%m-%d %H:%M:%S") }, "parameters": {} }
    for name, value in zip(GENE_NAMES, best_genes):
        data["parameters"][name] = float(value)
    with open(JSON_PATH, "w") as f:
        json.dump(data, f, indent=4)
    print(f"    -> Parámetros guardados en: {JSON_PATH}")

def save_top_individuals(population, fitnesses, filename="top_10_individuos.csv"):
    pop_with_fit = list(zip(fitnesses, population))
    pop_with_fit.sort(key=lambda x: x[0], reverse=True)
    filepath = os.path.join(RESULTADOS_DIR, filename)
    with open(filepath, "w", newline="") as f:
        writer = csv.writer(f)
        header = ["Rank", "Fitness"] + GENE_NAMES
        writer.writerow(header)
        for i in range(min(10, len(pop_with_fit))):
            fit, genes = pop_with_fit[i]
            row = [i+1, f"{fit:.2f}"] + [f"{val:.4f}" for val in genes]
            writer.writerow(row)

def initialize_population(size):
    sampler = qmc.LatinHypercube(d=len(RANGES))
    sample = sampler.random(n=size)
    return [tuple(r[0] + s[i]*(r[1]-r[0]) for i, r in enumerate(RANGES)) for s in sample]

def tournament_selection(population, scores):
    idx = np.random.choice(len(population), size=TOURNAMENT_SIZE, replace=False)
    return population[idx[np.argmax([scores[i] for i in idx])]]

def run_evolution():
    global interrupted
    log_history_path = os.path.join(RESULTADOS_DIR, "historial_fitness.csv")
    log_best_path = os.path.join(RESULTADOS_DIR, "historial_mejor_individuo.csv")
    
    with open(log_history_path, "w", newline="") as f:
        csv.writer(f).writerow(["Gen", "Best_Fit", "Avg_Fit", "Std_Dev_Fit"])
    with open(log_best_path, "w", newline="") as f:
        csv.writer(f).writerow(["Gen", "Fitness"] + GENE_NAMES)

    population = initialize_population(POPULATION_SIZE)
    best_gene_ever = None
    best_fit_ever = -1e9
    
    print(f"--- INICIANDO EVOLUCIÓN (TIGHT LOOP - 240Hz) ---")
    
    signal.signal(signal.SIGINT, signal_handler)
    start_time = time.time()

    for gen in range(NUM_GENERATIONS):
        if interrupted: break
        
        with ProcessPoolExecutor(max_workers=os.cpu_count(), initializer=init_worker) as ex:
            results = list(ex.map(evaluate_genome_robust, population))
        
        fitnesses = [r[0] for r in results]
        curr_best_fit = np.max(fitnesses)
        curr_best_idx = np.argmax(fitnesses)
        
        if curr_best_fit > best_fit_ever:
            best_fit_ever = curr_best_fit
            best_gene_ever = population[curr_best_idx]
            
        with open(log_history_path, "a", newline="") as f:
            csv.writer(f).writerow([gen, curr_best_fit, np.mean(fitnesses), np.std(fitnesses)])
        with open(log_best_path, "a", newline="") as f:
            row = [gen, curr_best_fit] + list(population[curr_best_idx])
            csv.writer(f).writerow(row)
            
        elapsed = time.time() - start_time
        print(f"Gen {gen:03d} | Best: {curr_best_fit:7.1f} | Avg: {np.mean(fitnesses):7.1f} | T: {elapsed:.0f}s")
        
        sorted_idx = np.argsort(fitnesses)[::-1]
        next_pop = [population[i] for i in sorted_idx[:ELITISM_COUNT]]
        
        while len(next_pop) < POPULATION_SIZE:
            p1 = tournament_selection(population, fitnesses)
            p2 = tournament_selection(population, fitnesses)
            child = []
            BLX_ALPHA = 0.5 

            for i in range(len(RANGES)):
                gene_min = min(p1[i], p2[i])
                gene_max = max(p1[i], p2[i])
                diff = gene_max - gene_min
                lower_bound = gene_min - (diff * BLX_ALPHA)
                upper_bound = gene_max + (diff * BLX_ALPHA)
                gene = np.random.uniform(lower_bound, upper_bound)
                if np.random.rand() < MUTATION_RATE:
                    scale = (RANGES[i][1] - RANGES[i][0]) * 0.10 
                    gene += np.random.normal(0, scale) 
                child.append(np.clip(gene, RANGES[i][0], RANGES[i][1]))
            
            next_pop.append(tuple(child))
        population = next_pop

    print("\n--- FINALIZADO ---")
    if best_gene_ever is not None:
        save_top_individuals(population, fitnesses)
        save_best_params_json(best_gene_ever, best_fit_ever)
        print("Mejor Fitness:", best_fit_ever)

if __name__ == "__main__":
    run_evolution()