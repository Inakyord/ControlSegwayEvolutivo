import pybullet as p
import pybullet_data
import numpy as np
import os
import csv
import time
import signal
import json 
from concurrent.futures import ProcessPoolExecutor
from scipy.stats import qmc

# Environment Setup
os.environ['PYBULLET_DISABLE_WELCOME_MESSAGE'] = '1'

# --- PATHS ---
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RESULTADOS_DIR = os.path.join(SCRIPT_DIR, "..", "resultados")
ASSETS_DIR = os.path.join(SCRIPT_DIR, "..", "assets")
URDF_PATH = os.path.join(ASSETS_DIR, "segwayRobot.urdf")
JSON_PATH = os.path.join(RESULTADOS_DIR, "mejores_parametros.json")

os.makedirs(RESULTADOS_DIR, exist_ok=True)

# --- HYPERPARAMETERS ---
MAX_TORQUE = 2.5
POPULATION_SIZE = 60
NUM_GENERATIONS = 2000
MUTATION_RATE = 0.10
TOURNAMENT_SIZE = 6
ELITISM_COUNT = 6

# --- GENE RANGES ---
RANGES = [
    (1, 800.0),      # 0: KP_BALANCE
    (1, 600.0),      # 1: KD_BALANCE
    (1, 800.0),      # 2: KP_FORWARD 
    (1, 600.0),      # 3: KD_FORWARD 
    (1, 800.0),      # 4: KP_TURNING
    (1, 600.0),      # 5: KD_TURNING
    (0.05, 0.4),     # 6: IMPULSE_DURATION_SEC
    (-25.0, -5.0),   # 7: IMPULSE_MAGNITUDE
    (2, 15.0),       # 8: TURNING_SPEED
    (1, 800.0),      # 9: KP_RECOVERY
    (1, 600.0),      # 10: KD_RECOVERY
    (0.1, 2.0),      # 11: KP_POSITION
    (0.02, 0.25),    # 12: TARGET_TILT
    (0.02, 0.12),    # 13: RECOVERY_THRESHOLD
    (0.05, 0.70),    # 14: LERP_ALPHA
    (0.0, 5.0)       # 15: KI_BALANCE
]

GENE_NAMES = [
    "KP_BALANCE", "KD_BALANCE", "KP_FORWARD", "KD_FORWARD", 
    "KP_TURNING", "KD_TURNING", "IMPULSE_DURATION_SEC", "IMPULSE_MAGNITUDE", 
    "TURNING_SPEED", "KP_RECOVERY", "KD_RECOVERY", "KP_POSITION", "TARGET_TILT",
    "RECOVERY_THRESHOLD", "LERP_ALPHA", "KI_BALANCE"
]

# --- TIME CONFIG ---
PHYSICS_FREQ = 240.0
PHYSICS_DT = 1.0 / PHYSICS_FREQ
PHASE_DURATION_SEC = 7.0
MAX_SIM_SECONDS = 63
MAX_SIM_STEPS = int(PHYSICS_FREQ * MAX_SIM_SECONDS)

# --- PENALTIES ---
TILT_FAIL_ANGLE_RAD = np.radians(46.0) 
DEATH_PENALTY = -5000.0 
PENALTY_FACTOR = 3000.0 

# --- GLOBALS ---
interrupted = False

def signal_handler(sig, frame):
    global interrupted
    if not interrupted:
        print("\n\n[!] Interruption detected. Saving progress...")
        interrupted = True

def init_worker():
    signal.signal(signal.SIGINT, signal.SIG_IGN)

def run_scenario(genes, scenario_type):
    # Unpack Genes
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
    if scenario_type == "random_tilt":
        start_tilt = np.radians(np.random.uniform(-10.0, 10.0))
    
    start_orn = p.getQuaternionFromEuler([start_tilt, 0, 0])

    try:
        robotId = p.loadURDF(URDF_PATH, start_pos, start_orn)
    except:
        p.disconnect()
        return -1e6, 0, 0 

    # Low Friction Setup
    for j in [0, 1]:
        p.changeDynamics(robotId, j, lateralFriction=0.5, rollingFriction=0.02, jointDamping=0.05) 
        p.setJointMotorControl2(robotId, j, p.VELOCITY_CONTROL, force=0)

    # State Machine
    REC_STATE_NORMAL = 0
    REC_STATE_PANIC = 1
    REC_STATE_SETTLING = 2
    recovery_state = REC_STATE_NORMAL
    settling_start_time = 0.0
    SETTLING_DURATION = 2.0          
    SETTLING_ANGLE_RAD = np.radians(5.0)

    # Phases
    phase_seq = [0, 1, 0, 2, 0, 1, 0, 2, 0] 
    phase_idx = 0
    max_phase_reached = 0
    
    current_time = 0.0
    next_phase_time = PHASE_DURATION_SEC
    
    target_y = 0.0
    hold_yaw_target = 0.0 
    HEADING_HOLD_KP = 40.0 
    
    smooth_kp, smooth_kd = kp_bal, kd_bal
    integral_error = 0.0
    total_score = 0.0
    prev_pos = start_pos
    prev_yaw = 0.0
    phase_start_time = 0.0
    total_forward_dist = 0.0 

    phase_accum_dist = 0.0
    phase_accum_yaw = 0.0
    
    MIN_DIST_PER_PHASE = 1.0
    MIN_YAW_PER_PHASE = 0.2

    kick_step = int(PHYSICS_FREQ * 2.0)

    # --- SIMULATION LOOP ---
    for step in range(MAX_SIM_STEPS):
        
        if phase_idx > max_phase_reached:
            max_phase_reached = phase_idx

        # 1. Sensors
        pos, orn = p.getBasePositionAndOrientation(robotId)
        euler = p.getEulerFromQuaternion(orn)
        tilt = euler[1] 
        yaw = euler[2]
        lin_vel, ang_vel = p.getBaseVelocity(robotId)
        tilt_vel = ang_vel[0]
        
        dist_step = np.linalg.norm(np.array(pos[:2]) - np.array(prev_pos[:2]))
        yaw_diff = abs(yaw - prev_yaw)
        if yaw_diff > np.pi: yaw_diff = (2*np.pi) - yaw_diff
        
        phase_accum_dist += dist_step
        phase_accum_yaw += yaw_diff
        
        # 2. Death Check (FALLING)
        if abs(tilt) > TILT_FAIL_ANGLE_RAD:
            p.disconnect()
            
            # --- STRATEGY: FAIL FORWARD ---
            # Trying and falling (-1500) is better than standing still (-3000).
            # This logic applies to BOTH Phase 1 (Forward) and Phase 2 (Turn).
            if phase_seq[phase_idx] in [1, 2]:
                actual_penalty = -1500.0
            else:
                actual_penalty = DEATH_PENALTY 
                
            return actual_penalty, step, max_phase_reached

        # 3. Impact
        if scenario_type == "heavy_kick" and step == kick_step:
            p.applyExternalForce(robotId, -1, [500, 0, 0], [0, 0, 0], p.WORLD_FRAME)

        # 4. Phase Management
        current_time = step * PHYSICS_DT
        
        if current_time >= next_phase_time:
            finishing_phase = phase_seq[phase_idx]
            
            # --- PERFORMANCE DEATH (Hard Kill) ---
            # If they don't meet the goal, they DIE (-3000 approx penalty).
            if finishing_phase == 1: # Forward
                if phase_accum_dist < MIN_DIST_PER_PHASE:
                    penalty = (MIN_DIST_PER_PHASE - phase_accum_dist) * PENALTY_FACTOR
                    total_score -= penalty
                    p.disconnect() 
                    return total_score, step, max_phase_reached
                
            elif finishing_phase == 2: # Turn
                if phase_accum_yaw < MIN_YAW_PER_PHASE:
                    penalty = (MIN_YAW_PER_PHASE - phase_accum_yaw) * PENALTY_FACTOR
                    total_score -= penalty
                    p.disconnect()
                    return total_score, step, max_phase_reached

            if phase_idx < len(phase_seq) - 1:
                phase_idx += 1
                next_phase_time = current_time + PHASE_DURATION_SEC
                phase_start_time = current_time
                
                if phase_seq[phase_idx] == 0: target_y = pos[1] 
                if phase_seq[phase_idx] == 1: hold_yaw_target = yaw 

                phase_accum_dist = 0.0
                phase_accum_yaw = 0.0

        cur_phase = phase_seq[phase_idx]

        # 5. Fitness
        if recovery_state != REC_STATE_NORMAL:
             total_score -= 0.1 

        if cur_phase == 0: # BALANCE
            total_score -= (abs(tilt) * 1.0) 
            total_score -= (dist_step * 100.0) 
            
        elif cur_phase == 1: # FORWARD
            if recovery_state == REC_STATE_NORMAL:
                vel_forward = lin_vel[0] 
                if vel_forward > 0:
                    total_score += (vel_forward * 0.2) 
                
                total_score -= (yaw_diff * 10.0) 
                total_forward_dist += dist_step
                
                # Bonus for holding the aggressive lean (Encourages trying)
                tilt_error = abs(tilt - tgt_tilt)
                if tilt_error < 0.05: total_score += 0.5

        elif cur_phase == 2: # TURN
            target_yaw_step = turn_spd * PHYSICS_DT
            error_yaw = abs(yaw_diff - target_yaw_step)
            
            if error_yaw < 0.005: 
                total_score += 0.5 
            else:
                total_score -= (error_yaw * 10.0)
            
            # --- NEW: SPIN BONUS ---
            # Just like "Lean Bonus" for forward, this rewards high yaw velocity.
            # It encourages "trying to spin" even if they haven't mastered stability yet.
            current_turn_rate = abs(ang_vel[2]) # Z-axis rotation
            if current_turn_rate > 1.0: # > 1 rad/s
                total_score += 0.5

        prev_pos = pos
        prev_yaw = yaw

        # 6. Control Logic
        if recovery_state == REC_STATE_NORMAL:
            if abs(tilt) > rec_thresh: recovery_state = REC_STATE_PANIC
        elif recovery_state == REC_STATE_PANIC:
            if abs(tilt) < SETTLING_ANGLE_RAD:
                recovery_state = REC_STATE_SETTLING
                settling_start_time = current_time
        elif recovery_state == REC_STATE_SETTLING:
            if abs(tilt) > rec_thresh: recovery_state = REC_STATE_PANIC
            elif (current_time - settling_start_time) > SETTLING_DURATION:
                recovery_state = REC_STATE_NORMAL
                if cur_phase == 0: target_y = pos[1]

        des_tilt = 0.0
        turn_val = 0.0
        kp_t, kd_t = kp_bal, kd_bal
        use_integral = False

        if recovery_state == REC_STATE_PANIC:
            kp_t, kd_t = kp_rec, kd_rec
        elif recovery_state == REC_STATE_SETTLING:
            kp_t, kd_t = kp_bal, kd_bal
            err_y = target_y - pos[1] 
            des_tilt = 0.0
        else: # NORMAL
            if cur_phase == 0: # BALANCE
                kp_t, kd_t = kp_bal, kd_bal
                err_y = target_y - pos[1]
                des_tilt = np.clip(kp_pos * err_y, -0.15, 0.15)
                use_integral = True
                
            elif cur_phase == 1: # FORWARD
                kp_t, kd_t = kp_fwd, kd_fwd
                des_tilt = tgt_tilt
                
                yaw_err = hold_yaw_target - yaw
                while yaw_err > np.pi: yaw_err -= 2*np.pi
                while yaw_err < -np.pi: yaw_err += 2*np.pi
                turn_val = np.clip(HEADING_HOLD_KP * yaw_err, -2.0, 2.0)
                
            elif cur_phase == 2: # TURN
                kp_t, kd_t = kp_turn, kd_turn
                
                # --- RAMP UP LOGIC ---
                # Helps physics stability, preventing instant falls
                time_in_phase = current_time - phase_start_time
                ramp = min(1.0, time_in_phase / 1.0)
                turn_val = turn_spd * ramp

        smooth_kp += (kp_t - smooth_kp) * lerp_alpha
        smooth_kd += (kd_t - smooth_kd) * lerp_alpha
        tilt_err = tilt - des_tilt

        if use_integral and abs(tilt) < 0.1:
            integral_error += tilt_err
            integral_error = np.clip(integral_error, -5.0, 5.0) 
        else:
            integral_error = 0.0 
            
        out = (smooth_kp * tilt_err) + (smooth_kd * tilt_vel) + (ki_bal * integral_error)

        # 7. Actuators (Priority Logic)
        raw_left = out + turn_val
        raw_right = out - turn_val

        max_req = max(abs(raw_left), abs(raw_right))
        limit = 25.0

        if max_req > limit:
            if abs(out) > limit:
                out = np.clip(out, -limit, limit)
                turn_val = 0.0 
            else:
                headroom = limit - abs(out)
                turn_val = np.clip(turn_val, -headroom, headroom)

            v_l = np.clip(out + turn_val, -limit, limit)
            v_r = np.clip(out - turn_val, -limit, limit)
        else:
            v_l = raw_left
            v_r = raw_right

        p.setJointMotorControl2(robotId, 0, p.VELOCITY_CONTROL, targetVelocity=v_l, force=MAX_TORQUE)
        p.setJointMotorControl2(robotId, 1, p.VELOCITY_CONTROL, targetVelocity=v_r, force=MAX_TORQUE)

        p.stepSimulation()

    p.disconnect()
    
    if total_forward_dist < 2.0:
        missing_dist = 2.0 - total_forward_dist
        total_score -= (missing_dist * 500.0)
    
    return total_score, MAX_SIM_STEPS, max_phase_reached

def evaluate_genome_robust(genes):
    scores = []
    steps_total = 0
    phase_acc = 0
    
    for s_type in ["standard", "random_tilt", "heavy_kick"]:
        s, st, ph = run_scenario(genes, s_type)
        scores.append(s)
        steps_total += st
        phase_acc += ph
    
    min_score = min(scores)
    avg_score = sum(scores) / len(scores)
    
    final_fitness = (min_score * 0.4) + (avg_score * 0.6)
    avg_phase = phase_acc / 3.0
    
    return final_fitness, avg_phase

def save_best_params_json(best_genes, fitness):
    data = { "metadata": { "fitness": float(fitness), "timestamp": time.strftime("%Y-%m-%d %H:%M:%S") }, "parameters": {} }
    for name, value in zip(GENE_NAMES, best_genes):
        data["parameters"][name] = float(value)
    with open(JSON_PATH, "w") as f:
        json.dump(data, f, indent=4)
    print(f"    -> Params saved to: {JSON_PATH}")

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
    
    print(f"--- STARTING EVOLUTION (240Hz) ---")
    
    signal.signal(signal.SIGINT, signal_handler)
    start_time = time.time()

    for gen in range(NUM_GENERATIONS):
        if interrupted: break
        
        with ProcessPoolExecutor(max_workers=os.cpu_count(), initializer=init_worker) as ex:
            results = list(ex.map(evaluate_genome_robust, population))
        
        fitnesses = [r[0] for r in results]
        phases = [r[1] for r in results]
        
        curr_best_fit = np.max(fitnesses)
        curr_best_idx = np.argmax(fitnesses)
        best_phase_int = int(phases[curr_best_idx])
        
        if curr_best_fit > best_fit_ever:
            best_fit_ever = curr_best_fit
            best_gene_ever = population[curr_best_idx]
            
        with open(log_history_path, "a", newline="") as f:
            csv.writer(f).writerow([gen, curr_best_fit, np.mean(fitnesses), np.std(fitnesses)])
        with open(log_best_path, "a", newline="") as f:
            row = [gen, curr_best_fit] + list(population[curr_best_idx])
            csv.writer(f).writerow(row)
            
        elapsed = time.time() - start_time
        print(f"Gen {gen:03d} | Best: {curr_best_fit:7.1f} | Phase: {best_phase_int:01d} | Avg: {np.mean(fitnesses):7.1f} | T: {elapsed:.0f}s")
        
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

    print("\n--- FINISHED ---")
    if best_gene_ever is not None:
        save_top_individuals(population, fitnesses)
        save_best_params_json(best_gene_ever, best_fit_ever)
        print("Best Fitness:", best_fit_ever)

if __name__ == "__main__":
    run_evolution()