import pybullet as p
import pybullet_data
import numpy as np
import time
import os
import json
import sys

# --- RUTAS ---
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RESULTADOS_DIR = os.path.join(SCRIPT_DIR, "..", "resultados")
ASSETS_DIR = os.path.join(SCRIPT_DIR, "..", "assets")
URDF_PATH = os.path.join(ASSETS_DIR, "segwayRobot.urdf")
JSON_PATH = os.path.join(RESULTADOS_DIR, "mejores_parametros.json")

def load_parameters():
    if not os.path.exists(JSON_PATH):
        print(f"\n[ERROR] No se encontró: {JSON_PATH}. Ejecuta 'evolucion.py' primero.")
        sys.exit(1)
    
    try:
        with open(JSON_PATH, "r") as f:
            data = json.load(f)
        return data["parameters"]
    except Exception as e:
        print(f"\n[ERROR] JSON corrupto: {e}")
        sys.exit(1)

# --- CARGA DE PARÁMETROS ---
P = load_parameters()

PHYSICS_FREQ = 240.0
CONTROL_FREQ = 240.0  
PHYSICS_DT = 1.0 / PHYSICS_FREQ
STEPS_PER_CONTROL = 1 

# Variables de Control
KP_BALANCE, KD_BALANCE = P["KP_BALANCE"], P["KD_BALANCE"]
KP_FORWARD, KD_FORWARD = P["KP_FORWARD"], P["KD_FORWARD"]
KP_TURNING, KD_TURNING = P["KP_TURNING"], P["KD_TURNING"]
KP_RECOVERY, KD_RECOVERY = P["KP_RECOVERY"], P["KD_RECOVERY"]

# Comportamiento
IMPULSE_STEPS = int(P["IMPULSE_DURATION_SEC"] * CONTROL_FREQ) 
IMPULSE_MAG = P["IMPULSE_MAGNITUDE"]
TURN_SPD = P["TURNING_SPEED"]
KP_POS = P["KP_POSITION"]
TGT_TILT = P["TARGET_TILT"]

# Genes de "Inteligencia"
REC_THRESH = P["RECOVERY_THRESHOLD"]
LERP_ALPHA = P["LERP_ALPHA"]
KI_BALANCE = P["KI_BALANCE"]

# Escenario
MAX_TORQUE = 2.5
NUM_ROBOTS = 3
ROBOT_SPACING = 1.5
PHASE_DURATION_SEC = 6.0
PHASE_STEPS_PHYSICS = int(PHYSICS_FREQ * PHASE_DURATION_SEC)

class SegwayTester:
    def __init__(self, num_robots=1):
        # GUI Mode para ver la simulación
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(PHYSICS_DT)

        p.loadURDF("plane.urdf")
        self.robots = []
        
        start_x = -(num_robots - 1) * ROBOT_SPACING / 2
        
        for i in range(num_robots):
            pos = [start_x + i * ROBOT_SPACING, 0, 0.23]
            # Pequeña variación inicial aleatoria
            tilt = np.radians(np.random.uniform(-3.0, 3.0))
            orn = p.getQuaternionFromEuler([tilt, 0, 0])
            
            rid = p.loadURDF(URDF_PATH, pos, orn)
            
            # Fricción idéntica al entrenamiento
            for j in [0, 1]:
                p.changeDynamics(rid, j, lateralFriction=1.0, rollingFriction=0.02, jointDamping=0.05)
                p.setJointMotorControl2(rid, j, p.VELOCITY_CONTROL, force=0)

            self.robots.append({
                "id": rid,
                "active": True,
                "phase_idx": 0,
                "phase_seq": [0, 1, 0, 2, 0, 1], 
                "next_phase_time": PHASE_DURATION_SEC,
                "phase_start_time": 0.0,
                "in_recovery": False,
                "target_y": 0.0,
                "smooth_kp": KP_BALANCE,
                "smooth_kd": KD_BALANCE,
                "integral_err": 0.0
            })

        self.global_step = 0
        self.sim_time = 0.0
        
        # Debug Text ID
        self.text_id = p.addUserDebugText("INIT", [0, 0, 1], textColorRGB=[0,0,0], textSize=1.5)

        print(f"\n[INFO] TIGHT LOOP ACTIVATED: 240Hz Physics & Control")
        print(f"[INFO] Torque Limit: {MAX_TORQUE}")

    def get_phase_name(self, idx):
        if idx == 0: return "BALANCE"
        if idx == 1: return "FORWARD"
        if idx == 2: return "TURN"
        return "UNKNOWN"

    def run(self):
        print("Starting Simulation (Ctrl+C to stop)")
        try:
            while True:
                # 1. ACTUALIZAR FASES
                self.sim_time = self.global_step * PHYSICS_DT
                
                # Debug Text (Update rarely to save performance, e.g. every 10 frames)
                if self.global_step % 10 == 0:
                    r0 = self.robots[0]
                    phase_name = self.get_phase_name(r0["phase_seq"][r0["phase_idx"]])
                    p.addUserDebugText(f"{phase_name} ({self.sim_time:.1f}s)", 
                                     [0, 0, 1.0], textColorRGB=[0,0,0], 
                                     textSize=1.5, replaceItemUniqueId=self.text_id)

                # --- BUCLE DE CONTROL (AHORA 240Hz - TIGHT LOOP) ---
                # REMOVED: "if self.global_step % STEPS_PER_CONTROL == 0:"
                # The logic now runs EVERY step, just like Script 1.
                
                for r in self.robots:
                    if not r["active"]: continue
                    
                    # Chequear tiempo de fase
                    if self.sim_time >= r["next_phase_time"]:
                        if r["phase_idx"] < len(r["phase_seq"]) - 1:
                            r["phase_idx"] += 1
                            r["next_phase_time"] += PHASE_DURATION_SEC
                            r["phase_start_time"] = self.sim_time
                            pos, _ = p.getBasePositionAndOrientation(r["id"])
                            if r["phase_seq"][r["phase_idx"]] == 0: 
                                r["target_y"] = pos[1]

                    # Sensores
                    pos, orn = p.getBasePositionAndOrientation(r["id"])
                    euler = p.getEulerFromQuaternion(orn)
                    tilt = euler[1]
                    lin_vel, ang_vel = p.getBaseVelocity(r["id"])
                    tilt_vel = ang_vel[0]

                    # --- LÓGICA DEL CEREBRO ---
                    
                    # 1. Recuperación de Pánico
                    if not r["in_recovery"] and abs(tilt) > REC_THRESH:
                        r["in_recovery"] = True
                    elif r["in_recovery"] and abs(tilt) < (REC_THRESH * 0.5):
                        r["in_recovery"] = False

                    # 2. Configurar Ganancias
                    p_id = r["phase_seq"][r["phase_idx"]]
                    des_tilt = 0.0
                    turn_val = 0.0
                    kp, kd = KP_BALANCE, KD_BALANCE

                    if r["in_recovery"]:
                        kp, kd = KP_RECOVERY, KD_RECOVERY
                    elif p_id == 0: # BALANCE
                        kp, kd = KP_BALANCE, KD_BALANCE
                        err_y = r["target_y"] - pos[1]
                        des_tilt = np.clip(KP_POS * err_y, -0.15, 0.15)
                    elif p_id == 1: # FORWARD
                        kp, kd = KP_FORWARD, KD_FORWARD
                        des_tilt = TGT_TILT
                    elif p_id == 2: # TURN
                        kp, kd = KP_TURNING, KD_TURNING
                        turn_val = TURN_SPD

                    # 3. Suavizado (LERP)
                    r["smooth_kp"] += (kp - r["smooth_kp"]) * LERP_ALPHA
                    r["smooth_kd"] += (kd - r["smooth_kd"]) * LERP_ALPHA

                    # 4. PID + Integral
                    tilt_err = tilt - des_tilt
                    
                    if p_id == 0 and abs(tilt) < 0.1:
                        r["integral_err"] += tilt_err
                        r["integral_err"] = np.clip(r["integral_err"], -5.0, 5.0)
                    else:
                        r["integral_err"] = 0.0

                    out = (r["smooth_kp"] * tilt_err) + (r["smooth_kd"] * tilt_vel) + (KI_BALANCE * r["integral_err"])

                    # 5. Motores
                    v_l, v_r = 0.0, 0.0
                    time_in_phase = self.sim_time - r["phase_start_time"]

                    if p_id == 1 and time_in_phase < P["IMPULSE_DURATION_SEC"]:
                        v_l = v_r = IMPULSE_MAG
                    else:
                        v_l = np.clip(out + turn_val, -25, 25)
                        v_r = np.clip(out - turn_val, -25, 25)

                    p.setJointMotorControl2(r["id"], 0, p.VELOCITY_CONTROL, targetVelocity=v_l, force=MAX_TORQUE)
                    p.setJointMotorControl2(r["id"], 1, p.VELOCITY_CONTROL, targetVelocity=v_r, force=MAX_TORQUE)
                    
                    if abs(tilt) > np.radians(45): r["active"] = False

                # --- SIMULACIÓN FÍSICA ---
                p.stepSimulation()
                self.global_step += 1
                
                time.sleep(PHYSICS_DT / 10)

        except KeyboardInterrupt:
            print("\nStopped.")

if __name__ == "__main__":
    SegwayTester(num_robots=3).run()