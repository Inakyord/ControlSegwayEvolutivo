import pybullet as p
import pybullet_data
import numpy as np
import time
import os
import json

# --- PATHS ---
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RESULTADOS_DIR = os.path.join(SCRIPT_DIR, "..", "resultados")
ASSETS_DIR = os.path.join(SCRIPT_DIR, "..", "assets")
URDF_PATH = os.path.join(ASSETS_DIR, "segwayRobot.urdf")
JSON_PATH = os.path.join(RESULTADOS_DIR, "mejores_parametros.json")

# --- LOAD PARAMETERS ---
if not os.path.exists(JSON_PATH):
    print(f"\n[ERROR] File not found: {JSON_PATH}")
    exit()

with open(JSON_PATH, "r") as f:
    data = json.load(f)
    P = data["parameters"]

# --- CONFIGURATION ---
PHYSICS_FREQ = 240.0
PHYSICS_DT = 1.0 / PHYSICS_FREQ
MAX_TORQUE = 2.5
ROBOT_SPACING = 1.5
HEADING_HOLD_KP = 40.0 # Matches Evolution

# Genes
KP_BALANCE, KD_BALANCE = P["KP_BALANCE"], P["KD_BALANCE"]
KP_FORWARD, KD_FORWARD = P["KP_FORWARD"], P["KD_FORWARD"]
KP_TURNING, KD_TURNING = P["KP_TURNING"], P["KD_TURNING"]
KP_RECOVERY, KD_RECOVERY = P["KP_RECOVERY"], P["KD_RECOVERY"]

IMPULSE_DUR = P["IMPULSE_DURATION_SEC"]
IMPULSE_MAG = P["IMPULSE_MAGNITUDE"]
TURN_SPD = P["TURNING_SPEED"]
KP_POS = P["KP_POSITION"]
TGT_TILT = P["TARGET_TILT"]
REC_THRESH = P["RECOVERY_THRESHOLD"]
LERP_ALPHA = P["LERP_ALPHA"]
KI_BALANCE = P["KI_BALANCE"]

class SegwayTester:
    def __init__(self, num_robots=3):
        self.client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(PHYSICS_DT)
        p.loadURDF("plane.urdf")
        
        self.robots = []
        start_x = -(num_robots - 1) * ROBOT_SPACING / 2
        
        for i in range(num_robots):
            pos = [start_x + i * ROBOT_SPACING, 0, 0.23]
            tilt = np.radians(np.random.uniform(-2.0, 2.0))
            orn = p.getQuaternionFromEuler([tilt, 0, 0])
            rid = p.loadURDF(URDF_PATH, pos, orn)
            
            for j in [0, 1]:
                p.changeDynamics(rid, j, lateralFriction=0.5, rollingFriction=0.02, jointDamping=0.05)
                p.setJointMotorControl2(rid, j, p.VELOCITY_CONTROL, force=0)

            self.robots.append({
                "id": rid,
                "active": True,
                "phase_idx": 0,
                "phase_seq": [0, 1, 0, 2, 0, 1, 0], 
                "next_phase_time": 6.0,
                "phase_start_time": 0.0,
                "in_recovery": False,
                "target_y": 0.0,
                "hold_yaw_target": 0.0,
                "smooth_kp": KP_BALANCE,
                "smooth_kd": KD_BALANCE,
                "integral_err": 0.0
            })

        self.global_step = 0
        self.text_id = p.addUserDebugText("INIT", [0, 0, 1], textColorRGB=[0,0,0], textSize=1.5)
        p.resetDebugVisualizerCamera(cameraDistance=3.0, cameraYaw=0, cameraPitch=-20, cameraTargetPosition=[0,0,0])

    def get_phase_name(self, idx):
        if idx == 0: return "BALANCE"
        if idx == 1: return "FORWARD"
        if idx == 2: return "TURN"
        return "UNKNOWN"

    def run(self):
        print(f"\n[INFO] Simulation Started.")
        try:
            while True:
                sim_time = self.global_step * PHYSICS_DT
                
                # Update UI
                if self.global_step % 15 == 0:
                    r0 = self.robots[0]
                    phase_name = self.get_phase_name(r0["phase_seq"][r0["phase_idx"]])
                    p.addUserDebugText(f"Mode: {phase_name} | Time: {sim_time:.1f}s", 
                                     [0, 0, 1.0], textColorRGB=[0,0,0], 
                                     textSize=1.5, replaceItemUniqueId=self.text_id)

                for r in self.robots:
                    if not r["active"]: continue
                    
                    # 1. Phase Management
                    if sim_time >= r["next_phase_time"]:
                        if r["phase_idx"] < len(r["phase_seq"]) - 1:
                            r["phase_idx"] += 1
                            r["next_phase_time"] += 6.0
                            r["phase_start_time"] = sim_time
                            
                            pos, orn = p.getBasePositionAndOrientation(r["id"])
                            cur_phase = r["phase_seq"][r["phase_idx"]]
                            
                            if cur_phase == 0: r["target_y"] = pos[1]
                            elif cur_phase == 1: r["hold_yaw_target"] = p.getEulerFromQuaternion(orn)[2]

                    # 2. Sensors
                    pos, orn = p.getBasePositionAndOrientation(r["id"])
                    euler = p.getEulerFromQuaternion(orn)
                    tilt = euler[1]
                    yaw = euler[2]
                    _, ang_vel = p.getBaseVelocity(r["id"])
                    
                    # 3. Recovery Logic
                    if not r["in_recovery"] and abs(tilt) > REC_THRESH: r["in_recovery"] = True
                    elif r["in_recovery"] and abs(tilt) < (REC_THRESH * 0.5): r["in_recovery"] = False

                    # 4. Control
                    cur_phase = r["phase_seq"][r["phase_idx"]]
                    kp, kd = KP_BALANCE, KD_BALANCE
                    des_tilt = 0.0
                    turn_val = 0.0

                    if r["in_recovery"]:
                        kp, kd = KP_RECOVERY, KD_RECOVERY
                    elif cur_phase == 0: # BALANCE
                        kp, kd = KP_BALANCE, KD_BALANCE
                        err_y = r["target_y"] - pos[1]
                        des_tilt = np.clip(KP_POS * err_y, -0.15, 0.15)
                    elif cur_phase == 1: # FORWARD
                        kp, kd = KP_FORWARD, KD_FORWARD
                        des_tilt = TGT_TILT
                        
                        yaw_err = r["hold_yaw_target"] - yaw
                        while yaw_err > np.pi: yaw_err -= 2*np.pi
                        while yaw_err < -np.pi: yaw_err += 2*np.pi
                        turn_val = np.clip(HEADING_HOLD_KP * yaw_err, -2.0, 2.0)
                        
                    elif cur_phase == 2: # TURN
                        kp, kd = KP_TURNING, KD_TURNING
                        
                        # --- RAMP UP LOGIC (MATCHES EVOLUTION) ---
                        time_in_phase = sim_time - r["phase_start_time"]
                        ramp = min(1.0, time_in_phase / 1.0)
                        turn_val = TURN_SPD * ramp

                    r["smooth_kp"] += (kp - r["smooth_kp"]) * LERP_ALPHA
                    r["smooth_kd"] += (kd - r["smooth_kd"]) * LERP_ALPHA
                    
                    tilt_err = tilt - des_tilt
                    if cur_phase == 0 and abs(tilt) < 0.1:
                        r["integral_err"] = np.clip(r["integral_err"] + tilt_err, -5.0, 5.0)
                    else:
                        r["integral_err"] = 0.0

                    out = (r["smooth_kp"] * tilt_err) + (r["smooth_kd"] * ang_vel[0]) + (KI_BALANCE * r["integral_err"])

                    # 5. Actuators
                    time_in_phase = sim_time - r["phase_start_time"]
                    if cur_phase == 1 and time_in_phase < IMPULSE_DUR and not r["in_recovery"]:
                        v_l = v_r = IMPULSE_MAG
                    else:
                        # Balance Priority
                        raw_left, raw_right = out + turn_val, out - turn_val
                        max_req = max(abs(raw_left), abs(raw_right))
                        limit = 25.0
                        if max_req > limit:
                            if abs(out) > limit:
                                out = np.clip(out, -limit, limit)
                                turn_val = 0.0
                            else:
                                turn_val = np.clip(turn_val, -(limit - abs(out)), limit - abs(out))
                        v_l = np.clip(out + turn_val, -limit, limit)
                        v_r = np.clip(out - turn_val, -limit, limit)

                    p.setJointMotorControl2(r["id"], 0, p.VELOCITY_CONTROL, targetVelocity=v_l, force=MAX_TORQUE)
                    p.setJointMotorControl2(r["id"], 1, p.VELOCITY_CONTROL, targetVelocity=v_r, force=MAX_TORQUE)
                    
                    if abs(tilt) > np.radians(60): 
                        r["active"] = False
                        p.changeVisualShape(r["id"], -1, rgbaColor=[1, 0, 0, 1])

                p.stepSimulation()
                self.global_step += 1
                time.sleep(PHYSICS_DT)

        except KeyboardInterrupt:
            p.disconnect()

if __name__ == "__main__":
    SegwayTester(num_robots=3).run()