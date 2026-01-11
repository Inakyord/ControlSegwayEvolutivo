import pybullet as p
import pybullet_data
import numpy as np
import time
import os
import json
import sys

# --- PATHS ---
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ASSETS_DIR = os.path.join(SCRIPT_DIR, "..", "assets")
RESULTADOS_DIR = os.path.join(SCRIPT_DIR, "..", "resultados")
URDF_PATH = os.path.join(ASSETS_DIR, "segwayRobot.urdf")
JSON_PATH = os.path.join(RESULTADOS_DIR, "mejores_parametros.json")

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
HEADING_HOLD_KP = 40.0 

# Genes
KP_BAL, KD_BAL = P["KP_BALANCE"], P["KD_BALANCE"]
KP_FWD, KD_FWD = P["KP_FORWARD"], P["KD_FORWARD"]
KP_TRN, KD_TRN = P["KP_TURNING"], P["KD_TURNING"]
KP_REC, KD_REC = P["KP_RECOVERY"], P["KD_RECOVERY"]

IMPULSE_STEPS = int(P["IMPULSE_DURATION_SEC"] * PHYSICS_FREQ)
IMPULSE_MAG = P["IMPULSE_MAGNITUDE"]
KP_POS = P["KP_POSITION"]
TGT_TILT = P["TARGET_TILT"]
TURN_SPD = P["TURNING_SPEED"]
REC_THRESH = P["RECOVERY_THRESHOLD"]
LERP_ALPHA = P["LERP_ALPHA"]
KI_BAL = P["KI_BALANCE"]

class SegwayManual:
    def __init__(self):
        self.client = p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(PHYSICS_DT)
        
        p.loadURDF("plane.urdf")
        self.robotId = p.loadURDF(URDF_PATH, [0, 0, 0.23], p.getQuaternionFromEuler([0, 0, 0]))
        
        for j in [0, 1]:
            p.changeDynamics(self.robotId, j, lateralFriction=0.5, rollingFriction=0.02, jointDamping=0.05)
            p.setJointMotorControl2(self.robotId, j, p.VELOCITY_CONTROL, force=0)

        self.mode = "BALANCE" 
        self.running = True
        self.in_recovery = False
        self.manual_turn_dir = 0 
        
        # State
        self.target_y = 0.0
        self.hold_yaw = 0.0
        self.impulse_counter = 0 
        self.smooth_kp = KP_BAL
        self.smooth_kd = KD_BAL
        self.integral_err = 0.0
        
        # Ramp logic for Manual Turn
        self.current_turn_val = 0.0
        
        self.debug_text_id = -1
        self.global_step = 0
        self.print_instructions()

    def print_instructions(self):
        print("\n" + "="*50)
        print(" MANUAL CONTROL (Synced with Evolution)")
        print("="*50)
        print(" [W] FORWARD (Impulse + Lean + Heading Hold)")
        print(" [S] STOP (Balance)")
        print(" [A] LEFT (Ramped Turn)")
        print(" [D] RIGHT (Ramped Turn)")
        print(" [Q] QUIT")
        print("="*50 + "\n")

    def draw_debug_text(self):
        if self.global_step % 10 != 0: return
        pos, _ = p.getBasePositionAndOrientation(self.robotId)
        msg = f"MODE: {self.mode}"
        if self.in_recovery: msg = "!!! RECOVERY !!!"
        self.debug_text_id = p.addUserDebugText(msg, [pos[0], pos[1], pos[2] + 0.5], [0,0,0], textSize=1.2, replaceItemUniqueId=self.debug_text_id)

    def process_input(self):
        keys = p.getKeyboardEvents()
        if ord('q') in keys and keys[ord('q')] & p.KEY_WAS_TRIGGERED: self.running = False

        if ord('w') in keys and (keys[ord('w')] & p.KEY_WAS_TRIGGERED):
            self.mode = "FORWARD"
            self.impulse_counter = 0
            _, orn = p.getBasePositionAndOrientation(self.robotId)
            self.hold_yaw = p.getEulerFromQuaternion(orn)[2]

        if ord('s') in keys and (keys[ord('s')] & p.KEY_WAS_TRIGGERED):
            self.mode = "BALANCE"
            self.target_y = p.getBasePositionAndOrientation(self.robotId)[0][1]
            self.manual_turn_dir = 0

        self.manual_turn_dir = 0
        if ord('a') in keys and (keys[ord('a')] & p.KEY_IS_DOWN):
            self.manual_turn_dir = 1
            self.mode = "TURN"
        elif ord('d') in keys and (keys[ord('d')] & p.KEY_IS_DOWN):
            self.manual_turn_dir = -1
            self.mode = "TURN"
        elif self.mode == "TURN": 
            self.mode = "BALANCE"
            self.target_y = p.getBasePositionAndOrientation(self.robotId)[0][1]

    def update_control_logic(self):
        pos, orn = p.getBasePositionAndOrientation(self.robotId)
        euler = p.getEulerFromQuaternion(orn)
        tilt = euler[1]
        yaw = euler[2]
        _, ang_vel = p.getBaseVelocity(self.robotId)
        
        if not self.in_recovery and abs(tilt) > REC_THRESH: self.in_recovery = True
        elif self.in_recovery and abs(tilt) < (REC_THRESH * 0.5): 
            self.in_recovery = False
            self.target_y = pos[1] 
            self.hold_yaw = yaw

        kp, kd = KP_BAL, KD_BAL
        des_tilt = 0.0
        target_turn = 0.0
        
        if self.in_recovery:
            kp, kd = KP_REC, KD_REC
        elif self.mode == "FORWARD":
            kp, kd = KP_FWD, KD_FWD
            des_tilt = TGT_TILT
            yaw_err = self.hold_yaw - yaw
            while yaw_err > np.pi: yaw_err -= 2*np.pi
            while yaw_err < -np.pi: yaw_err += 2*np.pi
            target_turn = np.clip(HEADING_HOLD_KP * yaw_err, -2.0, 2.0)
            
        elif self.mode == "TURN":
            kp, kd = KP_TRN, KD_TRN
            target_turn = TURN_SPD * self.manual_turn_dir
            
        else: # BALANCE
            kp, kd = KP_BAL, KD_BAL
            err_y = self.target_y - pos[1]
            des_tilt = np.clip(KP_POS * err_y, -0.15, 0.15)

        # --- SLEW RATE LIMITER (RAMP) ---
        # Slowly move current_turn_val towards target_turn
        # Rise time approx 1.0s (same as evolution)
        step_size = TURN_SPD * (PHYSICS_DT / 1.0) 
        if self.current_turn_val < target_turn:
            self.current_turn_val = min(target_turn, self.current_turn_val + step_size)
        elif self.current_turn_val > target_turn:
            self.current_turn_val = max(target_turn, self.current_turn_val - step_size)

        self.smooth_kp += (kp - self.smooth_kp) * LERP_ALPHA
        self.smooth_kd += (kd - self.smooth_kd) * LERP_ALPHA

        tilt_err = tilt - des_tilt
        if not self.in_recovery and abs(tilt) < 0.1:
            self.integral_err = np.clip(self.integral_err + tilt_err, -5.0, 5.0)
        else:
            self.integral_err = 0.0

        out = (self.smooth_kp * tilt_err) + (self.smooth_kd * ang_vel[0]) + (KI_BAL * self.integral_err)

        v_l, v_r = 0, 0
        if self.mode == "FORWARD" and self.impulse_counter < IMPULSE_STEPS:
            v_l = v_r = IMPULSE_MAG
            self.impulse_counter += 1
        else:
            # Balance Priority Logic
            turn_now = self.current_turn_val
            raw_l, raw_r = out + turn_now, out - turn_now
            max_req = max(abs(raw_l), abs(raw_r))
            limit = 25.0
            if max_req > limit:
                if abs(out) > limit:
                    out = np.clip(out, -limit, limit)
                    turn_now = 0.0
                else:
                    turn_now = np.clip(turn_now, -(limit-abs(out)), limit-abs(out))
            v_l, v_r = np.clip(out + turn_now, -limit, limit), np.clip(out - turn_now, -limit, limit)

        p.setJointMotorControl2(self.robotId, 0, p.VELOCITY_CONTROL, targetVelocity=v_l, force=MAX_TORQUE)
        p.setJointMotorControl2(self.robotId, 1, p.VELOCITY_CONTROL, targetVelocity=v_r, force=MAX_TORQUE)
        
        self.draw_debug_text()

    def loop(self):
        while self.running:
            self.process_input()
            self.update_control_logic()
            p.stepSimulation()
            self.global_step += 1
            time.sleep(PHYSICS_DT)
        p.disconnect()

if __name__ == "__main__":
    SegwayManual().loop()