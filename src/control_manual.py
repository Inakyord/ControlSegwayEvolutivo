import pybullet as p
import pybullet_data
import numpy as np
import time
import os
import json
import sys

# --- RUTAS ---
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ASSETS_DIR = os.path.join(SCRIPT_DIR, "..", "assets")
RESULTADOS_DIR = os.path.join(SCRIPT_DIR, "..", "resultados")
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
        print(f"[ERROR] JSON corrupto: {e}")
        sys.exit(1)

# --- CARGA DE PARÁMETROS ---
P = load_parameters()

# 1. CONSTANTES DE TIEMPO (TIGHT LOOP - 240Hz)
PHYSICS_FREQ = 240.0
CONTROL_FREQ = 240.0 
PHYSICS_DT = 1.0 / PHYSICS_FREQ
CONTROL_DT = 1.0 / CONTROL_FREQ
STEPS_PER_CONTROL = 1 

# 2. Variables Genéticas
KP_BAL, KD_BAL = P["KP_BALANCE"], P["KD_BALANCE"]
KP_FWD, KD_FWD = P["KP_FORWARD"], P["KD_FORWARD"]
KP_TRN, KD_TRN = P["KP_TURNING"], P["KD_TURNING"]
KP_REC, KD_REC = P["KP_RECOVERY"], P["KD_RECOVERY"]

# Comportamiento
IMPULSE_STEPS_TOTAL = int(P["IMPULSE_DURATION_SEC"] * CONTROL_FREQ)
IMPULSE_MAG = P["IMPULSE_MAGNITUDE"]
TURN_SPD = P["TURNING_SPEED"]
KP_POS = P["KP_POSITION"]
TGT_TILT = P["TARGET_TILT"]

# Dinámica
REC_THRESH = P["RECOVERY_THRESHOLD"]
LERP_ALPHA = P["LERP_ALPHA"]
KI_BAL = P["KI_BALANCE"]

# Constantes Físicas
MAX_TORQUE = 2.5
MAX_VEL = 25.0

class SegwayController:
    def __init__(self):
        # 1. Configuración Visual y Física
        self.client = p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)
        
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(PHYSICS_DT)
        
        # 2. Cargar Escena
        p.loadURDF("plane.urdf")
        start_orn = p.getQuaternionFromEuler([np.radians(2.0), 0, 0])
        self.robotId = p.loadURDF(URDF_PATH, [0, 0, 0.23], start_orn)
        
        # 3. Fricción y Motores
        for j in [0, 1]:
            p.changeDynamics(self.robotId, j, lateralFriction=1.0, rollingFriction=0.02)
            p.setJointMotorControl2(self.robotId, j, p.VELOCITY_CONTROL, force=0)

        # 4. Variables de Estado
        self.mode = "BALANCE" # Estado inicial
        self.running = True
        self.in_recovery = False
        self.target_y = 0.0
        self.impulse_counter = 0 
        self.turning = 0.0
        
        # Variables de Control (Memoria)
        self.smooth_kp = KP_BAL
        self.smooth_kd = KD_BAL
        self.integral_err = 0.0
        
        self.debug_text_id = -1
        self.global_step = 0
        self.print_instructions()

    def print_instructions(self):
        print("\n" + "="*50)
        print(" TIGHT LOOP CONTROLLER (LATCHED MODE)")
        print(f" Torque: {MAX_TORQUE} | Freq: {CONTROL_FREQ}Hz")
        print("="*50)
        print(" [W] FORWARD (Crucero)")
        print(" [S] STOP (Balanceo estático)")
        print(" [A] SPIN LEFT (Giro izquierda)")
        print(" [D] SPIN RIGHT (Giro derecha)")
        print(" [Q] QUIT")
        print("="*50 + "\n")

    def draw_debug_text(self):
        if self.global_step % 10 != 0: return

        pos, _ = p.getBasePositionAndOrientation(self.robotId)
        txt_pos = [pos[0], pos[1], pos[2] + 0.4]
        
        msg = f"MODE: {self.mode}"
        color = [0, 0, 0]
        
        if self.in_recovery:
            msg = "!!! PANIC !!!"
            color = [1, 0, 0]
        elif self.mode == "FORWARD" and self.impulse_counter < IMPULSE_STEPS_TOTAL:
             msg += " (IMPULSO)"
             color = [0, 0, 1]
            
        self.debug_text_id = p.addUserDebugText(msg, txt_pos, color, textSize=1.5, replaceItemUniqueId=self.debug_text_id)

    def process_input(self):
        keys = p.getKeyboardEvents()
        
        # Quit
        if ord('q') in keys and keys[ord('q')] & p.KEY_WAS_TRIGGERED:
            self.running = False

        # === LÓGICA DE ESTADOS PERSISTENTES (LATCH) ===
        
        # W -> FORWARD
        if ord('w') in keys and (keys[ord('w')] & p.KEY_WAS_TRIGGERED):
            self.mode = "FORWARD"
            self.impulse_counter = 0
            self.turning = 0.0
            print(">> SET MODE: FORWARD")

        # S -> BALANCE (STOP)
        if ord('s') in keys and (keys[ord('s')] & p.KEY_WAS_TRIGGERED):
            self.mode = "BALANCE"
            self.turning = 0.0
            # Guardar posición actual para quedarse quieto aquí
            pos, _ = p.getBasePositionAndOrientation(self.robotId)
            self.target_y = pos[1]
            print(">> SET MODE: BALANCE")

        # A -> LEFT SPIN
        if ord('a') in keys and (keys[ord('a')] & p.KEY_WAS_TRIGGERED):
            self.mode = "LEFT"
            self.turning = TURN_SPD
            # Guardar posición para girar en el sitio
            pos, _ = p.getBasePositionAndOrientation(self.robotId)
            self.target_y = pos[1]
            print(">> SET MODE: LEFT")

        # D -> RIGHT SPIN
        if ord('d') in keys and (keys[ord('d')] & p.KEY_WAS_TRIGGERED):
            self.mode = "RIGHT"
            self.turning = -TURN_SPD
            # Guardar posición para girar en el sitio
            pos, _ = p.getBasePositionAndOrientation(self.robotId)
            self.target_y = pos[1]
            print(">> SET MODE: RIGHT")

    def update_control_logic(self):
        # 1. Sensors
        pos, orn = p.getBasePositionAndOrientation(self.robotId)
        euler = p.getEulerFromQuaternion(orn)
        tilt = euler[1]
        lin_vel, ang_vel = p.getBaseVelocity(self.robotId)
        tilt_vel = ang_vel[0] 
        
        # 2. Panic Logic
        if not self.in_recovery and abs(tilt) > REC_THRESH:
            self.in_recovery = True
        elif self.in_recovery and abs(tilt) < (REC_THRESH * 0.5): 
            self.in_recovery = False
            self.target_y = pos[1] 

        # 3. Gain Selection
        kp, kd = KP_BAL, KD_BAL
        des_tilt = 0.0
        
        if self.in_recovery:
            kp, kd = KP_REC, KD_REC
        elif self.mode == "FORWARD":
            kp, kd = KP_FWD, KD_FWD
            des_tilt = TGT_TILT
        else: 
            # BALANCE, LEFT, y RIGHT usan la lógica de mantener posición
            kp, kd = KP_BAL, KD_BAL
            err_y = self.target_y - pos[1]
            des_tilt = np.clip(KP_POS * err_y, -0.15, 0.15)

        # 4. LERP Smoothing
        self.smooth_kp += (kp - self.smooth_kp) * LERP_ALPHA
        self.smooth_kd += (kd - self.smooth_kd) * LERP_ALPHA

        # 5. PID
        tilt_err = tilt - des_tilt
        
        if not self.in_recovery and self.mode in ["BALANCE", "LEFT", "RIGHT"] and abs(tilt) < 0.1:
            self.integral_err += tilt_err
            self.integral_err = np.clip(self.integral_err, -5.0, 5.0)
        else:
            self.integral_err = 0.0

        out = (self.smooth_kp * tilt_err) + (self.smooth_kd * tilt_vel) + (KI_BAL * self.integral_err)

        # 6. Actuators
        v_l, v_r = 0, 0
        
        if self.mode == "FORWARD" and self.impulse_counter < IMPULSE_STEPS_TOTAL:
            v_l = v_r = IMPULSE_MAG
            self.impulse_counter += 1
        else:
            # En modos giro, self.turning tendrá valor. En otros será 0.
            v_l = np.clip(out + self.turning, -MAX_VEL, MAX_VEL)
            v_r = np.clip(out - self.turning, -MAX_VEL, MAX_VEL)

        p.setJointMotorControl2(self.robotId, 0, p.VELOCITY_CONTROL, targetVelocity=v_l, force=MAX_TORQUE)
        p.setJointMotorControl2(self.robotId, 1, p.VELOCITY_CONTROL, targetVelocity=v_r, force=MAX_TORQUE)
        
        self.draw_debug_text()

    def loop(self):
        print("Simulation Started. Press W/A/S/D to switch modes.")
        while self.running:
            # 1. Process Input
            self.process_input()
            
            # 2. Control Logic
            self.update_control_logic()
            
            # 3. Physics Step
            p.stepSimulation()
            self.global_step += 1
            
            time.sleep(PHYSICS_DT / 10)
        
        p.disconnect()
        print("Simulation Ended.")

if __name__ == "__main__":
    SegwayController().loop()