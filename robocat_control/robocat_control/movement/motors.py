import time
import threading
from typing import Optional

import board
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

from robocat_control.movement.simulation_data import *  # noqa: F403
from robocat_control.movement.inverse_kinematics.steps import position_steps
# Inicialitzacio del bus I2C i la controladora PCA9685
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50  # 50Hz es l'estandard per a servos

# Inicialitza els 10 primers canals com a servos
_i2c_lock = threading.Lock()
_DEADBAND_DEG = 2.5
_MAX_DELTA_DEG_PER_STEP = 4.0
_RELEASE_AFTER_MOVE = False
_RELEASE_DELAY_SEC = 0.25
_SERVO_MIN_ANGLE = 3
_SERVO_MAX_ANGLE = 177
_last_servo_angles = {}
servos = []
for i in range(16):
    s = servo.Servo(pca.channels[i], min_pulse=500, max_pulse=2500)
    servos.append(s)


def _normalize_angle(angle):
    return int(round(max(_SERVO_MIN_ANGLE, min(_SERVO_MAX_ANGLE, angle))))


def _apply_servo_angle(servo_obj, angle):
    target = _normalize_angle(angle)
    key = id(servo_obj)
    prev = _last_servo_angles.get(key)
    if prev is not None and abs(target - prev) < _DEADBAND_DEG:
        return False
    servo_obj.angle = target
    _last_servo_angles[key] = target
    return True


class Pota:
    def __init__(self, cadera, genoll,right,front):
        self.servo_up = cadera
        self.servo_down = genoll
        self.state = "sit"  # Estat inicial de la pota
        self.right = right  
        self.front = front
        self.old_state = "start"

    def _planned_steps(self, steps, inter_method="linear"):
        new_pos = position(self.state)
        old_pos = position(self.old_state)

        up = 0
        down = 0
        if self.right:
            up = lambda a : 90 - a
            down = lambda a : a
        else:
            up = lambda a : 90 + a
            down = lambda a : 180 - a
        
        correction_factor = (up, down)
        pos_steps = position_steps(old_pos, new_pos, steps, inter_method, correction_factor)

        up_steps = [pos[0] for pos in pos_steps]
        down_steps = [pos[1] for pos in pos_steps]
        return up_steps, down_steps

    def set_new_position(self, t, inter_method="linear"):
        # Force linear interpolation to reduce oscillations.
        inter_method = "linear"
        steps = max(1, int(t / 0.1))
        delay = t / steps
        up_steps, down_steps = self._planned_steps(steps, inter_method)
        for up_angle, down_angle in zip(up_steps, down_steps):
            with _i2c_lock:
                _apply_servo_angle(self.servo_up, up_angle)
                _apply_servo_angle(self.servo_down, down_angle)
            time.sleep(delay)


    def up(self):
        #DEPRECATED
        pass

    def down(self, new_state):
        #DEPRECATED
        pass

    def forward(self):  
        self.set_state(forwards(self.state))

    def backward(self):
        self.set_state(backwards(self.state))
    
    def set_state(self, new_state):
        """Estableix un nou estat per a la pota."""
        assert new_state in position_states.keys(), "Invalid state"
        self.old_state = self.state
        self.state = new_state
    

class EstructuraPotes:
    """Classe per gestionar les potes del quadrupede."""
    def __init__(self, ultrasons: Optional[object] = None):
        self.ultrasons = ultrasons

        self.legs = [
            Pota(servos[8], servos[7], True,True),  # Pota 1
            Pota(servos[11], servos[10], False,True),  # Pota 2
            Pota(servos[2], servos[1], True, False),  # Pota 3
            Pota(servos[5], servos[4], False, False)   # Pota 4
        ]
        self._move_legs(self.legs, t=0.3, inter_method="linear")

    def _move_legs(self, legs, t=0.3, inter_method="linear"):
        # Force linear interpolation globally to reduce jitter.
        inter_method = "linear"
        base_steps = max(1, int(t / 0.1))
        trajectories = []
        max_delta = 0.0
        for leg in legs:
            up_steps, down_steps = leg._planned_steps(base_steps, inter_method)
            trajectories.append((leg, up_steps, down_steps))
            if len(up_steps) > 1:
                max_delta = max(max_delta, max(abs(b - a) for a, b in zip(up_steps, up_steps[1:])))
            if len(down_steps) > 1:
                max_delta = max(max_delta, max(abs(b - a) for a, b in zip(down_steps, down_steps[1:])))

        scale = 1.0
        if max_delta > _MAX_DELTA_DEG_PER_STEP and _MAX_DELTA_DEG_PER_STEP > 0:
            scale = max_delta / _MAX_DELTA_DEG_PER_STEP
        steps = min(120, max(base_steps, int(base_steps * scale + 0.999)))

        trajectories = []
        for leg in legs:
            up_steps, down_steps = leg._planned_steps(steps, inter_method)
            trajectories.append((leg.servo_up, up_steps, leg.servo_down, down_steps))

        delay = t / steps

        for idx in range(steps):
            with _i2c_lock:
                for servo_up, up_steps, servo_down, down_steps in trajectories:
                    _apply_servo_angle(servo_up, up_steps[idx])
                    _apply_servo_angle(servo_down, down_steps[idx])
            time.sleep(delay)

        if _RELEASE_AFTER_MOVE:
            time.sleep(_RELEASE_DELAY_SEC)
            with _i2c_lock:
                released = set()
                for leg in legs:
                    if id(leg.servo_up) not in released:
                        leg.servo_up.angle = None
                        _last_servo_angles.pop(id(leg.servo_up), None)
                        released.add(id(leg.servo_up))
                    if id(leg.servo_down) not in released:
                        leg.servo_down.angle = None
                        _last_servo_angles.pop(id(leg.servo_down), None)
                        released.add(id(leg.servo_down))
    
    def set_body_state(self,text):
        for leg in self.legs:
            leg.set_state(text)    

    def set_position(self,text):
        for leg in self.legs:
            leg.set_state(text)
        self._move_legs(self.legs, t=0.3, inter_method="linear")

    def body_forward(self):
        for leg in self.legs:
            leg.backward()

    def body_backward(self):
        for leg in self.legs:
            leg.forward()

    def body_upward(self):
        for leg in self.legs:
            leg.set_state("center")

    def body_downward(self):
        for leg in self.legs:
            leg.set_state("sit")
            
    def sit_hind_legs(self, t=0.2):
        """Sit using only the hind legs while front legs are raised."""
        
        # hind legs
        self.legs[2].set_state("sit_down")
        self.legs[3].set_state("sit_down")
        # raise front legs
        self.legs[0].set_state("up")
        self.legs[1].set_state("up")

        self._move_legs(self.legs, t=t, inter_method="linear")
        
            
    def strech(self, t=0.2):
        """Sit using only the hind legs while front legs are raised."""
        
        # hind legs
        self.legs[2].set_state("up")
        self.legs[3].set_state("up")
        # raise front legs
        self.legs[0].set_state("front_zero")
        self.legs[1].set_state("front_zero")

        self._move_legs(self.legs, t=t, inter_method="linear")
        

    def init_bot(self, t=0.2):
        for leg in self.legs:
            if leg.front:
                leg.set_state("front_zero")
            else:
                leg.set_state("back_zero")
    
        self._move_legs(self.legs, t=t, inter_method="linear")

    #set_positions
    def get_states(self):
        return [leg.state for leg in self.legs]

    """def follow_order(self, order, states, t=1):"""
    def follow_order(self, order,states = None, t=1):
        legs = self.legs
        inter_method = 'linear'

        direction, leg_n, method = order
        _ = method

        #BODY
        if leg_n == 4:
            if   direction == 'f': 
                self.body_forward()
            elif direction == 'b': 
                self.body_backward()
            elif direction == 'u': 
                self.body_upward()
            elif direction == 'd': 
                self.body_downward()
            else:
                raise ValueError(f"Unknown body direction '{direction}'")
        
        #LEG
        elif type(leg_n) == int:

            leg = legs[leg_n]
            legs = [leg]

            if direction ==  'f': 
                leg.forward()
            elif direction ==  'b':
                leg.backward()
            elif direction == 'ff':
                leg.forward()
                leg.forward()
            elif direction == 'bb':
                leg.backward()
                leg.backward()
            else:
                raise ValueError(f"Unknown leg direction '{direction}'")
        else:
            legs = [legs[i] for i in leg_n]
            for leg in legs:
                leg.set_state(direction)

        new_states = self.get_states()

        # Function move Body (single synchronized loop, no per-leg threads).
        self._move_legs(legs, t=t, inter_method=inter_method)
        return new_states

    def follow_sequance(self, sequance, cycles=1, t = 1):
        """states = self.init_bot()"""
        states = self.get_states()

        for order in sequance["start"]:
            states = self.follow_order(order, states, t)
        
        for _ in range(cycles):
            for order in sequance["cycle"]:
                #if self.ultrasons and self.ultrasons.mesura_distancia() > config.LLINDAR_ULTRASONIC:
                states = self.follow_order(order, states, t)
            
        for order in sequance["end"]:
            states = self.follow_order(order, states, t)

def get_angle(state,right):
    (up, down) = position(state)
    if right:
        up = 90 - up
        down = down
    else:
        up = 90 + up
        down = 180 - down
    return (up,down)

def new_angle(servo,angle_final,angle_inicial, duracio, passos=30):
    pas = (angle_final - angle_inicial) / passos
    delay = duracio / passos

    for i in range(passos + 1):
        angle_actual = angle_inicial + i * pas
        with _i2c_lock:
            _apply_servo_angle(servo, angle_actual)
        time.sleep(delay)

def new_angles(servo,angles, delay):
    for angle in angles:
        with _i2c_lock:
            _apply_servo_angle(servo, angle)
        time.sleep(delay)

def new_angles_pair(servo_a, angles_a, servo_b, angles_b, delay):
    if len(angles_a) != len(angles_b):
        raise ValueError("angles_a and angles_b must have the same length")
    for angle_a, angle_b in zip(angles_a, angles_b):
        with _i2c_lock:
            _apply_servo_angle(servo_a, angle_a)
            _apply_servo_angle(servo_b, angle_b)
        time.sleep(delay)

# Crear potes (ajusta els canals segons com els tinguis connectats)

def set_servo_angle(index, angle):
    """Posa un servo concret a un angle determinat."""
    if not (0 <= index < len(servos)):
        raise ValueError("Index de servo fora de rang.")
    if not (0 <= angle <= 180):
        raise ValueError("L'angle ha de ser entre 0 i 180 graus.")
    with _i2c_lock:
        _apply_servo_angle(servos[index], angle)

def sweep_servo(index, delay=0.01):
    """Mou el servo d'un extrem a l'altre per provar el rang complet."""
    if not (0 <= index < len(servos)):
        raise ValueError("Index de servo fora de rang.")

    # Anada: de 0 a 180 graus
    for angle in range(0, 181, 1):
        with _i2c_lock:
            _apply_servo_angle(servos[index], angle)
        time.sleep(delay)

    # Tornada: de 180 a 0 graus
    for angle in range(180, -1, -1):
        with _i2c_lock:
            _apply_servo_angle(servos[index], angle)
        time.sleep(delay)

def mou_cap(index=15, temps_gir=0.4, pausa=0.2, intensitat=30):
    """
    Mou un servo de 360 graus des de posicio aturada cap a dreta i esquerra.

    - intensitat: valor entre 0 i 90 (es suma/resta a 90)
      Ex: intensitat=30 -> 120 dreta, 60 esquerra
    """
    # DRETA
    with _i2c_lock:
        _apply_servo_angle(servos[index], 94 + intensitat)
    time.sleep(temps_gir)
    with _i2c_lock:
        _apply_servo_angle(servos[index], 94)
    time.sleep(pausa)

    # ESQUERRA
    with _i2c_lock:
        _apply_servo_angle(servos[index], 94 - intensitat)
    time.sleep(temps_gir)
    with _i2c_lock:
        _apply_servo_angle(servos[index], 94)
    time.sleep(pausa)


def test_aturada(servo = servos[15]):
    print("Buscant el punt d'aturada...")
    for angle in range(85, 96):  # Prova valors entre 85 i 95
        with _i2c_lock:
            _apply_servo_angle(servo, angle)
        print(f"Provat angle: {angle}")
        time.sleep(1)
        with _i2c_lock:
            _apply_servo_angle(servo, 94)  # pausa entre intents
        time.sleep(0.3)

with _i2c_lock:
    _apply_servo_angle(servos[15], 94)  # pausa entre intents
