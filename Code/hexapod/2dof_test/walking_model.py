#!/usr/bin/env python3
"""
2DOF Hexapod Controller
Tripod gait walking and turning via PCA9685 + Raspberry Pi 5

Leg layout (top view):
    1   2
  6       3
    5   4

Channel mapping:
  Leg N base -> channel (N-1)*2
  Leg N knee -> channel (N-1)*2 + 1
"""

import time
import math
try:
    from adafruit_pca9685 import PCA9685
    from adafruit_motor import servo as adafruit_servo
    import board
    import busio
    HARDWARE = True
except ImportError:
    print("[SIM MODE] adafruit libraries not found, running in simulation mode")
    HARDWARE = False

# ─── CONFIG ──────────────────────────────────────────────────────────────────

NUM_LEGS = 6

# Servo pulse range (adjust for your servos)
SERVO_MIN_PULSE = 500   # µs
SERVO_MAX_PULSE = 2500  # µs

# Neutral angles (degrees) — tune these so the robot stands level
BASE_NEUTRAL   = 90   # pointing straight out
KNEE_NEUTRAL   = 90   # leg roughly vertical

# Motion parameters
STEP_HEIGHT    = 30   # degrees the knee lifts during a swing phase
BASE_SWING     = 25   # degrees the base rotates forward/back per step
TURN_SWING     = 20   # degrees the base rotates per step when turning

STEP_DURATION  = 0.4  # seconds per half-step (swing or stance)
STEP_SUBSTEPS  = 20   # interpolation substeps for smooth motion

# Tripod groups
#   Group A: legs 1, 3, 5  (odd)
#   Group B: legs 2, 4, 6  (even)
TRIPOD_A = [1, 3, 5]
TRIPOD_B = [2, 4, 6]

# ─── HARDWARE INIT ───────────────────────────────────────────────────────────

servos = {}   # {(leg, joint): servo_object_or_angle}

def init_hardware():
    global servos
    if HARDWARE:
        i2c = busio.I2C(board.SCL, board.SDA)
        pca = PCA9685(i2c)
        pca.frequency = 50
        for leg in range(1, NUM_LEGS + 1):
            base_ch = (leg - 1) * 2
            knee_ch = (leg - 1) * 2 + 1
            servos[(leg, 'base')] = adafruit_servo.Servo(
                pca.channels[base_ch],
                min_pulse=SERVO_MIN_PULSE, max_pulse=SERVO_MAX_PULSE)
            servos[(leg, 'knee')] = adafruit_servo.Servo(
                pca.channels[knee_ch],
                min_pulse=SERVO_MIN_PULSE, max_pulse=SERVO_MAX_PULSE)
        print("Hardware initialised.")
    else:
        for leg in range(1, NUM_LEGS + 1):
            servos[(leg, 'base')] = BASE_NEUTRAL
            servos[(leg, 'knee')] = KNEE_NEUTRAL
        print("Simulation mode: angles will be printed.")

def set_servo(leg, joint, angle):
    """Set a single servo to an angle (0–180°)."""
    angle = max(0, min(180, angle))
    if HARDWARE:
        servos[(leg, joint)].angle = angle
    else:
        servos[(leg, joint)] = angle

def get_servo(leg, joint):
    if HARDWARE:
        return servos[(leg, joint)].angle or BASE_NEUTRAL
    return servos[(leg, joint)]

# ─── POSTURE ─────────────────────────────────────────────────────────────────

def stand():
    """Move all legs to neutral standing position."""
    print("Standing...")
    for leg in range(1, NUM_LEGS + 1):
        set_servo(leg, 'base', BASE_NEUTRAL)
        set_servo(leg, 'knee', KNEE_NEUTRAL)
    time.sleep(0.5)

def legs_to_angles(targets, duration):
    """
    Smoothly interpolate all legs to their target angles.

    targets: dict of {(leg, joint): target_angle}
    duration: total time in seconds
    """
    starts = {k: get_servo(k[0], k[1]) for k in targets}
    for step in range(1, STEP_SUBSTEPS + 1):
        t = step / STEP_SUBSTEPS
        # Smooth ease in/out
        t_smooth = t * t * (3 - 2 * t)
        for (leg, joint), target in targets.items():
            angle = starts[(leg, joint)] + (target - starts[(leg, joint)]) * t_smooth
            set_servo(leg, joint, angle)
        time.sleep(duration / STEP_SUBSTEPS)

# ─── TRIPOD GAIT PRIMITIVES ──────────────────────────────────────────────────

def tripod_step(swing_group, stance_group, base_dir, lift=True):
    """
    Execute one half-cycle of the tripod gait.

    swing_group:   legs that swing forward (lifted)
    stance_group:  legs that push backward (on ground)
    base_dir:      +1 = forward, -1 = backward
    lift:          whether to lift the swing legs
    """
    targets = {}

    # Swing legs: lift knee + rotate base forward
    for leg in swing_group:
        targets[(leg, 'base')] = BASE_NEUTRAL + BASE_SWING * base_dir
        targets[(leg, 'knee')] = KNEE_NEUTRAL - STEP_HEIGHT if lift else KNEE_NEUTRAL

    # Stance legs: push base backward (opposite direction)
    for leg in stance_group:
        targets[(leg, 'base')] = BASE_NEUTRAL - BASE_SWING * base_dir
        targets[(leg, 'knee')] = KNEE_NEUTRAL

    legs_to_angles(targets, STEP_DURATION)

    # Return swing legs knee to ground
    if lift:
        knee_down = {(leg, 'knee'): KNEE_NEUTRAL for leg in swing_group}
        legs_to_angles(knee_down, STEP_DURATION * 0.3)

def tripod_turn_step(swing_group, stance_group, turn_dir):
    """
    One half-cycle for turning in place.

    turn_dir: +1 = turn right, -1 = turn left
    Swing group rotates one way, stance the other.
    """
    targets = {}

    for leg in swing_group:
        # Legs on the left side rotate opposite to legs on the right for a pivot
        side = 1 if leg in [1, 6, 5] else -1
        targets[(leg, 'base')] = BASE_NEUTRAL + TURN_SWING * turn_dir * side
        targets[(leg, 'knee')] = KNEE_NEUTRAL - STEP_HEIGHT

    for leg in stance_group:
        side = 1 if leg in [1, 6, 5] else -1
        targets[(leg, 'base')] = BASE_NEUTRAL - TURN_SWING * turn_dir * side
        targets[(leg, 'knee')] = KNEE_NEUTRAL

    legs_to_angles(targets, STEP_DURATION)

    knee_down = {(leg, 'knee'): KNEE_NEUTRAL for leg in swing_group}
    legs_to_angles(knee_down, STEP_DURATION * 0.3)

# ─── HIGH-LEVEL COMMANDS ─────────────────────────────────────────────────────

def walk_forward(steps=4):
    """Walk forward for a given number of full stride cycles."""
    print(f"Walking forward {steps} steps...")
    stand()
    for i in range(steps):
        if not SIM_PRINT_STEP(i, steps): break
        tripod_step(TRIPOD_A, TRIPOD_B, base_dir=+1)
        tripod_step(TRIPOD_B, TRIPOD_A, base_dir=+1)
    stand()

def walk_backward(steps=4):
    """Walk backward for a given number of full stride cycles."""
    print(f"Walking backward {steps} steps...")
    stand()
    for i in range(steps):
        if not SIM_PRINT_STEP(i, steps): break
        tripod_step(TRIPOD_A, TRIPOD_B, base_dir=-1)
        tripod_step(TRIPOD_B, TRIPOD_A, base_dir=-1)
    stand()

def turn_right(steps=4):
    """Turn right (clockwise) for a number of half-cycles."""
    print(f"Turning right {steps} steps...")
    stand()
    for i in range(steps):
        if not SIM_PRINT_STEP(i, steps): break
        tripod_turn_step(TRIPOD_A, TRIPOD_B, turn_dir=+1)
        tripod_turn_step(TRIPOD_B, TRIPOD_A, turn_dir=+1)
    stand()

def turn_left(steps=4):
    """Turn left (counter-clockwise) for a number of half-cycles."""
    print(f"Turning left {steps} steps...")
    stand()
    for i in range(steps):
        if not SIM_PRINT_STEP(i, steps): break
        tripod_turn_step(TRIPOD_A, TRIPOD_B, turn_dir=-1)
        tripod_turn_step(TRIPOD_B, TRIPOD_A, turn_dir=-1)
    stand()

# ─── SIM HELPER ──────────────────────────────────────────────────────────────

def SIM_PRINT_STEP(i, total):
    if not HARDWARE:
        print(f"  Stride {i+1}/{total} | Angles: " +
              ", ".join(f"L{l}b={servos[(l,'base')]:.0f} k={servos[(l,'knee')]:.0f}"
                        for l in range(1, NUM_LEGS+1)))
    return True

# ─── MAIN ────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    init_hardware()

    # --- Test sequence ---
    walk_forward(steps=4)
    time.sleep(0.5)

    walk_backward(steps=4)
    time.sleep(0.5)

    turn_right(steps=3)
    time.sleep(0.5)

    turn_left(steps=3)
    time.sleep(0.5)

    stand()
    print("Done.")