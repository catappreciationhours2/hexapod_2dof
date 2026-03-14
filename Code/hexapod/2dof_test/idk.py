import time
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import adafruit_mpu6050
from ahrs.filters import Mahony

# ==========================================
# 1. HARDWARE INITIALIZATION (Staggered)
# ==========================================
print("Starting I2C Bus...")
i2c = busio.I2C(board.SCL, board.SDA)
time.sleep(0.5) # Give the bus a moment to stabilize

print("Initializing Servo Driver (PCA9685)...")
pca = PCA9685(i2c)
pca.frequency = 50
time.sleep(0.5) # Let the power stabilize after driver wakes up

print("Initializing IMU (MPU6050)...")
try:
    mpu = adafruit_mpu6050.MPU6050(i2c)
    print("IMU found successfully!")
except Exception as e:
    print(f"CRITICAL ERROR: Could not find IMU. Error: {e}")
    exit()

# ==========================================
# 2. SERVO SETUP & HOME POSITIONS
# ==========================================
MIN_PULSE = 500
MAX_PULSE = 2500
LIMIT = 15.0  # Max degrees away from home position

# Initialize 12 servo objects
servos = []
for i in range(12):
    s = servo.Servo(pca.channels[i], min_pulse=MIN_PULSE, max_pulse=MAX_PULSE)
    servos.append(s)

# Define Home Positions: Even = Base (0 deg), Odd = Knee (180 deg)
HOME_ANGLES = [180 if i % 2 != 0 else 0 for i in range(12)]

print("Moving to home positions...")
for i, s in enumerate(servos):
    s.angle = HOME_ANGLES[i]
time.sleep(1) # Let the physical robot settle

# ==========================================
# 3. IMU CALIBRATION
# ==========================================
SAMPLING_FREQ = 100.0
LOOP_DELAY = 1.0 / SAMPLING_FREQ
orientation = Mahony(frequency=SAMPLING_FREQ)
Q = np.array([1.0, 0.0, 0.0, 0.0])
prev_row = np.array([1.0, 0.0, 0.0, 0.0])

print("Calibrating Gyro Bias... DO NOT MOVE SENSOR!")
gyro_offsets = np.zeros(3)
for _ in range(100):
    gyro_offsets += np.array(mpu.gyro)
    time.sleep(0.01)
gyro_offsets /= 100

print("Letting filter settle...")
for _ in range(100):
    gyro_data = np.deg2rad(np.array(mpu.gyro) - gyro_offsets)
    accel_data = np.array(mpu.acceleration)
    Q = orientation.updateIMU(prev_row, gyr=gyro_data, acc=accel_data)
    prev_row = Q
    time.sleep(0.01)

r_init = R.from_quat(Q)
calibration_offset = r_init.as_euler('zyx', degrees=True)
print("--- STABILIZATION ACTIVE ---")

# ==========================================
# 4. MAIN STABILIZATION LOOP
# ==========================================
try:
    while True:
        start_time = time.time()

        # Update Orientation
        gyro_data = np.deg2rad(np.array(mpu.gyro) - gyro_offsets)
        accel_data = np.array(mpu.acceleration)
        Q = orientation.updateIMU(prev_row, gyr=gyro_data, acc=accel_data)
        prev_row = Q

        r = R.from_quat(Q)
        current_angles = r.as_euler('zyx', degrees=True) - calibration_offset
        
        pitch = current_angles[1] # Front-to-back tilt
        roll = current_angles[2]  # Side-to-side tilt

        # Map Offsets to Legs (Clockwise Layout)
        for leg_idx in range(6):
            knee_channel = (leg_idx * 2) + 1
            
            # --- Pitch logic (X-axis: Nose up/down) ---
            if leg_idx in [0, 5]:      # Front legs
                pitch_correction = pitch 
            elif leg_idx in [2, 3]:    # Back legs
                pitch_correction = -pitch
            else:                      # Middle legs (1 and 4)
                pitch_correction = 0   

            # --- Roll logic (Y-axis: Side to side) ---
            if leg_idx in [0, 1, 2]:   # Right side legs
                roll_correction = -roll
            elif leg_idx in [3, 4, 5]: # Left side legs
                roll_correction = roll

            # Apply the 15-degree limit and set the angle
            total_offset = pitch_correction + roll_correction
            safe_offset = np.clip(total_offset, -LIMIT, LIMIT)
            
            # Add the offset to the home angle (180 for knees)
            new_angle = HOME_ANGLES[knee_channel] + safe_offset
            
            # Final hardware safety clip
            servos[knee_channel].angle = np.clip(new_angle, 0, 180)

        # Maintain precise loop frequency
        elapsed = time.time() - start_time
        if elapsed < LOOP_DELAY:
            time.sleep(LOOP_DELAY - elapsed)

except KeyboardInterrupt:
    print("\nStopping...")
    pca.deinit()