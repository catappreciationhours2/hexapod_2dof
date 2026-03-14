# import time
# import math
# import numpy as np
# from scipy.spatial.transform import Rotation as R

# try:
#     from adafruit_pca9685 import PCA9685
#     from adafruit_motor import servo as adafruit_servo
#     import adafruit_mpu6050
#     import board
#     import busio
#     from ahrs.filters import Mahony
#     HARDWARE = True
# except ImportError:
#     print("[SIM MODE] adafruit libraries not found, running in simulation mode")
#     HARDWARE = False

# i2c = board.I2C()  # uses board.SCL and board.SDA
# mpu = adafruit_mpu6050.MPU6050(i2c)
# sampling_frequency = 2.0
# orientation = Mahony(frequency=sampling_frequency)

# Q = np.array([1.0, 0.0, 0.0, 0.0])
# prev_row = np.array([1.0, 0.0, 0.0, 0.0])

# # Q = np.array([0, 0, 0, 0])
# # prev_row = np.array([1, 1, 1, 1])

# for i in range(5):

#     print("Acceleration: X:%.2f, Y:%.2f, Z:%.2f m/s^2" % (mpu.acceleration))
#     print("Gyro X:%.2f, Y:%.2f, Z:%.2f deg/s" % (mpu.gyro))
#     gyro_data = np.array(mpu.gyro) 

#     accel_data = np.array(mpu.acceleration)

#     Q = orientation.updateIMU(prev_row, gyr=gyro_data, acc=accel_data)
#     prev_row = Q
#     print(Q)

#     r = R.from_quat(Q)

#     angles_degrees = r.as_euler('zyx', degrees=True)
#     print(f"Angles in degrees for calibration: {angles_degrees}")
#     time.sleep(0.5)


# while True:
#     print("Acceleration: X:%.2f, Y:%.2f, Z:%.2f m/s^2" % (mpu.acceleration))
#     print("Gyro X:%.2f, Y:%.2f, Z:%.2f deg/s" % (mpu.gyro))
#     gyro_data = np.deg2rad(np.array(mpu.gyro)) 

#     accel_data = np.array(mpu.acceleration)

#     Q = orientation.updateIMU(prev_row, gyr=gyro_data, acc=accel_data)
#     prev_row = Q
#     print(Q)

#     r = R.from_quat(Q)

#     angles_degrees_normal = r.as_euler('zyx', degrees=True)
#     angles_degrees_normal[0] = angles_degrees_normal[0] - angles_degrees[0]
#     angles_degrees_normal[1] = angles_degrees_normal[1] - angles_degrees[1]
#     angles_degrees_normal[2] = angles_degrees_normal[2] - angles_degrees[2]

#     print(f"Angles in degrees: {angles_degrees_normal}")
#     angles_degrees = angles_degrees_normal

#     print("")
#     time.sleep(0.5)

# import time
# import math
# import numpy as np
# from scipy.spatial.transform import Rotation as R
# from reset_angles import reset_all_servos

# try:
#     from adafruit_pca9685 import PCA9685
#     from adafruit_motor import servo 
#     import adafruit_mpu6050
#     import board
#     import busio
#     from ahrs.filters import Mahony
#     HARDWARE = True
# except ImportError:
#     print("[SIM MODE] adafruit libraries not found, running in simulation mode")
#     HARDWARE = False

# reset_all_servos()

# i2c = board.I2C()
# mpu = adafruit_mpu6050.MPU6050(i2c)

# # CRITICAL FIX 1: Increase to 100Hz for accurate movement tracking
# sampling_frequency = 100.0 
# orientation = Mahony(frequency=sampling_frequency)

# Q = np.array([1.0, 0.0, 0.0, 0.0])
# prev_row = np.array([1.0, 0.0, 0.0, 0.0])

# # --- CRITICAL FIX 2: GYRO BIAS CALIBRATION ---
# print("Calibrating Gyro Bias... DO NOT MOVE SENSOR!")
# gyro_offsets = np.zeros(3)
# samples = 100
# for _ in range(samples):
#     gyro_offsets += np.array(mpu.gyro)
#     time.sleep(0.01)
# gyro_offsets /= samples
# print(f"Gyro Offsets calculated: {gyro_offsets}")

# print("Letting filter settle...")
# # Run the filter a bit to let gravity (accel) correct the initial pitch/roll
# for i in range(100):
#     # Subtract the offset before converting to radians
#     gyro_data = np.deg2rad(np.array(mpu.gyro) - gyro_offsets) 
#     accel_data = np.array(mpu.acceleration)

#     Q = orientation.updateIMU(prev_row, gyr=gyro_data, acc=accel_data)
#     prev_row = Q
#     time.sleep(0.01)

# # Take our static baseline snapshot
# r_init = R.from_quat(Q)
# calibration_offset = r_init.as_euler('zyx', degrees=True)
# print(f"Baseline offset saved: {calibration_offset}")
# print("--- READY ---")

# # Main loop tracking
# loop_delay = 1.0 / sampling_frequency

# while True:
#     start_time = time.time()

#     # Subtract the gyro offset on every loop
#     gyro_data = np.deg2rad(np.array(mpu.gyro) - gyro_offsets) 
#     accel_data = np.array(mpu.acceleration)

#     Q = orientation.updateIMU(prev_row, gyr=gyro_data, acc=accel_data)
#     prev_row = Q

#     r = R.from_quat(Q)
#     current_angles = r.as_euler('zyx', degrees=True)

#     angles_degrees_normal = current_angles - calibration_offset

#     # Print rounded to 2 decimal places to make it readable
#     print(f"Angles (Z, Y, X): [{angles_degrees_normal[0]:.2f}, {angles_degrees_normal[1]:.2f}, {angles_degrees_normal[2]:.2f}]")
    


#     # Maintain the precise loop frequency
#     elapsed = time.time() - start_time
#     if elapsed < loop_delay:
#         time.sleep(loop_delay - elapsed)



    

import time
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
from reset_angles import reset_all_servos

try:
    from adafruit_servokit import ServoKit
    import adafruit_mpu6050
    import board
    import busio
    from ahrs.filters import Mahony
    HARDWARE = True
except ImportError:
    print("[SIM MODE] adafruit libraries not found, running in simulation mode")
    HARDWARE = False

# 1. Hardware Initialization
reset_all_servos()
i2c = board.I2C()
mpu = adafruit_mpu6050.MPU6050(i2c)

if HARDWARE:
    # Initialize the PCA9685 using ServoKit
    pca = PCA9685(i2c, address=0x40)
    pca.frequency = 50
    # Define which channels your 6 hexagon servos are plugged into
    servo_channels = [0, 1, 2, 3, 4, 5]

# 2. Hexagon Geometry Setup
# Calculate (X, Y) multipliers for 6 servos placed every 60 degrees on a circle
servo_positions = []
for i in range(6):
    angle_rad = math.radians(i * 60)
    servo_positions.append((math.cos(angle_rad), math.sin(angle_rad)))

# 3. IMU Filter Setup
sampling_frequency = 100.0 
orientation = Mahony(frequency=sampling_frequency)
Q = np.array([1.0, 0.0, 0.0, 0.0])
prev_row = np.array([1.0, 0.0, 0.0, 0.0])

# --- GYRO BIAS CALIBRATION ---
print("Calibrating Gyro Bias... DO NOT MOVE SENSOR!")
gyro_offsets = np.zeros(3)
samples = 100
for _ in range(samples):
    gyro_offsets += np.array(mpu.gyro)
    time.sleep(0.01)
gyro_offsets /= samples
print(f"Gyro Offsets calculated: {gyro_offsets}")

print("Letting filter settle...")
for i in range(100):
    gyro_data = np.deg2rad(np.array(mpu.gyro) - gyro_offsets) 
    accel_data = np.array(mpu.acceleration)
    Q = orientation.updateIMU(prev_row, gyr=gyro_data, acc=accel_data)
    prev_row = Q
    time.sleep(0.01)

r_init = R.from_quat(Q)
calibration_offset = r_init.as_euler('zyx', degrees=True)
print(f"Baseline offset saved: {calibration_offset}")
print("--- STABILIZATION ACTIVE ---")

loop_delay = 1.0 / sampling_frequency

# --- TUNING PARAMETERS ---
BASE_ANGLE = 0  # Centered at 90 so the servos can move +10 and -10 degrees. 
RESPONSE_GAIN = 1.0  
MAX_TILT = 10.0  # HARD LIMIT: Maximum allowed tilt in degrees from the BASE_ANGLE

try:
    while True:
        start_time = time.time()

        # Read and filter IMU data
        gyro_data = np.deg2rad(np.array(mpu.gyro) - gyro_offsets) 
        accel_data = np.array(mpu.acceleration)

        Q = orientation.updateIMU(prev_row, gyr=gyro_data, acc=accel_data)
        prev_row = Q

        r = R.from_quat(Q)
        current_angles = r.as_euler('zyx', degrees=True)

        # Calculate stabilized target angles
        angles_degrees_normal = current_angles - calibration_offset
        
        pitch = angles_degrees_normal[1] # Y-axis rotation
        roll = angles_degrees_normal[2]  # X-axis rotation

        # --- APPLY TO SERVOKIT ---
        if HARDWARE:
            for i, (x_mult, y_mult) in enumerate(servo_positions):
                # Calculate raw adjustment based on servo's position in the hexagon
                raw_adjustment = (pitch * y_mult) + (roll * x_mult)
                
                # Apply gain
                scaled_adjustment = raw_adjustment * RESPONSE_GAIN
                
                # CRITICAL FIX: Clamp the adjustment so it never exceeds +/- 10 degrees
                constrained_adjustment = max(-MAX_TILT, min(MAX_TILT, scaled_adjustment))
                
                # Calculate final target angle
                target_angle = BASE_ANGLE + constrained_adjustment
                
                # Absolute safety clamp for ServoKit bounds
                target_angle = max(0, min(180, target_angle))
                
                # Move the specific servo
                s = servo.Servo(pca.channels[channel_num], 
                                min_pulse=MIN_PULSE, 
                                max_pulse=MAX_PULSE)

        # Print out the pitch and roll for debugging
        print(f"Pitch: {pitch:.1f} | Roll: {roll:.1f}")

        # Maintain 100Hz loop speed
        elapsed = time.time() - start_time
        if elapsed < loop_delay:
            time.sleep(loop_delay - elapsed)

except KeyboardInterrupt:
    print("\nTest stopped by user. Returning servos to flat...")
    if HARDWARE:
        for channel in servo_channels:
            kit.servo[channel].angle = BASE_ANGLE
except Exception as e:
    print(f"\nAn error occurred: {e}")