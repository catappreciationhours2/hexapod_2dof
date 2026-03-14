import time
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

def test_servos():
    # Initialize I2C and PCA9685
    i2c = busio.I2C(board.SCL, board.SDA)
    pca = PCA9685(i2c)
    pca.frequency = 50

    print("Starting Servo Channel Test...")
    print("Each leg will move: Base (45° -> 135°) then Knee (45° -> 135°)")
    print("------------------------------------------------------------")

    try:
        for leg_idx in range(6):
            print(f"Testing Leg {leg_idx}...")
            
            # Define the two joints based on your mapping: (leg_idx * 2) + joint_idx
            base_channel = leg_idx * 2
            knee_channel = (leg_idx * 2) + 1
            
            # 1. Test Base
            print(f"  -> Moving Base (Channel {base_channel})")
            s_base = servo.Servo(pca.channels[base_channel], min_pulse=500, max_pulse=2500)
            s_base.angle = 45
            time.sleep(0.5)
            s_base.angle = 135
            time.sleep(0.5)
            s_base.angle = 90 # Return to neutral
            
            # 2. Test Knee
            print(f"  -> Moving Knee (Channel {knee_channel})")
            s_knee = servo.Servo(pca.channels[knee_channel], min_pulse=500, max_pulse=2500)
            s_knee.angle = 45
            time.sleep(0.5)
            s_knee.angle = 135
            time.sleep(0.5)
            s_knee.angle = 90 # Return to neutral
            
            time.sleep(1) # Pause before next leg

    except KeyboardInterrupt:
        print("\nTest stopped by user.")
    finally:
        pca.deinit()
        print("PCA9685 connection closed.")

if __name__ == "__main__":
    test_servos()