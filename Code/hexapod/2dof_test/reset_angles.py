import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import time

# --- Configuration ---
# Match the pulse widths to your specific servos (usually 500 to 2500)
MIN_PULSE = 500
MAX_PULSE = 2500
TARGET_ANGLE = 90

def reset_all_servos():
    try:
        # Initialize I2C and PCA9685
        i2c = busio.I2C(board.SCL, board.SDA)
        pca = PCA9685(i2c, address=0x40)
        pca.frequency = 50  # Standard 50Hz for analog servos
        
        print(f"Setting all 12 servos to {TARGET_ANGLE} degrees...")

        # Loop through your 6 legs
        for leg_idx in range(6):
            # Loop through the 2 joints per leg (Base and Knee)
            for joint_idx in range(2):
                # Your mapping: channel = (leg * 2) + joint
                channel_num = (leg_idx * 2) + joint_idx
                
                # Create the servo object
                s = servo.Servo(pca.channels[channel_num], 
                                min_pulse=MIN_PULSE, 
                                max_pulse=MAX_PULSE)
                
                # Command the angle
                s.angle = TARGET_ANGLE
                print(f"Leg {leg_idx}, Joint {joint_idx} (Channel {channel_num}) set to {TARGET_ANGLE}")
        
        print("\nAll servos initialized. Keep the script running for a moment to ensure positions are held.")
        time.sleep(1)
        
        # Cleanup
        pca.deinit()
        print("PCA9685 deactivated.")

    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    reset_all_servos()