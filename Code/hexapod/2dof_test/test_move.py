import time
from adafruit_servokit import ServoKit

# Initialize the PCA9685 using ServoKit. 
# 'channels=16' matches the 16 pins on the PCA9685 board.
kit = ServoKit(channels=16)

print("--- PCA9685 Servo Test ---")
print("Targeting Channel 0...")

try:
    while True:
        # Move to 0 degrees
        print("0 degrees")
        kit.servo[0].angle = 0
        time.sleep(1)

        # Move to 90 degrees (Center)
        print("90 degrees")
        kit.servo[0].angle = 90
        time.sleep(1)

        # Move to 180 degrees
        print("180 degrees")
        kit.servo[0].angle = 180
        time.sleep(1)

except KeyboardInterrupt:
    print("\nTest stopped by user.")
except Exception as e:
    print(f"\nAn error occurred: {e}")
    print("Check if your I2C wires are loose or if the battery is connected.")