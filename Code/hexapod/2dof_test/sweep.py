import time
import board
import busio
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.reset()
pca.frequency = 50
time.sleep(0.1)

for channel_num in range(16):
    print(f"Testing Channel {channel_num}...")
    # Move to center
    pca.channels[channel_num].duty_cycle = 4900 
    time.sleep(1)
    # Give it a tiny nudge so you can see/hear it
    pca.channels[channel_num].duty_cycle = 7000
    time.sleep(1)
    # Turn off the PWM so it stops buzzing
    pca.channels[channel_num].duty_cycle = 0