import board
import busio
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)

# This tells every channel to stop sending a signal (Duty Cycle = 0)

for i in range(16):
    pca.channels[i].duty_cycle = 0
pca.deinit()