import board
import busio
from adafruit_pca9685 import PCA9685

# Initialize I2C
i2c = busio.I2C(board.SCL, board.SDA)
print('initialized!')

# Initialize PCA9685
pca = PCA9685(i2c)
pca.frequency = 50 # Standard for servos
print('here!')