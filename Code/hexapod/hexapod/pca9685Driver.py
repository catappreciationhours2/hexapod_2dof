"""
Driver functions to communicate with the PCA9685 I2C Servo Driver.
Replaces the SSC-32U Serial Driver.
"""
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import numpy as np
from typing import Any, Optional

# --- CONFIGURATION ---
# The PCA9685 typically uses 12-bit resolution (0-4095)
# Most analog servos use 500us to 2500us pulse widths.
MIN_PULSE = 500
MAX_PULSE = 2500
# ---------------------

def angleToPW(angle: float) -> float:
    """
    Still returns pulse width in microseconds for compatibility, 
    though the new sendData handles the conversion to PWM duty cycle.
    """
    return round(2000 * angle / 180 + 500)

def connect(address: int = 0x40) -> PCA9685:
    """
    Connects to the PCA9685 via I2C.
    Default address is usually 0x40.
    """
    try:
        i2c = busio.I2C(board.SCL, board.SDA)
        pca = PCA9685(i2c, address=address)
        pca.frequency = 50  # Standard 50Hz for servos
        return pca
    except Exception as e:
        raise ConnectionError(f"Could not connect to PCA9685: {e}")

def disconnect(pca: PCA9685) -> bool:
    """Deinitializes the PCA9685."""
    pca.deinit()
    print('PCA9685 connection closed.')
    return True

def sendPositions(pca: PCA9685, angles: np.ndarray):
    """
    Instead of building a serial string, this function directly
    commands the servos over I2C.
    
    Parameters:
    ----------
    pca: The PCA9685 object from connect()
    angles: 6x3 numpy array of angles (Degrees)
    """
    if angles.shape != (6, 3):
        raise ValueError('Input angles must be a 6x3 numpy array')

    # Apply the same coordinate transformations as the original driver
    temp_angles = np.copy(angles)
    temp_angles[0:3, 2] = - temp_angles[0:3, 2]
    temp_angles[3:6, 0] = (temp_angles[3:6, 0] + 360) % 360.0
    temp_angles[3:6, 1] = - temp_angles[3:6, 1]
    
    adjustment = np.array([[90, 90, 90],
                           [90, 90, 90],
                           [90, 90, 90],
                           [270, 90, 90],
                           [270, 90, 90],
                           [270, 90, 90]])
    
    processed_angles = adjustment - temp_angles

    # Loop through legs and joints
    for leg_idx in range(6):
        for joint_idx in range(2): # 2-DOF logic (Base and Knee)
            
            # Channel mapping: Leg 0 uses 0,1; Leg 1 uses 2,3...
            channel = (leg_idx * 2) + joint_idx
            angle = processed_angles[leg_idx, joint_idx]
            
            # Constrain angle to 0-180 to prevent servo damage
            angle = max(0, min(180, angle))
            
            # Create a servo object on that channel
            # We define pulse range here to match your specific servos
            s = servo.Servo(pca.channels[channel], 
                            min_pulse=MIN_PULSE, 
                            max_pulse=MAX_PULSE)
            s.angle = angle

    return True