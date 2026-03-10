"""
Driver functions to communicate with the Lynxmotion SSC-32U.

Functions to communicate with the Lynxmotion SSC-32U to drive the 18 servos of
the hexapod. These functions format commands based on the user manual for the
controller board.

Functions
---------
angleToPW:
    Convert an angle to the pulse width to command that angle.
anglesToSerial:
    Takes the hexapods servo angles and converts them to a serial command.
connect:
    Connect to the input COM port.
disconnect:
    Disconnects from the serial port.
sendData:
    Sends the commands for the servos to the Lynxmotion SSC-32U.

Notes
-----
As of right now, this product may have been discontinued.
"""
import serial
from serial import SerialException
import numpy as np
from typing import Any, Optional


def angleToPW(angle: float) -> float:
    """
    Convert an angle to the pulse width to command that angle.

    Takes an input `angle` in degrees and converts it to the pulse width to
    command that angle in microseconds.

    Parameters
    ----------
    angle: float
        An angle in degrees for the servo to move to.

    Returns
    -------
    float
        The pulse width in microseconds that represents the angle.

    Notes
    -----
    The pulse width is between 500 microseconds and 2500 microseconds for an
    angle between 0 and 180 degrees.
    """
    # returns the pulse width as the equivalent whole number between 500us
    # (0 degrees) and 2500us (180 degrees)
    return round(2000 * angle / 180 + 500)


def anglesToSerial(angles: np.ndarray, speed: Optional[int] = None,
                   time: Optional[int] = None) -> bytes:
    """
    Takes the hexapods servo angles and converts them to a serial command.

    Converts an array of 18 servo angles to the formatted serial command for
    the Lynxmotion SSC-32U. The servo angles need to be in the format made in
    the leg module.

    Parameters
    ----------
    angles: np.ndarray
        6x3 numpy array of the 18 servo angles of the hexapod's legs.
    speed: int, optional
        How fast the servos move in microseconds/second (the default is None).
        A speed of 1000 takes 1 second to go 90 degrees.
    time: int, optional
        The maximum amount of time in milliseconds to move (the default is
        None).

    Returns
    -------
    serial_string: bytes
        The formatted message for all 18 servos together.

    Raises
    ------
    Exception
        If the input numpy array is not a 6x3 array.

    See Also
    --------
    angleToPW:
        Convert an angle to the pulse width to command that angle.

    Notes
    -----
    The message is converted to bytes in the return statement.
    """
    if angles.shape == (6, 3):
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
        temp_angles = adjustment - temp_angles
        angles_processed = temp_angles.flatten()
    else:
        raise Exception(
            'Input angles were the wrong format. Should be a 6x3 numpy array')
    # speed is in microseconds/second and time is in milliseconds. A speed of
    # 1000us takes 1 second to go 90 degrees
    serial_string = ''
    # We loop through each leg (0-5)
    for leg_idx in range(6):
        # We only loop through the first TWO joints (0=Base, 1=Knee)
        # Joint 2 (Tibia) is ignored.
        for joint_idx in range(2):
            
            # CALCULATE THE PHYSICAL PIN (Channel)
            # Standard SSC-32U mapping for 3-DOF is usually (leg * 4) or continuous.
            # If your physical servos are plugged into pins 0,1, 4,5, 8,9... (skipping 2,3, 6,7)
            # use this logic:
            pin = (leg_idx * 2) + joint_idx 
            
            angle = temp_angles[leg_idx, joint_idx]
            
            serial_string += '#' + str(pin) + 'P' + str(angleToPW(angle))
            
            if speed is not None:
                serial_string += 'S' + str(speed)

    if time is not None:
        serial_string += 'T' + str(time)
        
    serial_string += '\r'
    return bytes(serial_string, 'ascii')


def connect(com: str) -> Any:
    """
    Connect to the input COM port.

    Open a serial port with the Lynxmotion SSC-32U on the desired COM port.

    Parameters
    ----------
    com: str
        COM port to connect to as a string.

    Returns
    -------
    ser: Serial Port
        The formatted serial port from pyserial.

    See Also
    --------
    disconnect
    """
    try:
        ser = serial.serial_for_url(com)
        ser.baudrate = 9600
        return ser
    except SerialException:
        raise SerialException('Serial port did not open')


def disconnect(ser: Any) -> bool:
    """
    Disconnects from the serial port.

    Parameters
    ----------
    ser: Serial Port
        The pyserial Serial Port connected to by connect.

    Returns
    -------
    bool
        Returns True if the serial port closed and False if not.

    See Also
    --------
    connect
    """
    ser.close()
    print('Serial port is closed.')
    return True


def sendData(ser: Any, serial_string: bytes) -> bool:
    """
    Sends the commands for the servos to the Lynxmotion SSC-32U.

    Takes the input byte string and writes it across the input serial port.

    Parameters
    ----------
    ser: Serial Port
        The pyserial serial port made in the connect function
    serial_string: bytes
        The message to move the servos with the Lynxmotion SSC-32U.

    Returns
    -------
    bool
        Always returns true at the end of the function.

    See Also
    --------
    connect
    anglesToSerial:
        Takes the hexapods servo angles and converts them to a serial command.
    """
    try:
        ser.write(serial_string)
        return True
    except SerialException:
        raise SerialException('Data did not Write!')
