"""
Scripts for controlling the hexapod.

These are scripts that control how the hexapod walks and turns. This module
combines the work of the rest of the library and as a result, imports most of
it to utilize most of the functions.

Functions
---------
emgController:
    Controls the hexapod to walk or turn based on EMG.
sendPositions:
    Send each position in a set to the servo controller.
sit:
    Tells the Hexapod to sit with its body on the ground.
stand:
    Tells the hexapod to stand in the neutral position.
turnCycle:
    Tells the hexapod to turn to an angle without EMG
walkCycle:
    Tells the hexapod to walk a specified distance without the need for EMG
"""
import string
from hexapod.leg import recalculateLegAngles, startLegPos, legModel, getFeetPos
from hexapod.body import bodyPos, bodyAngle
from hexapod.move import (emgToWalk, resetWalkStance, emgToTurn,
                          resetTurnStance, walk, turn, simultaneousWalkTurn)
from hexapod.ssc32uDriver import anglesToSerial, connect, sendData
from hexapod.piToPi import (emgEstablishServer, gamePadEstablishServer,
                            switchMode, pollGamePad)
from math import hypot, atan2, degrees
import numpy as np
from typing import Any
from time import sleep

# NEED TO UPDATE: start_radius


def emgController(usb_port: string, mode: bool) -> None:
    """
    Controls the hexapod to walk or turn based on EMG.

    Takes in EMG signals and tells the hexapod how to walk or turn based on
    those EMG signals. The signals are sent over a COM port to the Lynxmotion
    SSC32U servo controller. This controller combines the work of the rest of
    the library to get the hexapod to move. This is the main function to run
    when using the hexapod for its intended purpose.

    Parameters
    ----------
    usb_port: string
        The name of the port to connect to
    mode: bool
        This parameter determines if the hexapod is walking or turning. `mode`
        equalling 1 is walking and 0 is turning.

    See Also
    --------
    hexapod.piToPi.switchMode:
        Sends a boolean based on EMG to indicate when to change movement
        modes.
    gamePadController:
        Controls the hexapod to walk or turn based on a game pad.

    Notes
    -----
    This function has the serial port to communicate to and the threshold
    of co-contraction to switch modes hardcoded.
    """
    port = connect(usb_port)  # connect to the servo controller
    conn = emgEstablishServer()
    # setup the starting robot positions
    body_model = bodyPos(pitch=0, roll=0, yaw=0, Tx=0, Ty=0, Tz=0,
                         body_offset=85)
    start_leg = startLegPos(body_model, start_radius=170, start_height=60)
    # get the serial message from the angles
    message = anglesToSerial(start_leg, 500, 2000)
    sendData(port, message)  # send the serial message
    leg_model = legModel(start_leg, body_model)

    # iterate forever
    previous_step = 0
    previous_turn_angle = 0
    right_foot = True
    while True:
        if mode:  # True = walk, False = turn
            [leg_model, right_foot, previous_step, positions] =\
                emgToWalk(body_model, leg_model, right_foot, previous_step,
                          conn, max_distance=30)
        else:
            [leg_model, right_foot, previous_turn_angle, positions] =\
                emgToTurn(body_model, leg_model, right_foot,
                          conn, previous_turn_angle, max_turn_angle=15)

        sendPositions(port, positions, body_model)

        if switchMode(conn, 0.75):
            if mode:
                [leg_model, right_foot, positions] =\
                    resetWalkStance(body_model, leg_model, right_foot,
                                    previous_step, previous_angle=90)
                previous_step = 0
            else:
                [leg_model, right_foot, positions] =\
                    resetTurnStance(body_model, leg_model, right_foot,
                                    previous_turn_angle)
            mode = not mode
            sendPositions(port, positions, body_model)


def gamePadController(usb_port: string, mode: int) -> None:
    """
    Controls the hexapod to walk or turn based on a game pad.

    Takes in input from a game pad, currently an Xbox controller, and
    move the hexapod based on this input. Allows for the switching between
    states of movement to walk, turn, and move the body.

    Parameters
    ----------
    usb_port: string
        The name of the port to connect to
    mode: int
        This parameter determines if the current state of movement.

    See Also
    --------
    emgController:
        Controls the hexapod to walk or turn based on EMG.

    Notes
    -----
    This function has the serial port to communicate to and the threshold
    of co-contraction to switch modes hardcoded.
    """
    port = connect(usb_port)  # connect to the servo controller
    conn = gamePadEstablishServer()
    # setup the starting robot positions
    stand(usb_port)
    body_model = bodyPos(pitch=0, roll=0, yaw=0, Tx=0, Ty=0, Tz=0,
                         body_offset=85)
    start_leg = startLegPos(body_model, start_radius=170, start_height=60)
    # get the serial message from the angles
    leg_model = legModel(start_leg, body_model)
    previous_walk_step = 0
    previous_walk_angle = 90
    previous_turn_angle = 0
    turn_feet_positions = getFeetPos(leg_model)
    right_foot = True

    previous_mode = 1

    max_walk_distance = 30
    max_turn_angle = 15
    max_body_shift = 15
    max_body_turn = 15
    start_down = True
    while mode != 5:
        [rs_x, rs_y, rs_t, ls_x, ls_y, ls_t, a, b, y, x, down_up_d,
         right_left_d, rt, rb, lt, lb, back, start] = pollGamePad(conn)

        if b == 1:
            mode = 5
        elif start == 1:
            if mode != 4 and start_down:
                start_down = False
                previous_mode = mode
                mode = 4
                print("Sitting")
                sit(usb_port)
            elif start_down:
                start_down = False
                print("Back to mode ", previous_mode)
                stand(usb_port)
                mode = previous_mode
        elif rt > 0.5 and mode != 4:
            mode = 2
        elif lt > 0.5 and mode != 4:
            mode = 3
        elif not start_down:
            print("reset start button")
            start_down = True
        elif a == 1 and mode != 4:
            mode = 1

        match mode:
            case 1:  # walk/turn
                pitch = roll = Tx = Ty = 0
                if rb == 1:
                    [pitch, roll] = body_model =\
                        bodyAngle(analog_x=right_left_d, analog_y=-down_up_d,
                                  max_angle=max_body_turn)
                    body_model =\
                        bodyPos(pitch=pitch, roll=roll, yaw=0, Tx=0, Ty=0,
                                Tz=0, body_offset=85)
                else:
                    Tx = max_body_shift * right_left_d
                    Ty = max_body_shift * -down_up_d
                    body_model =\
                        bodyPos(pitch=0, roll=0, yaw=0, Tx=Tx,
                                Ty=Ty, Tz=0, body_offset=85)

                left_stick_mag = hypot(ls_x, ls_y)
                if left_stick_mag < 0.1:
                    left_stick_mag = 0

                right_stick_mag = hypot(rs_x, rs_y)
                if right_stick_mag < 0.1:
                    right_stick_mag = 0

                if left_stick_mag == 0 and right_stick_mag == 0:
                    continue

                walk_distance = left_stick_mag * max_walk_distance
                walk_angle = degrees(atan2(ls_y, ls_x))
                turn_angle =\
                    right_stick_mag * max_turn_angle / 2 * np.sign(rs_x)

                [turn_feet_positions, right_foot, previous_walk_step,
                 previous_walk_angle, previous_turn_angle, move_positions] =\
                    simultaneousWalkTurn(turn_feet_positions, right_foot,
                                         previous_walk_step,
                                         previous_walk_angle,
                                         previous_turn_angle, walk_distance,
                                         walk_angle, turn_angle)
                sendPositions(port, move_positions, body_model)
            case 2:
                [pitch, roll] = body_model =\
                    bodyAngle(analog_x=ls_x, analog_y=ls_y,
                              max_angle=max_body_turn)
                Tx = max_body_shift * rs_x
                Ty = max_body_shift * rs_y
                body_model =\
                    bodyPos(pitch=pitch, roll=roll, yaw=0, Tx=Tx,
                            Ty=Ty, Tz=0, body_offset=85)
                start_leg = startLegPos(body_model, start_radius=170,
                                        start_height=60)
                # get the serial message from the angles
                message = anglesToSerial(start_leg, 500, 2000)
                sendData(port, message)  # send the serial message

            case 3:
                yaw = hypot(ls_x, ls_y) * max_body_turn * np.sign(ls_x)
                Tz = hypot(rs_x, rs_y) * max_body_shift * np.sign(rs_y)
                body_model =\
                    bodyPos(pitch=0, roll=0, yaw=yaw, Tx=0,
                            Ty=0, Tz=Tz, body_offset=85)
                start_leg = startLegPos(body_model, start_radius=170,
                                        start_height=60)
                # get the serial message from the angles
                message = anglesToSerial(start_leg, 500, 2000)
                sendData(port, message)  # send the serial message

            case 4:  # Pause control
                continue
            case 5:
                print("Ending Control")
                break
            case _:  # default case
                print("This option was not initiated.")
                print("Switching base to walk/turn mode.")
                mode = 1


def sendPositions(port: Any, positions: np.ndarray,
                  body_model: np.ndarray) -> bool:
    """
    Send each position in a set to the servo controller.

    Takes a list of command positions and iterates through them to send each
    position as a servo command to the Lynxmotion SSC32U.

    Parameters
    ----------
    port: Serial Port
        The USB port that the servo signals are sent over
    positions: np.ndarray
        A numpy array of a set of foot positions to command the hexapod to.
        These foot positions are a 6x3 numpy array of x, y, z positions for
        the six legs and the number of 6x3 arrays in the positions matrix is
        determined by how far the hexapod walked or turned.
    body_model: np.ndarray
        The 7x3 numpy array containing the locations of the coax servos.

    Returns
    -------
    bool:
        The function returns true when finished.

    Notes
    -----
    In this function, the ssc32uDriver.anglesToSerial function does not have
    its speed or time inputs used as each change in position is so small, that
    using these parameters is unneeded. Also note that there is a 5ms delay
    between commands.
    """
    for position in positions:
        # convert the feet positions to angles
        angles = recalculateLegAngles(position, body_model)
        # get the serial message from the angles
        message = anglesToSerial(angles, 2000)
        sendData(port, message)  # send the serial message
    return True


def sit(usb_port: string) -> None:
    """
    Tells the Hexapod to sit with its body on the ground.

    Recreates the neutral hexapod positions, but with a lowered height so that
    the body of the hexapod touches the ground. This is the position that the
    hexapod should be turned off in.

    Parameters
    ----------
    usb_port: string
        The name of the port to connect to
    """
    port = connect(usb_port)  # connect to the servo controller
    body_model = bodyPos(pitch=0, roll=0, yaw=0, Tx=0, Ty=0, Tz=0,
                         body_offset=85)
    sit_leg = startLegPos(body_model, start_radius=120, start_height=10)
    # get the serial message from the angles
    message = anglesToSerial(sit_leg, 500, 2000)
    sendData(port, message)  # send the serial message


def stand(usb_port: string) -> None:
    """
    Tells the hexapod to stand in the neutral position.

    Recreates the neutral hexapod positions that occur at the beginning
    of the controller function.

    Parameters
    ----------
    usb_port: string
        The name of the port to connect to

    See Also
    --------
    emgController:
        Controls the hexapod to walk or turn based on EMG.

    Notes
    -----
    This function assumes that there is already a connection to the serial
    port for the Lynxmotion SSC32U.
    """
    # controls the hexapod to walk or turn and send the commands
    port = connect(usb_port)  # connect to the servo controller
    # setup the starting robot positions
    body_model = bodyPos(pitch=0, roll=0, yaw=0, Tx=0, Ty=0, Tz=0,
                         body_offset=85)
    start_leg = startLegPos(body_model, start_radius=170, start_height=10)
    # get the serial message from the angles
    message = anglesToSerial(start_leg, 500, 2000)
    sendData(port, message)  # send the serial message
    sleep(2)
    start_leg = startLegPos(body_model, start_radius=170, start_height=60)
    # get the serial message from the angles
    message = anglesToSerial(start_leg, 500, 2000)
    sendData(port, message)  # send the serial message


def turnCycle(usb_port: string, turn_angle: float) -> None:
    """
    Tells the hexapod to turn to an angle without EMG

    This function just uses the move.turn function and sends its positions to
    the Lynxmotion SSC32U.

    Parameters
    ----------
    usb_port: string
        The name of the port to connect to
    turn_angle: float, default=60
        The angle to turn to. A positive angle if a left turn.

    See Also
    --------
    hexapod.move.turn
    """
    port = connect(usb_port)  # connect to the servo controller
    # setup the starting robot positions
    body_model = bodyPos(pitch=0, roll=0, yaw=0, Tx=0, Ty=0, Tz=0,
                         body_offset=85)
    start_leg = startLegPos(body_model, start_radius=170, start_height=60)
    # get the serial message from the angles
    message = anglesToSerial(start_leg, 500, 2000)
    sendData(port, message)  # send the serial message
    leg_model = legModel(start_leg, body_model)
    positions = turn(leg_model, turn_angle, 10)
    sendPositions(port, positions, body_model)


def walkCycle(usb_port: string, distance: float, angle: float) -> None:
    """
    Tells the hexapod to walk a specified distance without the need for EMG.

    This function just uses the move.walk function and sends its positions to
    the Lynxmotion SSC32U.

    Parameters
    ----------
    usb_port: string
        The name of the port to connect to
    distance: float
        The length in millimeters that the hexapod will walk. This distance is
        broken up into steps based on the max step size in move.walk
    angle: float
        The direction the hexapod will walk in. 90 degrees is forward.

    See Also
    --------
    hexapod.move.walk
    """
    # controls the hexapod to walk or turn and send the commands
    port = connect(usb_port)  # connect to the servo controller
    # setup the starting robot positions
    body_model = bodyPos(pitch=0, roll=0, yaw=0, Tx=0, Ty=0, Tz=0,
                         body_offset=85)
    start_leg = startLegPos(body_model, start_radius=170, start_height=60)
    # get the serial message from the angles
    message = anglesToSerial(start_leg, 500, 2000)
    sendData(port, message)  # send the serial message
    leg_model = legModel(start_leg, body_model)
    positions = walk(leg_model, distance, angle, 10)
    sendPositions(port, positions, body_model)
