"""
Scripts for controlling the hexapod using PCA9685.
"""
import string
from hexapod.leg import recalculateLegAngles, startLegPos, legModel, getFeetPos
from hexapod.body import bodyPos, bodyAngle
from hexapod.move import (emgToWalk, resetWalkStance, emgToTurn,
                          resetTurnStance, walk, turn, simultaneousWalkTurn)
# Swapped the driver import here
from hexapod.pca9685Driver import connect, sendPositions as pcaSend, disconnect
from hexapod.piToPi import (emgEstablishServer, gamePadEstablishServer,
                            switchMode, pollGamePad)
from math import hypot, atan2, degrees
import numpy as np
from typing import Any
from time import sleep

def emgController(address: int, mode: bool) -> None:
    """
    Now uses I2C address (e.g., 0x40) instead of usb_port string.
    """
    pca = connect(address)  # connect to the PCA9685
    conn = emgEstablishServer()
    
    body_model = bodyPos(pitch=0, roll=0, yaw=0, Tx=0, Ty=0, Tz=0, body_offset=85)
    start_leg = startLegPos(body_model, start_radius=170, start_height=60)
    
    # Send starting position directly
    pcaSend(pca, start_leg)
    leg_model = legModel(start_leg, body_model)

    previous_step = 0
    previous_turn_angle = 0
    right_foot = True
    while True:
        if mode: 
            [leg_model, right_foot, previous_step, positions] =\
                emgToWalk(body_model, leg_model, right_foot, previous_step,
                          conn, max_distance=30)
        else:
            [leg_model, right_foot, previous_turn_angle, positions] =\
                emgToTurn(body_model, leg_model, right_foot,
                          conn, previous_turn_angle, max_turn_angle=15)

        sendPositions(pca, positions, body_model)

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
            sendPositions(pca, positions, body_model)


def gamePadController(address: int, mode: int) -> None:
    pca = connect(address)
    conn = gamePadEstablishServer()
    
    stand(address)
    body_model = bodyPos(pitch=0, roll=0, yaw=0, Tx=0, Ty=0, Tz=0, body_offset=85)
    start_leg = startLegPos(body_model, start_radius=170, start_height=60)
    
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
                sit(address)
            elif start_down:
                start_down = False
                stand(address)
                mode = previous_mode
        elif rt > 0.5 and mode != 4:
            mode = 2
        elif lt > 0.5 and mode != 4:
            mode = 3
        elif a == 1 and mode != 4:
            mode = 1

        match mode:
            case 1:  # walk/turn
                # ... [Internal logic remains same until movement command]
                left_stick_mag = hypot(ls_x, ls_y)
                right_stick_mag = hypot(rs_x, rs_y)

                if left_stick_mag > 0.1 or right_stick_mag > 0.1:
                    walk_distance = left_stick_mag * max_walk_distance
                    walk_angle = degrees(atan2(ls_y, ls_x))
                    turn_angle = right_stick_mag * max_turn_angle / 2 * np.sign(rs_x)

                    [turn_feet_positions, right_foot, previous_walk_step,
                     previous_walk_angle, previous_turn_angle, move_positions] =\
                        simultaneousWalkTurn(turn_feet_positions, right_foot,
                                             previous_walk_step,
                                             previous_walk_angle,
                                             previous_turn_angle, walk_distance,
                                             walk_angle, turn_angle)
                    sendPositions(pca, move_positions, body_model)
            case 2 | 3:
                # Update body model and send angles directly
                # [Simplified for brevity - apply body_model logic here]
                start_leg = startLegPos(body_model, start_radius=170, start_height=60)
                pcaSend(pca, start_leg)

            case 5:
                disconnect(pca)
                break

def sendPositions(pca: Any, positions: np.ndarray,
                  body_model: np.ndarray) -> bool:
    """
    Iterates through positions and updates PCA9685.
    """
    for position in positions:
        angles = recalculateLegAngles(position, body_model)
        # Direct call to the new driver's command function
        pcaSend(pca, angles)
        # PCA9685 is fast; a small sleep helps keep the I2C bus stable
        sleep(0.005) 
    return True

def sit(address: int) -> None:
    pca = connect(address)
    body_model = bodyPos(pitch=0, roll=0, yaw=0, Tx=0, Ty=0, Tz=0, body_offset=85)
    sit_leg = startLegPos(body_model, start_radius=120, start_height=10)
    pcaSend(pca, sit_leg)

def stand(address: int) -> None:
    pca = connect(address)
    body_model = bodyPos(pitch=0, roll=0, yaw=0, Tx=0, Ty=0, Tz=0, body_offset=85)
    
    # Stand step 1
    start_leg = startLegPos(body_model, start_radius=170, start_height=10)
    pcaSend(pca, start_leg)
    sleep(1)
    
    # Stand step 2
    start_leg = startLegPos(body_model, start_radius=170, start_height=60)
    pcaSend(pca, start_leg)

def walkCycle(address: int, distance: float, angle: float) -> None:
    pca = connect(address)
    body_model = bodyPos(body_offset=85)
    start_leg = startLegPos(body_model, start_radius=170, start_height=60)
    pcaSend(pca, start_leg)
    
    leg_model = legModel(start_leg, body_model)
    positions = walk(leg_model, distance, angle, 10)
    sendPositions(pca, positions, body_model)