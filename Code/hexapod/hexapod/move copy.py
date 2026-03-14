"""
Functions to calculate linear and angular movement for the hexapod.

These are a collection of functions to find how the hexapod will take steps to
move linearly in any direction aor to turn itself in the x-y plane about the
z axis.

Functions
---------
emgToTurn:
    Turns a dynamic angle based on a normalized EMG input.
emgToWalk:
    Walks a dynamic distance based a normalized EMG input.
omniWalk:
    Walks in any direction based on the previous step.
resetStance:
    Completes the final step in simultaneous turning and walking.
resetTurnStance:
    Completes the final step in turning to a neutral stance.
resetWalkStance:
    Completes the final step in walking to a neutral stance.
simultaneousWalkTurn:
    Makes a step that allows both a turn and a walk in any direction.
stepForward:
    Calculate the x, y, z position updates to move in a step in a direction.
stepTurn:
    Calculate the positions of each foot when turning about an angle.
stepTurnFoot:
    Calculate the position of a foot when turning the hexapod about an angle.
turn:
    Creates the series of feet positions to turn the hexapod about the z axis.
walk:
    Creates a series of feet positions to use when walking in a direction.
"""
from math import degrees, radians, sin, cos, atan2, hypot
import numpy as np
from hexapod.leg import getFeetPos, recalculateLegAngles, legModel
from hexapod.piToPi import pollEMG
import socket
from typing import Tuple


def emgToTurn(body_model: np.ndarray, leg_model: np.ndarray, right_foot: bool,
              previous_turn_angle: float, conn: socket,
              max_turn_angle: float = 15) -> Tuple[np.ndarray, bool, float,
                                                   np.ndarray]:
    """
    Turns a dynamic angle based on a normalized EMG input.

    Takes two forearm EMG signals and turns an angle determined by how large
    the difference between the two EMG signals are. The angle to turn is
    determined by a percentage of the `max_turn_angle` scaled by the difference
    in EMG signals.

    Parameters
    ----------
    body_model: np.ndarray
        The 7x3 numpy array containing the locations of the coax servos.
    leg_model: np.ndarray
        the 4x3x6 numpy array that holds the locations of the coax, femur,
        and tibia servos as well as the feet end positions.
    right_foot: bool
        An indicator if the right or left set of legs are taking the step.
        The "right" set are legs 0, 2, and 4 and the "left" are 1, 3, and 5.
    previous_turn_angle: float
        The angle the hexapod turned the last time this function was called.
    conn: socket
        The server socket EMG data is send to
    max_turn_angle: float, default=15
        The angle that the EMG difference scales to. The higher the
        difference, the closer to the `max_turn_angle` the hexapod will turn.

     Returns
    -------
    [leg_model, right_foot, previous_turn_angle, turn_positions]:
        Tuple[np.ndarray, bool, float, np.ndarray]
        The updated input parameters for the next time the function is run as
        well as the positions to move to.

    See Also
    --------
    turn:
        Creates the series of feet positions to turn the hexapod about the z
        axis.
    resetTurnStance:
        Completes the final step in turning to a neutral stance.
    hexapod.leg.recalculateLegAngles:
        Finds the coax, femur, and tibia angles of each leg.

    Notes
    -----
    When the difference in EMG is positive, the hexapod turns left. When it is
    negative, it turns right. The higher the `max_turn_angle` parameter is,
    the farther a change in EMG will turn the hexapod.
    """
    # call a function to poll for forearm emg values from the raspberry pi zero
    [fcr_emg, edc_emg] = pollEMG(conn)
    # finds the difference between EMG signals to move right or left
    emg = fcr_emg - edc_emg

    turn_angle = round(max_turn_angle * emg)
    feet_positions = getFeetPos(leg_model)

    turn_positions =\
        stepTurn(feet_positions,
                 step_angle=np.sign(turn_angle) * (abs(turn_angle)
                                                   + previous_turn_angle),
                 right_foot=right_foot)

    previous_turn_angle = turn_angle
    right_foot = not right_foot
    leg_model = legModel(recalculateLegAngles(turn_positions[-1, :, :],
                                              body_model), body_model)

    return (leg_model, right_foot, previous_turn_angle, turn_positions)


def emgToWalk(body_model: np.ndarray, leg_model: np.ndarray, right_foot: bool,
              previous_step: float, conn: socket,
              max_distance: float = 30) -> Tuple[np.ndarray, bool, float,
                                                 np.ndarray]:
    """
    Walks a dynamic distance based a normalized EMG input.

    Takes two forearm EMG signals and walks forward a distance determined
    by how large the difference between the two EMG signals are. The distance
    to move is determined by a percentage of the `max_step_size` scaled by the
    difference in EMG signals.

    Parameters
    ----------
    body_model: np.ndarray
        The 7x3 numpy array containing the locations of the coax servos.
    leg_model: np.ndarray
        the 4x3x6 numpy array that holds the locations of the coax, femur,
        and tibia servos as well as the feet end positions.
    right_foot: bool
        An indicator if the right or left set of legs are taking the step.
        The "right" set are legs 0, 2, and 4 and the "left" are 1, 3, and 5.
    previous_step: float
        The distance the hexapod walked the last time this function was called.
    conn: socket
        The server socket EMG data is send to
    max_distance: float, default=30
        The distance that the EMG difference scales to. The higher the
        difference, the closer to the `max_distance` the hexapod will walk.

    Returns
    -------
    [leg_model, right_foot, previous_step, walk_positions]:
        Tuple[np.ndarray, bool, float, np.ndarray]

        The updated input parameters for the next time the function is run as
        well as the positions to move to.

    See Also
    --------
    walk:
        Creates a series of feet positions to use when walking in a direction.
    resetWalkStance:
        Completes the final step in walking to a neutral stance.
    hexapod.leg.recalculateLegAngles:
        Finds the coax, femur, and tibia angles of each leg.

    Notes
    -----
    When the difference in EMG is positive, the hexapod walks forwards. When it
    is negative, it walks backwards. The higher the `max_distance` parameter
    is, the farther a change in EMG will move the hexapod.
    """
    # call a function to poll for forearm emg values from the raspberry pi zero
    [fcr_emg, edc_emg] = pollEMG(conn)
    # finds the difference between EMG signals to move forward or backwards
    emg = fcr_emg - edc_emg

    # find a integer distance to move that is a percentage of the max distance.
    distance = round(max_distance * emg)
    walk_positions = stepForward(step_angle=90,
                                 distance=distance + previous_step,
                                 right_foot=right_foot)
    feet_positions = getFeetPos(leg_model)
    # add all of the feet positions to the walk
    for i in range(walk_positions.shape[0]):
        walk_positions[i, :, :] = walk_positions[i, :, :] + feet_positions

    previous_step = distance
    right_foot = not right_foot
    leg_model = legModel(recalculateLegAngles(walk_positions[-1, :, :],
                                              body_model), body_model)
    return (leg_model, right_foot, previous_step, walk_positions)


def omniWalk(body_model: np.ndarray, leg_model: np.ndarray, right_foot: bool,
             previous_step: float = 0, previous_angle: float = 0,
             distance: float = 30,
             angle: float = 90) -> Tuple[np.ndarray, bool, float, float,
                                         np.ndarray]:
    """
    Walks in any direction based on the previous step.

    Walks in much the same way as the regular walk function and the emgToWalk
    function, but this function can walk in any direction between each step.
    The new step direction is found using the previous step size and angle.

    Parameters
    ----------
    body_model: np.ndarray
        The 7x3 numpy array containing the locations of the coax servos.
    leg_model: np.ndarray
        the 4x3x6 numpy array that holds the locations of the coax, femur,
        and tibia servos as well as the feet end positions.
    right_foot: bool
        An indicator if the right or left set of legs are taking the step.
        The "right" set are legs 0, 2, and 4 and the "left" are 1, 3, and 5.
    previous_step: float, default=0
        The distance the hexapod walked the last time this function was called.
    previous_angle: float, default=0
        The angle the hexapod walked the last time this function was called.
    distance: float, default=30
        The step size of the current step.
    angle: float, default=90
        The angle that the hexapod will walk in.

    Returns
    -------
    [leg_model, right_foot, previous_step, previous angle, walk_positions]:
        Tuple[np.ndarray, bool, float, float, np.ndarray]

        The updated input parameters for the next time the function is run as
        well as the positions to move to.

    See Also
    --------
    walk:
        Creates a series of feet positions to use when walking in a direction.
    resetWalkStance:
        Completes the final step in walking to a neutral stance.
    emgToWalk:
        Walks a dynamic distance based a normalized EMG input.
    hexapod.leg.recalculateLegAngles:
        Finds the coax, femur, and tibia angles of each leg.

    Notes
    -----
    This function uses the vector addition of the previous step size along its
    angle with the step size along the new angle to find the new resultant
    step size and angle. This is to reset the hexapod's stance while also
    moving in the new direction.
    """
    # components of previous step
    previous_x = previous_step * cos(radians(previous_angle))
    previous_y = previous_step * sin(radians(previous_angle))
    # components of desired step
    current_x = distance * cos(radians(angle))
    current_y = distance * sin(radians(angle))
    # combined step
    x = previous_x + current_x
    y = previous_y + current_y
    step_magnitude = hypot(x, y)
    step_angle = degrees(atan2(y, x))
    walk_positions = stepForward(step_angle=step_angle,
                                 distance=step_magnitude,
                                 right_foot=right_foot)
    feet_positions = getFeetPos(leg_model)
    # add all of the feet positions to the walk
    for i in range(walk_positions.shape[0]):
        walk_positions[i, :, :] = walk_positions[i, :, :] + feet_positions

    previous_step = distance
    previous_angle = angle
    right_foot = not right_foot
    leg_model = legModel(recalculateLegAngles(walk_positions[-1, :, :],
                                              body_model), body_model)
    return (leg_model, right_foot, previous_step, previous_angle,
            walk_positions)


def resetStance(body_model: np.ndarray, leg_model: np.ndarray,
                right_foot: bool, previous_walk_step: float = 0,
                previous_walk_angle: float = 0,
                previous_turn_angle: float = 0) -> Tuple[np.ndarray, bool,
                                                         np.ndarray]:
    """
    Completes the final step in simultaneous turning and walking.

    Takes the final step of the simultaneous turn and walk cycle by repeating
    the previous step with the opposite legs as the last step.

    Parameters
    ----------
    body_model: np.ndarray
        The 7x3 numpy array containing the locations of the coax servos.
    leg_model: np.ndarray
        the 4x3x6 numpy array that holds the locations of the coax, femur,
        and tibia servos as well as the feet end positions.
    right_foot: bool
        An indicator if the right or left set of legs are taking the step.
        The "right" set are legs 0, 2, and 4 and the "left" are 1, 3, and 5.
    previous_walk_step: float, default=0
        The distance the hexapod walked the last time this function was called.
    previous_walk_angle: float, default=0
        The angle the hexapod walked the last time this function was called.
    previous_turn_angle: float
        The angle the hexapod turned the last time this function was called.

    Returns
    -------
    [leg_model, right_foot, move_positions]: Tuple[np.ndarray, bool,
                                                   np.ndarray]
        The updated `leg_model`, `right_foot`, and `turn_positions` parameters.
        The first two allow the hexapod to switch to the walking phase with an
        updated model and smoothly move the correct set of legs on the next
        step, while the `turn_positions` numpy array is used to take the final
        step when the positions are converted to servo angles.

    See Also
    --------
    simultaneousWalkTurn:
        Makes a step that allows both a turn and a walk in any direction
    hexapod.leg.recalculateLegAngles:
        Finds the coax, femur, and tibia angles of each leg.

    Notes
    -----
    This function is only run when the simultaneous walking and turning
    function is paused
    """
    feet_positions = getFeetPos(leg_model)
    turn_positions = stepTurn(feet_positions,
                              step_angle=previous_turn_angle,
                              right_foot=right_foot)

    walk_positions = stepForward(step_angle=previous_walk_angle,
                                 distance=previous_walk_step,
                                 right_foot=right_foot)

    move_positions = turn_positions + walk_positions
    leg_model = legModel(recalculateLegAngles(move_positions[-1, :, :],
                                              body_model), body_model)
    right_foot = not right_foot
    return (leg_model, right_foot, move_positions)


def resetTurnStance(body_model: np.ndarray, leg_model: np.ndarray,
                    right_foot: bool,
                    previous_turn_angle: float) -> Tuple[np.ndarray, bool,
                                                         np.ndarray]:
    """
    Completes the final step in turning to a neutral stance.

    Takes the final step of the turn cycle by repeating the previous step
    with the opposite legs as the last step.

    Parameters
    ----------
    body_model: np.ndarray
        The 7x3 numpy array containing the locations of the coax servos.
    leg_model: np.ndarray
        the 4x3x6 numpy array that holds the locations of the coax, femur,
        and tibia servos as well as the feet end positions.
    right_foot: bool
        An indicator if the right or left set of legs are taking the step.
        The "right" set are legs 0, 2, and 4 and the "left" are 1, 3, and 5.
    previous_turn_angle: float
        The angle the hexapod turned the last time this function was called.

    Returns
    -------
    [leg_model, right_foot, turn_positions]: Tuple[np.ndarray, bool,
                                                   np.ndarray]
        The updated `leg_model`, `right_foot`, and `turn_positions` parameters.
        The first two allow the hexapod to switch to the walking phase with an
        updated model and smoothly move the correct set of legs on the next
        step, while the `turn_positions` numpy array is used to take the final
        step when the positions are converted to servo angles.

    See Also
    --------
    turn:
        Creates the series of feet positions to turn the hexapod about the z
        axis.
    hexapod.leg.recalculateLegAngles:
        Finds the coax, femur, and tibia angles of each leg.

    Notes
    -----
    This function is only run when switching to the walking phase of movement
    The hexapod will also reset its stance during the turn if the user does not
    contract at all.
    """
    feet_positions = getFeetPos(leg_model)

    turn_positions = stepTurn(feet_positions,
                              step_angle=previous_turn_angle,
                              right_foot=right_foot)

    right_foot = not right_foot
    leg_model = legModel(recalculateLegAngles(turn_positions[-1, :, :],
                                              body_model), body_model)

    return (leg_model, right_foot, turn_positions)


def resetWalkStance(body_model: np.ndarray, leg_model: np.ndarray,
                    right_foot: bool, previous_step: float,
                    previous_angle: float) -> Tuple[np.ndarray, bool,
                                                    np.ndarray]:
    """
    Completes the final step in walking to a neutral stance.

    Takes the final step of the walk cycle by repeating the previous step
    with the opposite legs as the last step.

    Parameters
    ----------
    body_model: np.ndarray
        The 7x3 numpy array containing the locations of the coax servos.
    leg_model: np.ndarray
        the 4x3x6 numpy array that holds the locations of the coax, femur,
        and tibia servos as well as the feet end positions.
    right_foot: bool
        An indicator if the right or left set of legs are taking the step.
        The "right" set are legs 0, 2, and 4 and the "left" are 1, 3, and 5.
    previous_step: float
        The distance the hexapod walked in its last step.
    previous_angle: float
        The angle the hexapod walked in its last step.

    Returns
    -------
    [leg_model, right_foot, walk_positions]: Tuple[np.ndarray, bool,
                                                   np.ndarray]
        The updated `leg_model`, `right_foot`, and `walk_positions` parameters.
        The first two allow the hexapod to switch to the turning phase with an
        updated model and smoothly move the correct set of legs on the next
        step, while the `walk_positions` numpy array is used to take the final
        step when the positions are converted to servo angles.

    See Also
    --------
    walk:
        Creates a series of feet positions to use when walking in a direction.
    hexapod.leg.recalculateLegAngles:
        Finds the coax, femur, and tibia angles of each leg.

    Notes
    -----
    This function is only run when switching to the turning phase of movement.
    The hexapod will also reset its stance during the walk if the user does not
    contract at all.
    """
    walk_positions = stepForward(step_angle=previous_angle,
                                 distance=previous_step,
                                 right_foot=right_foot)
    feet_positions = getFeetPos(leg_model)
    # add all of the feet positions to the walk
    for i in range(walk_positions.shape[0]):
        walk_positions[i, :, :] = walk_positions[i, :, :] + feet_positions

    leg_model = legModel(recalculateLegAngles(walk_positions[-1, :, :],
                                              body_model), body_model)
    right_foot = not right_foot
    return (leg_model, right_foot, walk_positions)


def simultaneousWalkTurn(turn_feet_positions: np.ndarray,
                         right_foot: bool, previous_walk_step: float = 0,
                         previous_walk_angle: float = 0,
                         previous_turn_angle: float = 0,
                         walk_distance: float = 30,
                         walk_angle: float = 90,
                         turn_angle: float = 15) -> Tuple[np.ndarray, bool,
                                                          float, float, float,
                                                          np.ndarray]:
    """
    Makes a step that allows both a turn and a walk in any direction.

    Allows the hexapod to walk and/or turn at the same time by finding the
    feet positions to turn the hexapod and then applying the translation from
    walking.

    Parameters
    ----------
    turn_feet_positions: np.ndarray
        The 6x3 numpy array containing the locations of the feet after a turn.
    right_foot: bool
        An indicator if the right or left set of legs are taking the step.
        The "right" set are legs 0, 2, and 4 and the "left" are 1, 3, and 5.
    previous_walk_step: float, default=0
        The distance the hexapod walked the last time this function was called.
    previous_walk_angle: float, default=0
        The angle the hexapod walked the last time this function was called.
    previous_turn_angle: float
        The angle the hexapod turned the last time this function was called.
    walk_distance: float, default=30
        The step size of the current step.
    walk_angle: float, default=90
        The angle that the hexapod will walk in.
    turn_angle: float, default=15
        The angle that the hexapod will turn to.

    Returns
    -------
    [turn_feet_positions, right_foot, previous_walk_step, previous_walk_angle,
     previous_turn_angle, move_positions]:
        Tuple[np.ndarray, bool, float, float, float, np.ndarray]

        The updated input parameters for the next time the function is run as
        well as the positions to move to.

    See Also
    --------
    emgToWalk:
        Walks a dynamic distance based a normalized EMG input.
    emgToTurn:
        Turns a dynamic angle based on a normalized EMG input.
    omniWalk:
        Walks in any direction based on the previous step.

    Notes
    -----
    The walking code is the same as the omniWalk code. Simultaneous walking
    and turning is done by having the rotation of the hexapod found first
    before the translation of the hexapod. This is the same concept as done
    when applying a rotation and translation matrix to a point.
    """
    turn_positions =\
        stepTurn(turn_feet_positions,
                 step_angle=np.sign(turn_angle) * (abs(turn_angle)
                                                   + previous_turn_angle),
                 right_foot=right_foot)

    turn_feet_positions = turn_positions[-1, :, :]

    # components of previous step
    previous_x = previous_walk_step * cos(radians(previous_walk_angle))
    previous_y = previous_walk_step * sin(radians(previous_walk_angle))
    # components of desired step
    current_x = walk_distance * cos(radians(walk_angle))
    current_y = walk_distance * sin(radians(walk_angle))
    # combined step
    x = previous_x + current_x
    y = previous_y + current_y
    step_magnitude = hypot(x, y)
    step_angle = degrees(atan2(y, x))
    walk_positions = stepForward(step_angle=step_angle,
                                 distance=step_magnitude,
                                 right_foot=right_foot)

    move_positions = turn_positions + walk_positions

    previous_walk_step = walk_distance
    previous_walk_angle = walk_angle
    previous_turn_angle = turn_angle
    right_foot = not right_foot

    return (turn_feet_positions, right_foot, previous_walk_step,   
            previous_walk_angle, previous_turn_angle, move_positions)


def stepForward(step_angle: float = 90, distance: float = 30,
                step_height: float = 15, right_foot: bool = True,
                z_resolution: float = 10) -> np.ndarray:
    """
    Calculate the x, y, z position updates to move in a step in a direction.

    Finds the relative positions to move the hexapod's six legs to when taking
    a step. The resolution of each sub-movement is determined by the
    resolution of the upwards sub-movements and the steps taken are parabolic.

    Parameters
    ----------
    step_angle: float, default=90
        The direction the step is taken in in degrees. 90 degrees is forward.
    distance: float, default=30
        How far forward the step will be in millimeters.
    step_height: float, default=15
        How high the step will be in millimeters.
    right_foot: bool, default=True
        An indicator if the right or left set of legs are taking the step.
        The "right" set are legs 0, 2, and 4 and the "left" are 1, 3, and 5.
    z_resolution: float, default=10
        the size in mm of the upwards sub-movements.

    Returns
    -------
    feet: np.ndarray
        The 6x3 numpy array that holds the set of relative positions changes
        to take from where the feet were when taking each part of the step.

    Notes
    -----
    The number of sub-movements is determined by the resolution in the z
    direction. If you want fewer sub-movements, raise the `z_resolution`
    number that is hardcoded in the function.
    """
    z = np.array([-(i ** 2) / 4 + ((step_height) ** 2) / 4
                  for i in np.arange(-step_height, step_height + z_resolution,
                  z_resolution)])
    x = np.linspace(0, distance * cos(radians(step_angle)), z.size)
    y = np.linspace(0, distance * sin(radians(step_angle)), z.size)
    lead_foot = np.dstack((x, y, z)).reshape(z.size, 1, 3)
    dragging_foot =\
        np.dstack((- x, - y, np.zeros(z.size))).reshape(z.size, 1, 3)

    # legs 0, 2, and 4 are the right legs and legs 1, 3, 5 as the left legs
    if right_foot:  # right foot
        feet = np.concatenate((lead_foot, dragging_foot, lead_foot,
                               dragging_foot, lead_foot, dragging_foot),
                              axis=1)
    else:
        feet = np.concatenate((dragging_foot, lead_foot, dragging_foot,
                               lead_foot, dragging_foot, lead_foot), axis=1)

    return feet


def stepTurn(feet_pos: np.ndarray, step_angle: float = 15,
             step_height: float = 15, right_foot: bool = True,
             z_resolution: float = 10) -> np.ndarray:
    """
    Calculate the positions of each foot when turning about an angle.

    Finds the absolute positions of each leg of the hexapod using the
    stepTurnFoot function to find the relative additions to the current
    feet positions.

    Parameters
    ----------
    feet_pos: np.ndarray
        The 6x3 numpy array of the x, y, z, positions of each foot of the
        hexapod. This array can be found the leg.getFeetPos function.
    step_angle: float, default=15
        The angle to move make the step to. A positive angle is a left step.
    step_height: float, default=15
        How high the step will be in millimeters.
    right_foot: bool, default=True
        An indicator if the right or left set of legs are taking the step.
        The "right" set are legs 0, 2, and 4 and the "left" are 1, 3, and 5.
    z_resolution: float, default=10
        the size in mm of the upwards sub-movements.

    Returns
    -------
    previous_foot: np.ndarray
        The 6x3 numpy array that holds the set of positions to take during the
        turning step.

    See Also
    --------
    stepTurnFoot:
        Calculate the position of a foot when turning the hexapod about an
        angle.
    hexapod.leg.getFeetPos:
        Output the x, y, z position of the feet of the hexapod.

    Notes
    -----
    This function outputs the absolute positions of the feet instead of the
    relative positions to add to the feet's current positions.
    """
    for i in range(6):
        footstep = stepTurnFoot(foot_x=feet_pos[i, 0],
                                foot_y=feet_pos[i, 1],
                                foot_z=feet_pos[i, 2],
                                step_angle=step_angle,
                                step_height=step_height,
                                right_foot=right_foot,
                                z_resolution=z_resolution)
        right_foot = not right_foot
        if i == 0:
            previous_foot = footstep
        else:
            previous_foot = np.concatenate((previous_foot, footstep), axis=1)

    return previous_foot


def stepTurnFoot(foot_x: float, foot_y: float, foot_z: float,
                 step_angle: float = 15, step_height: float = 15,
                 right_foot: bool = True,
                 z_resolution: float = 10) -> np.ndarray:
    """
    Calculate the position of a foot when turning the hexapod about an angle.

    Finds the relative position changes as sub-movements for one foot in turning
    the hexapod. This function is used in conjunction with the stepTurn
    function to turn the hexapod.

    Parameters
    ----------
    foot_x: float
        The x value of the leg's end position.
    foot_y: float
        The y value of the leg's end position.
    foot_z: float
        The z value of the leg's end position.
    step_angle: float, default=15
        The angle to move make the step to. A positive angle is a left step.
    step_height: float, default=15
        How high the step will be in millimeters.
    right_foot: bool, default=True
        An indicator if the foot is a part of the right or left set of legs
        taking the step. The "right" set are legs 0, 2, and 4 and the "left"
        are 1, 3, and 5.
    z_resolution: float, default=10
        the size in mm of the upwards sub-movements.

    Returns
    -------
    np.ndarray
        The 1x3 numpy array that holds the absolute position of the given foot
        when taking the turning step.

    See Also
    --------
    stepTurn:
        Calculate the positions of each foot when turning about an angle.

    Notes
    -----
    This function finds the absolute position changes for one foot instead
    of all of them like in the stepForward function. It is also an absolute
    position and not a relative position change.
    """
    radius = hypot(foot_x, foot_y)
    foot_angle = degrees(atan2(foot_y, foot_x))

    z = np.array([-(i ** 2) / 4 + ((step_height) ** 2) / 4 + foot_z
                  for i in np.arange(-step_height, step_height + z_resolution,
                  z_resolution)])
    x = np.empty(z.size)
    y = np.empty(z.size)
    angles = np.linspace(foot_angle, foot_angle + step_angle, z.size)
    for i, angle in enumerate(angles):
        x[i] = radius * cos(radians(angle))
        y[i] = radius * sin(radians(angle))

    # legs 0, 2, and 4 are the right legs and legs 1, 3, 5 as the left legs
    if right_foot:  # right foot
        angles = np.linspace(foot_angle, foot_angle + step_angle, z.size)
    else:
        angles = np.linspace(foot_angle, foot_angle - step_angle, z.size)
        z = np.zeros(z.size) + foot_z

    for i, angle in enumerate(angles):
        x[i] = radius * cos(radians(angle))
        y[i] = radius * sin(radians(angle))

    return np.dstack((x, y, z)).reshape(z.size, 1, 3)


def turn(leg_model: np.ndarray, turn_angle: float = 60,
         z_resolution: float = 10) -> np.ndarray:
    """
    Creates the series of feet positions to turn the hexapod about the z axis.

    Takes a set angle to turn and breaks that movement up into steps based
    on a max turn angle inside the function. Each step is then input into the
    stepTurn function with the set of legs that takes the step alternating.
    The final step puts the hexapod back in its neutral position.

    Parameters
    ----------
    leg_model: np.ndarray
        the 4x3x6 numpy array that holds the locations of the coax, femur,
        and tibia servos as well as the feet end positions.
    turn_angle: float, default=60
        The angle to turn to. A positive angle if a left turn.
    z_resolution: float, default=10
        the size in mm of the upwards sub-movements.

    Returns
    -------
    turn_positions: np.ndarray
        An nx6x3 array of the absolute foot positions to take during all of the
        sub-movements during the turn. These positions are used to calculate
        the servo angles to take in the turn.

    Raises
    ------
    ValueError
        If `turn_angle` is zero.

    See Also
    --------
    hexapod.leg.recalculateLegAngles:
        Finds the coax, femur, and tibia angles of each leg.
    """
    max_turn_angle = 15  # sets the maximum angle to turn by.
    # Raise an error is the robot is not commanded to move a non zero angle
    if turn_angle == 0:
        raise ValueError("turn angle must be a number larger than 0.")
    steps = int(abs(turn_angle / max_turn_angle))
    if abs(turn_angle % max_turn_angle) > 0:
        steps += 1

    right_foot = True  # If the right foot is moving forward
    # Sets the remaining turn distance to the full turn
    remaining_turn_distance = turn_angle
    for i in range(steps):  # iterate over the number of steps to take
        # if the remaining turn distance to move is less than the max turn
        # angle, then turn the robot the remaining angle
        if abs(remaining_turn_distance) <= max_turn_angle * 2:
            # Get the current feet positions from the last stepped position
            if 'turn_positions' in locals():
                feet_positions = turn_positions[-1, :, :]
            # if there was not a step yet get feet positions from the leg model
            else:
                feet_positions = getFeetPos(leg_model)

            # if this is the only step then the feet only need to turn the
            # robot the remaining turn distance
            if steps == 1:
                temp_turn_positions =\
                    stepTurn(feet_positions,
                             step_angle=remaining_turn_distance,
                             right_foot=right_foot,
                             z_resolution=z_resolution)
            # if this is not the first step the robot needs to move forward
            # the max turn size first to bring the robot to a neutral position
            # before moving the rest of the distance.
            else:
                temp_turn_positions =\
                    stepTurn(feet_positions,
                             step_angle=np.sign(remaining_turn_distance)
                             * abs(remaining_turn_distance),
                             right_foot=right_foot,
                             z_resolution=z_resolution)

            # try to add the next step to the walk
            if 'turn_positions' in locals():
                turn_positions =\
                    np.concatenate((turn_positions,
                                    temp_turn_positions),
                                   axis=0)
            # if this is the first step, create walk array with the first step
            else:
                turn_positions = temp_turn_positions
            remaining_turn_distance -=\
                np.sign(remaining_turn_distance) * max_turn_angle
            right_foot = not right_foot  # switch which foot steps forward
            break

        # if this is the first of more than one step, turn the max angle
        if i == 0:
            feet_positions = getFeetPos(leg_model)
            turn_positions =\
                stepTurn(feet_positions,
                         step_angle=np.sign(remaining_turn_distance)
                         * max_turn_angle,
                         right_foot=right_foot,
                         z_resolution=z_resolution)
            # reduce the remaining distance by the max step size
            remaining_turn_distance -=\
                np.sign(remaining_turn_distance) * max_turn_angle
            right_foot = not right_foot
        # if this is not the first step and the next step is more than the max
        # angle, move the legs forward by twice the max angle to reset and
        # then turn the max distance
        else:
            feet_positions = turn_positions[-1, :, :]

            temp_turn_positions =\
                stepTurn(feet_positions,
                         step_angle=np.sign(remaining_turn_distance)
                         * max_turn_angle * 2,
                         right_foot=right_foot,
                         z_resolution=z_resolution)

            turn_positions = np.concatenate((turn_positions,
                                             temp_turn_positions),
                                            axis=0)
            remaining_turn_distance -=\
                np.sign(remaining_turn_distance) * max_turn_angle * 2
            right_foot = not right_foot

    feet_positions = turn_positions[-1, :, :]
    # reset the position of the robot by moving the last step distance
    temp_turn_positions =\
        stepTurn(feet_positions,
                 step_angle=remaining_turn_distance,
                 right_foot=right_foot,
                 z_resolution=z_resolution)

    turn_positions = np.concatenate((turn_positions,
                                     temp_turn_positions),
                                    axis=0)
    return turn_positions


def walk(leg_model: np.ndarray, distance: float = 30,
         angle: float = 90, z_resolution: float = 10) -> np.ndarray:
    """
    Creates a series of feet positions to use when walking in a direction.

    Takes a set distance to move and breaks that movement up into steps based
    on a max step size inside the function. Each step is then input into the
    stepForward function with the set of legs that takes the step alternating.
    The final step puts the hexapod back in its neutral position.

    Parameters
    ----------
    leg_model: np.ndarray
        the 4x3x6 numpy array that holds the locations of the coax, femur,
        and tibia servos as well as the feet end positions.
    distance: float, default=30
        The linear distance to move in millimeters.
    angle: float, default=90
        The direction the step is taken in in degrees. 90 degrees is forward.
    z_resolution: float, default=10
        the size in mm of the upwards sub-movements.

    Returns
    -------
    walk_positions: np.ndarray
        An nx6x3 array of the absolute foot positions to take during all of the
        sub-movements during the walk. These positions are used to calculate
        the servo angles to take in the walk.

    Raises
    ------
    ValueError
        If `distance` is zero or negative.

    See Also
    --------
    hexapod.leg.recalculateLegAngles:
        Finds the coax, femur, and tibia angles of each leg.

    Notes
    -----
    This function does not allow for a negative distance to walk. If the user
    wants the hexapod to walk backwards, the angle should be set to -90
    degrees.
    """
    max_step_size = 30  # Maximum step distance
    # raise an error if the robot is not commanded to move a positive distance
    if distance <= 0:
        raise ValueError("distance must be a positive distance")
    steps = int(distance / max_step_size)
    if distance % max_step_size > 0:
        steps += 1

    right_foot = True  # If the right foot is moving forward
    # Sets the remaining distance to move forward as the full distance to move
    remaining_distance = distance
    for i in range(steps):  # iterate over the number of steps to take
        # if the remaining distance to move is less than the max step size,
        # then move the robot remaining distance
        if remaining_distance <= max_step_size:
            # if this is the only step then the feet only needs to move
            # forward the remaining distance
            if steps == 1:
                temp_walk_positions = stepForward(step_angle=angle,
                                                  distance=remaining_distance,
                                                  right_foot=right_foot,
                                                  z_resolution=z_resolution)
            # if this is not the first step the robot needs to move forward
            # the max step size first to bring the robot to a neutral position
            # before moving the rest of the distance.
            else:
                temp_walk_positions = stepForward(step_angle=angle,
                                                  distance=remaining_distance
                                                  + max_step_size,
                                                  right_foot=right_foot,
                                                  z_resolution=z_resolution)

            # Get the current feet positions from the last stepped position
            if 'walk_positions' in locals():
                feet_positions = walk_positions[-1, :, :]
            # if there was not a step yet get the feet positions from the leg
            # model
            else:
                feet_positions = getFeetPos(leg_model)
            # add all of the feet positions to the walk
            for j in range(temp_walk_positions.shape[0]):
                temp_walk_positions[j, :, :] = temp_walk_positions[j, :, :]\
                    + feet_positions
            # try to add the next step to the walk
            if 'walk_positions' in locals():
                walk_positions = np.concatenate((walk_positions,
                                                 temp_walk_positions),
                                                axis=0)
            # create walk array with the first step
            else:
                walk_positions = temp_walk_positions
            right_foot = not right_foot  # switch which foot steps forward
            break

        # if this is the first of more than one step, move forward the max
        # distance
        if i == 0:
            walk_positions = stepForward(step_angle=angle,
                                         distance=max_step_size,
                                         right_foot=right_foot,
                                         z_resolution=z_resolution)
            feet_positions = getFeetPos(leg_model)
            for j in range(walk_positions.shape[0]):
                walk_positions[j, :, :] = walk_positions[j, :, :]\
                    + feet_positions
            # reduce the remaining distance by the max step size
            remaining_distance -= max_step_size
            right_foot = not right_foot

        # if this is not the first step and the next step is more than the max
        # distance, move the legs forward by twice the max distance to reset
        # and then move forward the max distance
        else:
            temp_walk_positions = stepForward(step_angle=angle,
                                              distance=max_step_size * 2,
                                              right_foot=right_foot,
                                              z_resolution=z_resolution)

            feet_positions = walk_positions[-1, :, :]

            for j in range(temp_walk_positions.shape[0]):
                temp_walk_positions[j, :, :] = temp_walk_positions[j, :, :]\
                    + feet_positions
            walk_positions = np.concatenate((walk_positions,
                                             temp_walk_positions),
                                            axis=0)
            remaining_distance -= max_step_size
            right_foot = not right_foot

    # reset the position of the robot by moving the last step distance
    temp_walk_positions = stepForward(step_angle=angle,
                                      distance=remaining_distance,
                                      right_foot=right_foot,
                                      z_resolution=z_resolution)

    feet_positions = walk_positions[-1, :, :]
    for j in range(temp_walk_positions.shape[0]):
        temp_walk_positions[j, :, :] = temp_walk_positions[j, :, :]\
            + feet_positions
    walk_positions = np.concatenate((walk_positions,
                                     temp_walk_positions),
                                    axis=0)
    return walk_positions
