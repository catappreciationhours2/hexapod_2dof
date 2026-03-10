"""
Functions to generate, change, and use the model of the hexapod's legs.

These functions take the leg dimensions and servo angles to find their position
or can do the opposite and find the angles from the dimensions. Together,
these functions define the leg model and allow the user to update and pull
relevant information about the legs.

Functions
---------
getFeetPos:
    Output the x, y, z position of the feet of the hexapod.
legAngle:
    Finds the angles for the coax, femur, and tibia leg segments.
legModel:
    Generates the model of the legs based on the servo angles of the legs.
legPos:
    Finds the positions for the leg segments.
recalculateLegAngles:
    Finds the coax, femur, and tibia angles of each leg.
startLegPos:
    Find the neutral position of the hexapod.
"""
import numpy as np
from numpy.linalg import inv
import math
from math import degrees, sin, cos, acos, atan2, sqrt, pi
from hexapod.rotation import yRot, zRot
from typing import List

RIGID_LEG_LENGTH = 164.96 # sum of tibia and femur - NEED TO UPDATE


def getFeetPos(leg_model: np.ndarray) -> np.ndarray:
    """
    Output the x, y, z position of the feet of the hexapod.

    Return the current positions of the ends of the legs or where the feet
    of the hexapod currently are.

    Parameters
    ----------
    leg_model: np.ndarray
        the 4x3x6 numpy array that holds the locations of the coax, femur,
        and tibia servos as well as the feet end positions.

    Returns
    -------
    feet_positions: np.ndarray
        The 6x3 numpy array of where each leg of the hexapod ends as an
        x, y, z point.
    """
    feet_positions = np.empty([6, 3])
    for i in range(6):
        feet_positions[i, :] = leg_model[3, :, i]
    return feet_positions


def legAngle(x: float, y: float, z: float, coax: float = 26.34,
             femur: float = 76.2, tibia: float = 88.32) -> List[float]:
    """
    Finds the angles for the coax, femur, and tibia leg segments.

    Takes the foot position, or end point of the leg, and calculates what angle
    each segment of the legs should have to achieve that end position.

    Parameters
    ----------
    x: float
        The x location of the foot.
    y: float
        The y location of the foot.
    z: float
        The z location of the foot.
    coax: float, default=26.34
        The length of the coax segment in millimeters or the segment from the
        body attachment point to the beginning of the femur.
    femur: float, default=76.2
        The length of the femur segment in millimeters or the segment from the
        beginning of the femur to the start of the tibia.
    tibia: float, default=88.32
        The length of the tibia segment in millimeters or the segment from the
        start of the tibia to the foot of the leg.

    Returns
    -------
    [coax_angle, femur_angle, tibia_angle]: List[float]
        A list of the three servo angles for the leg segments.

    See Also
    --------
    recalculateLegAngles:
        Finds the coax, femur, and tibia angles of each leg.

    Notes:
        Refer to the Kinematics Calculations document in the Docs folder for
        how the equations in this function were found.
    """
    base_angle = math.degrees(math.atan2(y, x))
    
    base_rot_inv = inv(zRot(-base_angle))
    leg_rotated = np.matmul(base_rot_inv, np.array([[x, y, z]]).T)
    
    lx = leg_rotated[0].item()
    lz = leg_rotated[2].item()
    
    dx = lx - coax
    dz = lz
    
    # FIX: We remove the negative sign from dz to point the angle downwards 
    # OR we use math.atan2(dz, dx) depending on your yRot implementation.
    # Try this specific orientation for downward legs:
    knee_angle = math.degrees(math.atan2(dz, dx))
    
    if abs(base_angle) <= 1e-10: base_angle = 0
    if abs(knee_angle) <= 1e-10: knee_angle = 0
    
    return [base_angle, knee_angle, 0]


def legModel(leg_angles: np.ndarray, body_model: np.ndarray) -> np.ndarray:
    """
    Generates the model of the legs based on the servo angles of the legs.

    Recreates the leg model from all of the servo angles of the legs. This
    method is used when walking and turning to update the model of the robot
    for the next step.

    Parameters
    ----------
    leg_angles: np.ndarray
        A 6x3 numpy array of each leg's coax, femur, and tibia servo angles.
    body_model: np.ndarray
        The 7x3 numpy array containing the locations of the coax servos.

    Returns
    -------
    leg_model: np.ndarray
        the 4x3x6 numpy array that holds the locations of the coax, femur,
        and tibia servos as well as the feet end positions.

    See Also
    --------
    legPos:
        Finds the positions for the leg segments.
    """
    leg_model = np.empty([4, 3, 6])
    for i in range(6):
        # leg model takes the coax angle, femur angle, tibia angle, model of
        # the hexapod body, leg number
        leg_model[:, :, i] = legPos(leg_angles[i][0], leg_angles[i][1],
                                    leg_angles[i][2], body_model, i)
    return leg_model


def legPos(base_angle: float, knee_angle: float, tibia_angle: float,
           body_model: np.ndarray, leg_num: int, coax: float = 26.34,
           femur: float = 76.2, tibia: float = 88.32) -> np.ndarray:
    """
    Forward Kinematics for 2-DOF leg.
    Calculates 3D positions of joints.
    """
    # Use the combined rigid length
    rigid_limb = RIGID_LEG_LENGTH

    base_rot = zRot(base_angle)
    # Knee rotates locally on Y axis after base rotation
    knee_rot = np.matmul(yRot(knee_angle), base_rot)

    # Point 0: Body attachment
    leg_base = np.array([body_model[leg_num, :]]).T
    
    # Point 1: Knee joint (end of coax)
    leg_knee = np.matmul(inv(base_rot), np.array([[coax, 0, 0]]).T) + leg_base
    
    # Point 2 & 3: Foot (end of rigid limb)
    # We double up the knee point to keep the array size (4,3) for the plotter
    leg_foot = np.matmul(inv(knee_rot), np.array([[rigid_limb, 0, 0]]).T) + leg_knee

    leg_positions = np.concatenate((leg_base.T, leg_knee.T, leg_knee.T, leg_foot.T),
                                   axis=0)
    return leg_positions


def recalculateLegAngles(feet_positions: np.ndarray,
                         body_model: np.ndarray) -> np.ndarray:
    """
    Finds the coax, femur, and tibia angles of each leg.

    Finds the coax, femur, and tibia servo angles of each leg based on the
    location of each leg's foot.

    Parameters
    ----------
    feet_positions: np.ndarray
        A 6x3 numpy array of each leg's end position in x, y, z.
    body_model: np.ndarray
        The 7x3 numpy array containing the locations of the coax servos.

    Returns
    -------
    leg_angles: np.ndarray
        A 6x3 numpy array of each leg's coax, femur, and tibia servo angles.

    See Also
    --------
    legAngle:
        Finds the angles for the coax, femur, and tibia leg segments.
    """
    leg_angles = np.empty([6, 3])
    for i in range(6):
        leg_angles[i, :] = legAngle(feet_positions[i, 0] - body_model[i, 0],
                                    feet_positions[i, 1] - body_model[i, 1],
                                    feet_positions[i, 2] - body_model[i, 2])
    return leg_angles


# NEED TO UPDATE START_RADIUS, START_HEIGHT
def startLegPos(body_model: np.ndarray, start_radius: float = 170,
                start_height: float = 60) -> np.ndarray:
    """
    Find the neutral position of the hexapod.

    Create the starting angles of the legs on the hexapod based on the
    standing radius on the ground and height off the ground.

    Parameters
    ----------
    body_model: np.ndarray
        The 7x3 numpy array containing the locations of the coax servos.
    start_radius: float, default=180
        The radius in millimeters that the feet make with the ground when
        equally spaced around the hexapod
    start_height: float, default=60
        The distance off the ground from the top of the legs to the bottom
        in millimeters.

    Returns
    -------
    start_leg: np.ndarray
        The 6x3 numpy array of servo angles for the neutral position of the
        hexapod

    See Also
    --------
    recalculateAngles:
        Finds the coax, femur, and tibia angles of each leg.
    """
    start_leg_pos = np.array([[start_radius * cos(pi / 3), start_radius
                               * sin(pi / 3), - start_height],
                              [start_radius, 0, - start_height],
                              [start_radius * cos(- pi / 3), start_radius
                               * sin(- pi / 3), - start_height],
                              [start_radius * cos(- 2 * pi / 3) , start_radius
                               * sin(- 2 * pi / 3), - start_height],
                              [- start_radius, 0, - start_height],
                              [start_radius * cos(2 * pi / 3), start_radius
                               * sin(2 * pi / 3), - start_height]])
    start_leg = recalculateLegAngles(start_leg_pos, body_model)
    return start_leg
