import numpy as np


def calculateDistance(coordinate1, coordinate2) -> float:
    """Calculates the distance between two 2D coordinates.

    Args:
        coordinate1 (list): x, y coordinates.
        coordinate2 (list): x, y coordinates.

    Returns:
        float: Distance between coordinates.
    """
    deltaX = np.abs(coordinate1[0] - coordinate2[0])
    deltaY = np.abs(coordinate1[1] - coordinate2[1])
    return np.hypot(deltaX, deltaY)


def calculateAngleAccordingToXAxis(ballCoordinate, robotCoordinate) -> float:
    """Finds the angle between robot-ball vector and x-axis.

    Args:
        ballCoordinate (list): x, y, z coordinates of the ball.
        robotCoordinate (list): x, y coordinates of the robot.

    Returns:
        float: Angle between robot-ball vector and x-axis.
    """
    hypot = calculateDistance(ballCoordinate, robotCoordinate)
    deltaX = np.abs(ballCoordinate[0] - robotCoordinate[0])

    cosTheta = deltaX / hypot
    degree = np.degrees(np.arccos(cosTheta))

    return degree


def calculateBallRegion(ballCoordinate, robotCoordinate) -> int:
    """Ball region is the region of the ball according to robot.
        We assumed the robot as origin of coordinate system.

    Args:
        ballCoordinate (list): x, y, z coordinates of the ball.
        robotCoordinate (list): x, y coordinates of the robot.

    Returns:
        int: 1 = top-right, 2 = top-left, 3 = bottom-left, 4 = bottom-right
    """
    ballRegion = 0
    # Find the location of the ball according to the robot.
    if ballCoordinate[0] > robotCoordinate[0]:
        if ballCoordinate[1] > robotCoordinate[1]:
            ballRegion = 1
        else:
            ballRegion = 4
    else:
        if ballCoordinate[1] > robotCoordinate[1]:
            ballRegion = 2
        else:
            ballRegion = 3

    return ballRegion


def calculateTurningAngleAccordingToRobotHeading(
    ballCoordinate, robotCoordinate, robotHeadingAngle
) -> float:
    """Calculates the turning angle to the ball for robot according to it's heading.

    Args:
        ballCoordinate (list): x, y, z coordinates of the ball.
        robotCoordinate (list): x, y coordinates of the robot.
        robotHeadingAngle (float): Robot heading angle as radian.

    Returns:
        float: The angle needed to turn the ball.
    """

    degree = calculateAngleAccordingToXAxis(ballCoordinate, robotCoordinate)
    ballRegion = calculateBallRegion(ballCoordinate, robotCoordinate)

    # Recalculate the degree according to ballRegion.
    if ballRegion == 2:
        degree = 180 - degree
    elif ballRegion == 3:
        degree = degree - 180
    elif ballRegion == 4:
        degree = -degree

    # Finds the angle of robots heading.
    zAxisDegree = np.degrees(robotHeadingAngle)

    turningAngle = degree - zAxisDegree
    if turningAngle > 180:
        turningAngle = turningAngle - 360
    elif turningAngle < -180:
        turningAngle = turningAngle + 360

    return turningAngle
