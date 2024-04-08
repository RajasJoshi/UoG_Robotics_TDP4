import numpy as np
import random

class SharedState:
    def __init__(self):
        self.random_team()

    def random_team(self):
        # Generate a random boolean value
        self.red_team_starts = random.choice([True, False])
        self.blue_team_starts = not self.red_team_starts

    def game_start(self):
        return self.red_team_starts, self.blue_team_starts

shared_state = SharedState()

def calculateDistance(coordinate1, coordinate2) -> float:
    """Calculates the distance between two 2D coordinates.

    Args:
        coordinate1 (list): x, y coordinates.
        coordinate2 (list): x, y coordinates.

    Returns:
        float: Distance between coordinates.
    """
    deltaX = coordinate1[0] - coordinate2[0]
    deltaY = coordinate1[1] - coordinate2[1]
    return np.hypot(deltaX, deltaY)

def calculateTargetAngle(target, current):
        return np.degrees(
            np.arctan2(
                target[1] - current[1],
                target[0] - current[0],
            )
        )


@staticmethod
def calculateAngle(v1, v2):
    """Calculate the angle between two vectors.

    Args:
        v1 (list): The first vector.
        v2 (list): The second vector.

    Returns:
        float: The angle between the vectors in degrees.
    """
    dotProduct = v1[0] * v2[0] + v1[1] * v2[1]
    magnitudeProduct = (v1[0] ** 2 + v1[1] ** 2) ** 0.5 * (
        v2[0] ** 2 + v2[1] ** 2
    ) ** 0.5
    cosAngle = dotProduct / magnitudeProduct
    angle = np.arccos(cosAngle)
    return np.degrees(angle)


def calculateTurnAngle(targetAngle, robotAngle):
    return (targetAngle - robotAngle + 180) % 360 - 180
