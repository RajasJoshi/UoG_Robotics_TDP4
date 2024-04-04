"""
The Basic Supervisor class.
All Supervisor classes should be derived from this class.
"""

import os
import sys
import time

from controller import Supervisor
from Utils import Functions

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))


class SupervisorBase(Supervisor):
    RobotList = [
        "RedGoalkeeper",
        "RedDefender",
        "RedForwardB",
        "RedForwardA",
        "BlueGoalkeeper",
        "BlueDefender",
        "BlueForwardB",
        "BlueForwardA",
    ]

    def __init__(self):
        super().__init__()

        self.emitter = self.getDevice("emitter")
        self.ball = self.getFromDef("BALL")
        self.robots = {name: self.getFromDef(name) for name in self.RobotList}
        self.latestGoalTime = 0
        self.ballPriority = "R"
        self.previousBallLocation = [0, 0, 0.0798759]
        self.score = [0, 0]
        Red = "Red"
        Blue = "Blue"
        self.setLabel(0, "█" * 100, 0, 0, 0.1, 0xFFFFFF, 0.3, "Lucida Console")
        self.setLabel(1, "█" * 100, 0, 0.048, 0.1, 0xFFFFFF, 0.3, "Lucida Console")
        self.setLabel(2, Red, 0.01, 0.003, 0.08, 0xFF0000, 0, "Arial")
        self.setLabel(3, Blue, 0.01, 0.051, 0.08, 0x0000FF, 0, "Arial")

    def updateScoreboard(self):
        for i in range(2):
            color = 0xFF0000 if i == 0 else 0x0000FF
            self.setLabel(
                4 + i,
                "{:d}".format(self.score[i]),
                0.8,
                0.003 + 0.048 * i,
                0.08,
                color,
                0,
                "Arial",
            )

    def isitGoal(self):
        ball_coordinate = self.getBallPosition()

        if abs(ball_coordinate[0]) > 4.5:
            if ball_coordinate[1] < 0.7 and ball_coordinate[1] > -0.7:
                goal_team = "RED" if 4.5 < ball_coordinate[0] else "BLUE"
                print(f"{goal_team} GOAL!")
                if goal_team == "RED":
                    self.score[0] += 1
                else:
                    self.score[1] += 1

                self.resetSimulation()

    def getBallPosition(self) -> list:
        """Get the soccer ball coordinate on the field.

        Returns:
            list: x, y, z coordinates.
        """
        newBallLocation = self.ball.getPosition()

        if max(map(abs, newBallLocation[:2])) < 4.5:
            if (
                max(
                    abs(a - b)
                    for a, b in zip(self.previousBallLocation, newBallLocation)
                )
                > 0.05
            ):
                self.ballPriority = "N"
                self.previousBallLocation = newBallLocation

        return newBallLocation

    def setBallPosition(self, ballPosition) -> None:
        """Set the soccer ball coordinate on the field.

        Args:
            list: x, y, z coordinates.
        """
        self.previousBallLocation = ballPosition
        self.ball.getField("translation").setSFVec3f(ballPosition)
        self.ball.resetPhysics()

    def getRobotPosition(self, robotName) -> list:
        """Get the robot coordinate on the field.

        Returns:
            list: x, y, z coordinates.
        """
        return self.robots[robotName].getPosition()

    def getRobotOrientation(self, robotName) -> list:
        """Get the robot coordinate on the field.

        Returns:
            list: x, y, z coordinates.
        """
        return self.robots[robotName].getOrientation()

    def getBallOwner(self) -> str:
        """Calculate the ball owner team from the distances from the ball.

        Returns:
            str: Ball owner team first letter.
        """
        ballPosition = self.getBallPosition()
        distances = {
            name: Functions.calculateDistance(ballPosition, self.getRobotPosition(name))
            for name in self.robots
        }
        ballOwnerRobotName = min(distances, key=distances.get)
        if distances[ballOwnerRobotName] < 0.2:
            return ballOwnerRobotName.ljust(9, "*")
        else:
            return 'None'

    def sendSupervisorData(self) -> None:
        """Send Data (ballPosition, ballOwner, ballPriority, ...) to Robots. Channel is '0'."""

        # Pack the values into a string to transmit
        message = ",".join(
            map(
                str,
                [
                    self.getTime(),
                    self.ballPriority,
                    self.getBallOwner(),
                    *self.getBallPosition(),
                    *self.getRobotPosition("RedGoalkeeper"),
                    *self.getRobotPosition("RedDefender"),
                    *self.getRobotPosition("RedForwardB"),
                    *self.getRobotPosition("RedForwardA"),
                    *self.getRobotPosition("BlueGoalkeeper"),
                    *self.getRobotPosition("BlueDefender"),
                    *self.getRobotPosition("BlueForwardB"),
                    *self.getRobotPosition("BlueForwardA"),
                ],
            )
        )
        self.emitter.send(message.encode("utf-8"))

    def resetSimulation(self):
        self.previousBallLocation = [0, 0, 0.0798759]
        self.simulationReset()
        for robot in self.robots.values():
            robot.resetPhysics()

    def get_data(self):
        data = {'time_steps': self.getTime(),
                'possession': self.getBallOwner(),
                'RedForwardA': self.getRobotPosition("RedGoalkeeper"),
                'RedDefender': self.getRobotPosition("RedDefender"),
                'RedForwardB': self.getRobotPosition("RedForwardB"),
                'RedForwardA': self.getRobotPosition("RedForwardA"),
                'BlueGoalkeeper': self.getRobotPosition("BlueGoalkeeper"),
                'BlueDefender': self.getRobotPosition("BlueDefender"),
                'BlueForwardB': self.getRobotPosition("BlueForwardB"),
                'BlueForwardA': self.getRobotPosition("BlueForwardA")}
        
        return data