"""
The Basic Supervisor class.
All Supervisor classes should be derived from this class.
"""

import os
import sys

currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)

import struct

from controller import Supervisor
from Utils import Functions
from Utils.Consts import TIME_STEP


class SupervisorBase(Supervisor):
    def __init__(self):
        super().__init__()

        self.emitter = self.getDevice("emitter")

        self.ball = self.getDevice("RobocupSoccerBall")

        self.robots = {
            "RedGoalkeeper": self.getDevice("RedGoalkeeper"),
            "RedDefenderLeft": self.getDevice("RedDefenderLeft"),
            "RedDefenderRight": self.getDevice("RedDefenderRight"),
            "RedForward": self.getDevice("RedForward"),
            "BlueGoalkeeper": self.getDevice("BlueGoalkeeper"),
            "BlueDefenderLeft": self.getDevice("BlueDefenderLeft"),
            "BlueDefenderRight": self.getDevice("BlueDefenderRight"),
            "BlueForward": self.getDevice("BlueForward"),
        }

        self.ballPriority = "R"

        self.previousBallLocation = [0, 0, 0.0798759]

    def getBallPosition(self) -> list:
        """Get the soccer ball coordinate on the field.

        Returns:
            list: x, y, z coordinates.
        """
        ballTranslation = self.ball.getField("translation")
        newBallLocation = ballTranslation.getSFVec3f()

        if abs(newBallLocation[0]) < 4.5 and abs(newBallLocation[1]) < 3:
            if (
                self.previousBallLocation[0] + 0.05 < newBallLocation[0]
                or self.previousBallLocation[0] - 0.05 > newBallLocation[0]
                or self.previousBallLocation[1] + 0.05 < newBallLocation[1]
                or self.previousBallLocation[1] - 0.05 > newBallLocation[1]
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
        ballTranslation = self.ball.getField("translation")
        ballTranslation.setSFVec3f(ballPosition)
        self.ball.resetPhysics()

    def getRobotPosition(self, robotName) -> list:
        """Get the robot coordinate on the field.

        Returns:
            list: x, y, z coordinates.
        """
        robotTranslation = self.robots[robotName].getField("translation")
        return robotTranslation.getSFVec3f()

    def getBallOwner(self) -> str:
        """Calculate the ball owner team from the distances from the ball.

        Returns:
            str: Ball owner team first letter.
        """

        ballPosition = self.getBallPosition()
        ballOwnerRobotName = "RedGoalkeeper"
        minDistance = Functions.calculateDistance(
            ballPosition, self.getRobotPosition(ballOwnerRobotName)
        )
        for i, key in enumerate(self.robots):
            tempDistance = Functions.calculateDistance(
                ballPosition, self.getRobotPosition(key)
            )
            if tempDistance < minDistance:
                minDistance = tempDistance
                ballOwnerRobotName = key

        if len(ballOwnerRobotName) < 9:
            for i in range(len(ballOwnerRobotName), 9):
                ballOwnerRobotName = ballOwnerRobotName + "*"

        return ballOwnerRobotName

    def sendSupervisorData(self) -> None:
        """Send Data (ballPosition, ballOwner, ballPriority, ...) to Robots. Channel is '0'."""

        ballPosition = self.getBallPosition()
        ballOwner = bytes(self.getBallOwner(), "utf-8")
        ballPriority = bytes(self.ballPriority, "utf-8")

        redGk = self.getRobotPosition("RedGoalkeeper")
        redDefLeft = self.getRobotPosition("RedDefenderLeft")
        redDefRight = self.getRobotPosition("RedDefenderRight")
        redFw = self.getRobotPosition("RedForward")
        blueGk = self.getRobotPosition("BlueGoalkeeper")
        blueDef = self.getRobotPosition("BlueDefenderLeft")
        blueFwLeft = self.getRobotPosition("BlueDefenderRight")
        blueFwRight = self.getRobotPosition("BlueForward")

        data = struct.pack(
            "dd9ss24d",
            ballPosition[0],
            ballPosition[1],
            ballOwner,
            ballPriority,
            redGk[0],
            redGk[1],
            redGk[2],
            redDefLeft[0],
            redDefLeft[1],
            redDefLeft[2],
            redDefRight[0],
            redDefRight[1],
            redDefRight[2],
            redFw[0],
            redFw[1],
            redFw[2],
            blueGk[0],
            blueGk[1],
            blueGk[2],
            blueDef[0],
            blueDef[1],
            blueDef[2],
            blueFwLeft[0],
            blueFwLeft[1],
            blueFwLeft[2],
            blueFwRight[0],
            blueFwRight[1],
            blueFwRight[2],
        )
        self.emitter.send(data)

    def setBallPriority(self, priority):
        self.ballPriority = priority

    def resetSimulation(self):
        self.previousBallLocation = [0, 0, 0.0798759]
        self.simulationReset()
        for robot in self.robots.values():
            robot.resetPhysics()

    def stopSimulation(self):
        self.simulationSetMode(self.SIMULATION_MODE_PAUSE)
