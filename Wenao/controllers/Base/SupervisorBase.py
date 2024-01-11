"""
The Basic Supervisor class.
All Supervisor classes should be derived from this class.
"""

import os
import sys

currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)


from controller import Supervisor
from Utils import Functions


class SupervisorBase(Supervisor):
    def __init__(self):
        super().__init__()

        self.emitter = self.getDevice("emitter")

        self.ball = self.getFromDef("BALL")

        self.robots = {
            "RedGoalkeeper": self.getFromDef("RedGoalkeeper"),
            "RedDefenderLeft": self.getFromDef("RedDefenderLeft"),
            "RedDefenderRight": self.getFromDef("RedDefenderRight"),
            "RedForward": self.getFromDef("RedForward"),
            "BlueGoalkeeper": self.getFromDef("BlueGoalkeeper"),
            "BlueDefenderLeft": self.getFromDef("BlueDefenderLeft"),
            "BlueDefenderRight": self.getFromDef("BlueDefenderRight"),
            "BlueForward": self.getFromDef("BlueForward"),
        }

        self.ballPriority = "R"

        self.previousBallLocation = [0, 0, 0.0798759]

    def getBallPosition(self) -> list:
        """Get the soccer ball coordinate on the field.

        Returns:
            list: x, y, z coordinates.
        """
        newBallLocation = self.ball.getPosition()

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
        robotTranslation = self.robots[robotName].getPosition()
        return robotTranslation

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
        RedGoalkeeper = self.getRobotPosition("RedGoalkeeper")
        RedDefenderLeft = self.getRobotPosition("RedDefenderLeft")
        RedDefenderRight = self.getRobotPosition("RedDefenderRight")
        RedForward = self.getRobotPosition("RedForward")
        BlueGoalkeeper = self.getRobotPosition("BlueGoalkeeper")
        BlueDefenderLeft = self.getRobotPosition("BlueDefenderLeft")
        BlueDefenderRight = self.getRobotPosition("BlueDefenderRight")
        BlueForward = self.getRobotPosition("BlueForward")

        # Pack the values into a string to transmit

        message = ",".join(
            map(
                str,
                ballPosition
                + RedGoalkeeper
                + RedDefenderLeft
                + RedDefenderRight
                + RedForward
                + BlueGoalkeeper
                + BlueDefenderLeft
                + BlueDefenderRight
                + BlueForward,
            )
        )

        # Send the message using the emitter
        self.emitter.send(message.encode())

    def setBallPriority(self, priority):
        self.ballPriority = priority

    def resetSimulation(self):
        self.previousBallLocation = [0, 0, 0.0798759]
        self.simulationReset()
        for robot in self.robots.values():
            robot.resetPhysics()

    def stopSimulation(self):
        self.simulationSetMode(self.SIMULATION_MODE_PAUSE)
