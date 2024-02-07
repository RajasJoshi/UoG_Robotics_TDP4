"""
The Basic Robot behaviour and feature class.
All robots should be derived from this class.
"""

import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))


import configparser
from enum import Enum
import numpy as np
from controller import Robot
from Utils import Functions
from Utils.Consts import Motions
from Utils.ImageServer import ImageServer
from Utils.ProcessSupervisor import SupervisorData


class RobotState(Enum):
    INIT = 0
    LOOK_THE_BALL = 1
    BE_A_GOALKEEPER = 2
    SAVE_THE_BALL = 3


class SoccerRobot(Robot):
    PHALANX_MAX = 8

    def __init__(self, config):
        Robot.__init__(self)
        self.robotName = self.getName()
        self.currentlyPlaying = False
        self.config = config
        self.AppState = RobotState.INIT

        self.StartLocation = [4.5, 0.00]

        self.enableDevices()
        # Load motion files
        self.motions = Motions()
        self.currentlyMoving = False
        self.motionQueue = [self.motions.standInit]
        self.startMotion()

        self.bVisionUsed = config.getboolean("BlueTeam", "Vision")

        if self.bVisionUsed:
            self.TopCamServer = ImageServer(
                self.cameraTop.getWidth(),
                self.cameraTop.getHeight(),
                self.cameraTop,
                self.robotName,  # Pass robot name to ImageServer
                "Top",
            )
            self.BottomCamServer = ImageServer(
                self.cameraBottom.getWidth(),
                self.cameraBottom.getHeight(),
                self.cameraBottom,
                self.robotName,  # Pass robot name to ImageServer
                "Bottom",
            )
        else:
            self.Supervisor = SupervisorData(self.robotName)

    def run(self):
        try:
            while self.step(self.timeStep) != -1:
                self.clearMotionQueue()

                if self.isNewDataAvailable():
                    self.Supervisor.updateData(self.receiver)
                    whatToDoNext = self.NextMotion()

                    if self.isNewMotionValid(whatToDoNext):
                        self.addMotionToQueue(whatToDoNext)
                        self.startMotion()

                if self.bVisionUsed:
                    try:
                        top_image = self.cameraTop.getImage()
                        bottom_image = self.cameraBottom.getImage()

                        self.TopCamServer.send(top_image)
                        self.BottomCamServer.send(bottom_image)

                    except ValueError as e:
                        # Handle the exception (e.g., print an error message)
                        print(f"Error getting camera image: {e}")

                if self.step(self.timeStep) == -1:
                    break
        except Exception as e:
            print(f"An error occurred: {e}")

    def enableDevices(self):
        # get the time step of the current world.
        self.timeStep = int(self.getBasicTimeStep())

        # camera
        self.cameraTop = self.getDevice("CameraTop")
        self.cameraBottom = self.getDevice("CameraBottom")
        self.cameraTop.enable(4 * self.timeStep)
        self.cameraBottom.enable(4 * self.timeStep)

        # accelerometer
        self.accelerometer = self.getDevice("accelerometer")
        self.accelerometer.enable(4 * self.timeStep)

        # inertial unit
        self.inertialUnit = self.getDevice("inertial unit")
        self.inertialUnit.enable(self.timeStep)

        # ultrasound sensors
        self.ultrasound = []
        self.ultrasound.append(self.getDevice("Sonar/Left"))
        self.ultrasound.append(self.getDevice("Sonar/Right"))
        self.ultrasound[0].enable(self.timeStep)
        self.ultrasound[1].enable(self.timeStep)

        # Receiver
        self.receiver = self.getDevice("receiver")
        self.receiver.enable(self.timeStep)

    def interruptMotion(self) -> None:
        """Interrupt if the robot is moving."""
        if self.currentlyMoving:
            self.currentlyMoving.stop()
            self.currentlyMoving = False

    def startMotion(self) -> None:
        """Start a motion from the queue if the previous one completed."""
        if self.currentlyMoving == False or self.currentlyMoving.isOver():
            if len(self.motionQueue) > 0:
                currentMotion = self.motionQueue.pop(0)
                currentMotion.play()
                self.currentlyMoving = currentMotion

    def addMotionToQueue(self, motion) -> None:
        """Add the next motion of the robot to the queue.

        Args:
            motion (Motion): Loaded motion variable.
        """
        self.motionQueue.append(motion)

    def clearMotionQueue(self) -> None:
        """Clear the motion queue."""
        self.motionQueue.clear()

    def isNewMotionValid(self, newMotion) -> bool:
        """Compare the new motion and current motion.

        Args:
            newMotion (Motion): New motion

        Returns:
            bool: Is the new motion valid?
        """
        if newMotion == None or (
            self.currentlyMoving != False
            and self.currentlyMoving.isOver() != True
            and newMotion.name == self.currentlyMoving.name
        ):
            return False

        return True

    def getTurningMotion(self, turningAngle):
        """Decide the motion according to the turning angle.

        Args:
            turningAngle (double): Turning angle in degrees.

        Returns:
            Motion: Decided motion.
        """
        if turningAngle > 90:
            return self.motions.turnLeft180
        elif turningAngle > 50:
            return self.motions.turnLeft60
        elif turningAngle > 30:
            return self.motions.turnLeft40
        elif turningAngle > 20:
            return self.motions.turnLeft30
        elif turningAngle > 10:
            return self.motions.turnLeft20
        elif turningAngle < -50:
            return self.motions.turnRight60
        elif turningAngle < -30:
            return self.motions.turnRight40
        elif turningAngle < -10:
            return self.motions.turnRight20
        else:
            return None

    def getRollPitchYaw(self) -> list:
        """Get the Roll, Pitch and Yaw angles of robot.

        Returns:
            list: Roll, Pitch and Yaw angles.
        """
        return self.inertialUnit.getRollPitchYaw()

    def isNewDataAvailable(self) -> bool:
        """Check if there is a new ball data available.

        Returns:
            bool: Is there any new ball data available?
        """
        return self.receiver.getQueueLength() > 0

    def NextMotion(self):
        # Fall Detection
        acc = self.accelerometer.getValues()
        if (
            np.abs(acc[0]) > np.abs(acc[1])
            and np.abs(acc[0]) > np.abs(acc[2])
            and acc[0] < -5
        ):
            return self.motions.standUpFromFront
        elif (
            np.abs(acc[0]) > np.abs(acc[1])
            and np.abs(acc[0]) > np.abs(acc[2])
            and acc[2] > 0
        ):
            return self.motions.standUpFromBack

        # Get the current position
        currentSelfPosition = self.Supervisor.getSelfPosition()
        currentBallPosition = self.Supervisor.getBallData()
        match self.AppState:
            case RobotState.INIT:
                # Calculate the distance to the goal position
                distance = Functions.calculateDistance(
                    self.StartLocation, currentSelfPosition
                )

                if distance <= 0.2:
                    self.AppState = RobotState.LOOK_THE_BALL
                    return self.motions.standInit
                else:
                    # Calculate the angle to the target position
                    targetAngle = np.degrees(
                        np.arctan2(
                            self.StartLocation[1] - currentSelfPosition[1],
                            self.StartLocation[0] - currentSelfPosition[0],
                        )
                    )

                    # Get the robot's orientation angle
                    robotAngle = np.degrees(self.getRollPitchYaw()[2])

                    # Calculate the turn angle in the range [-180, 180)
                    turnAngle = (targetAngle - robotAngle + 180) % 360 - 180
                    if abs(turnAngle) > 10:
                        return self.getTurningMotion(turnAngle)

                return self.motions.forwardLoop

            case RobotState.LOOK_THE_BALL:
                # Calculate the angle to the target position
                targetAngle = np.degrees(
                    np.arctan2(
                        currentBallPosition[1] - currentSelfPosition[1],
                        currentBallPosition[0] - currentSelfPosition[0],
                    )
                )

                # Get the robot's orientation angle
                robotAngle = np.degrees(self.getRollPitchYaw()[2])

                # Calculate the turn angle in the range [-180, 180)
                turnAngle = (targetAngle - robotAngle + 180) % 360 - 180

                if abs(turnAngle) > 10:
                    return self.getTurningMotion(turnAngle)

                # Calculate the distance to the goal position
                distance = Functions.calculateDistance(
                    self.StartLocation, currentSelfPosition
                )

                if distance <= 0.2 and abs(turnAngle) < 10:
                    self.AppState = RobotState.BE_A_GOALKEEPER

                return self.motions.standInit

            case RobotState.BE_A_GOALKEEPER:
                isTheBallClose = Functions.calculateDistance(
                    currentBallPosition, [4.5, 0.00]
                )

                targetAngle = np.degrees(
                    np.arctan2(
                        currentBallPosition[1] - currentSelfPosition[1],
                        currentBallPosition[0] - currentSelfPosition[0],
                    )
                )

                # Get the robot's orientation angle
                robotAngle = np.degrees(self.getRollPitchYaw()[2])

                # Calculate the turn angle in the range [-180, 180)
                turnAngle = (targetAngle - robotAngle + 180) % 360 - 180
                turningMotion = self.getTurningMotion(turnAngle)
                if turningMotion is not None:
                    return turningMotion

                if isTheBallClose <= 1.0:
                    # Calculate the distance to the goal position
                    distance = Functions.calculateDistance(
                        currentBallPosition, currentSelfPosition
                    )

                    if distance <= 0.2 and abs(turnAngle) < 10:
                        return self.motions.shoot

                    return self.motions.forwardLoop

            case _:
                self.AppState = RobotState.INIT


def main():
    # Create the robot and initialize the camera
    config = configparser.ConfigParser()
    config.read("../Utils/globalconfig.ini")
    robot = SoccerRobot(config)
    robot.run()


if __name__ == "__main__":
    main()
