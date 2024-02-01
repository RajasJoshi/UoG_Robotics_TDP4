"""
The Basic Robot behaviour and feature class.
All robots should be derived from this class.
"""
import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))


import math
from enum import Enum

import numpy as np
from controller import Robot
from Utils import Functions
from Utils.Consts import Motions
from Utils.ImageServer import ImageServer


class RobotState(Enum):
    INIT = 0
    LOOK_THE_BALL = 1
    BE_A_FORWARD = 2


class SoccerRobot(Robot):
    PHALANX_MAX = 8

    def __init__(self):
        Robot.__init__(self)
        self.robotName = self.getName()
        self.currentlyPlaying = False
        self.supervisorData = {
            "time": 0,
            "ballPriority": "",
            "ballOwner": "",
            "ballPosition": [0, 0, 0],
            "RedGoalkeeper": [0, 0, 0],
            "RedDefender": [0, 0, 0],
            "RedForwardB": [0, 0, 0],
            "RedForwardA": [0, 0, 0],
            "BlueGoalkeeper": [0, 0, 0],
            "BlueDefender": [0, 0, 0],
            "BlueForwardB": [0, 0, 0],
            "BlueForwardA": [0, 0, 0],
        }
        self.RobotList = [
            "RedGoalkeeper",
            "RedDefender",
            "RedForwardB",
            "RedForwardA",
            "BlueGoalkeeper",
            "BlueDefender",
            "BlueForwardB",
            "BlueForwardA",
        ]

        self.AppState = RobotState.INIT

        self.StartLocation = [-0.23606, 1.78171]
        self.TargetgoalPosition = [3.25978, 0.0196566]
        self.enableDevices()
        # Load motion files
        self.motions = Motions()

        self.currentlyMoving = False
        self.motionQueue = [self.motions.standInit]
        self.startMotion()

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

    def run(self):
        try:
            while self.step(self.timeStep) != -1:
                self.clearMotionQueue()

                if self.isNewDataAvailable():
                    self.getNewSupervisorData()
                    whatToDoNext = self.NextMotion()

                    if self.isNewMotionValid(whatToDoNext):
                        self.addMotionToQueue(whatToDoNext)
                        self.startMotion()

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

        # GPS
        self.gps = self.getDevice("gps")
        self.gps.enable(4 * self.timeStep)

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

        # get phalanx motor tags
        # the real Nao has only 2 motors for RHand/LHand
        # but in Webots we must implement RHand/LHand with 2x8 motors
        self.lphalanx = []
        self.rphalanx = []
        self.maxPhalanxMotorPosition = []
        self.minPhalanxMotorPosition = []
        for i in range(0, self.PHALANX_MAX):
            self.lphalanx.append(self.getDevice("LPhalanx%d" % (i + 1)))
            self.rphalanx.append(self.getDevice("RPhalanx%d" % (i + 1)))

            # assume right and left hands have the same motor position bounds
            self.maxPhalanxMotorPosition.append(self.rphalanx[i].getMaxPosition())
            self.minPhalanxMotorPosition.append(self.rphalanx[i].getMinPosition())

        # shoulder pitch motors
        self.RShoulderPitch = self.getDevice("RShoulderPitch")
        self.LShoulderPitch = self.getDevice("LShoulderPitch")

        # Receiver
        self.receiver = self.getDevice("receiver")
        self.receiver.enable(self.timeStep)

        # Emitter
        self.emitter = self.getDevice("emitter")

        # keyboard
        self.keyboard = self.getKeyboard()
        self.keyboard.enable(10 * self.timeStep)

    def interruptMotion(self) -> None:
        """Interrupt if the robot is moving."""
        if self.currentlyMoving:
            self.currentlyMoving.stop()
            self.currentlyMoving = False

    def interruptForwardsSprint(self) -> None:
        """Interrupt if the robot is moving forward."""
        if (
            self.currentlyMoving
            and self.currentlyMoving.name == "forwardsSprint"
            and self.currentlyMoving.getTime() == 1360
        ):  # we reached the end of forward.motion
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

    def getSelfPosition(self, robotName) -> list:
        """Get the robot coordinate on the field.

        Returns:
            list: x, y coordinates.
        """
        for key, value in self.supervisorData.items():
            # Compare search_string with the keys (case-insensitive)
            if robotName.lower() == key.lower():
                return value

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

    def getNewSupervisorData(self) -> None:
        """Get the latest supervisor data."""
        data = self.receiver.getString()

        if isinstance(data, bytes):
            message = data.decode("utf-8")
        else:
            message = data
        self.receiver.nextPacket()

        # Split the received string into individual values
        values = message.split(",")

        # Extract and process received values
        self.supervisorData["time"] = float(values[0])
        self.supervisorData["ballPriority"] = values[1]
        self.supervisorData["ballOwner"] = values[2]
        self.supervisorData["ballPosition"] = [float(values[i]) for i in range(3, 6)]

        for i, robot in enumerate(self.RobotList):
            self.supervisorData[robot] = [
                float(values[j]) for j in range(6 + i * 3, 9 + i * 3)
            ]

    def getBallData(self) -> list:
        """Get the latest coordinates of the ball and robots.

        Returns:
            list: x, y coordinates.
        """

        return self.getSelfPosition("ballPosition")

    def getBallOwner(self) -> str:
        """Get the ball owner team player.

        Returns:
            str: Ball owner team player.
        """
        ballOwner = ""
        for i in range(2, 11):
            ballOwner = ballOwner + self.supervisorData[i].decode("utf-8")

        return ballOwner.strip("*")

    def getBallPriority(self) -> str:
        """Get the ball prior team first letter.

        Returns:
            str: Ball prior team first letter.
        """

        return self.supervisorData[11].decode("utf-8")

    def NextMotion(self):
        # Fall Detection
        acc = self.accelerometer.getValues()
        if (
            math.fabs(acc[0]) > math.fabs(acc[1])
            and math.fabs(acc[0]) > math.fabs(acc[2])
            and acc[0] < -5
        ):
            return self.motions.standUpFromFront
        elif (
            math.fabs(acc[0]) > math.fabs(acc[1])
            and math.fabs(acc[0]) > math.fabs(acc[2])
            and acc[2] > 0
        ):
            return self.motions.standUpFromBack

        collision = self.detect_collision()
        if self.isNewMotionValid(collision):
            self.addMotionToQueue(collision)
            self.startMotion()

        # Get the current position
        currentSelfPosition = self.getSelfPosition(self.robotName)
        currentBallPosition = self.getBallData()

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
                    targetAngle = math.degrees(
                        math.atan2(
                            self.StartLocation[1] - currentSelfPosition[1],
                            self.StartLocation[0] - currentSelfPosition[0],
                        )
                    )

                    # Get the robot's orientation angle
                    robotAngle = math.degrees(self.getRollPitchYaw()[2])

                    # Calculate the turn angle in the range [-180, 180)
                    turnAngle = (targetAngle - robotAngle + 180) % 360 - 180
                    if abs(turnAngle) > 10:
                        return self.getTurningMotion(turnAngle)

                return self.motions.forwardLoop

            case RobotState.LOOK_THE_BALL:
                # Calculate the angle to the target position
                targetAngle = math.degrees(
                    math.atan2(
                        currentBallPosition[1] - currentSelfPosition[1],
                        currentBallPosition[0] - currentSelfPosition[0],
                    )
                )

                # Get the robot's orientation angle
                robotAngle = math.degrees(self.getRollPitchYaw()[2])

                # Calculate the turn angle in the range [-180, 180)
                turnAngle = (targetAngle - robotAngle + 180) % 360 - 180

                if abs(turnAngle) > 10:
                    return self.getTurningMotion(turnAngle)

                # Calculate the distance to the goal position
                distance = Functions.calculateDistance(
                    self.StartLocation, currentSelfPosition
                )

                if distance <= 0.2 and abs(turnAngle) < 10:
                    self.AppState = RobotState.BE_A_FORWARD

                return self.motions.standInit

            case RobotState.BE_A_FORWARD:
                if self.supervisorData["ballOwner"][0] != "R":
                    # Calculate the ball distance to the goal position
                    ball_distance = Functions.calculateDistance(
                        self.TargetgoalPosition, currentBallPosition
                    )

                    # Calculate the robot's distance to the ball position
                    robot_distance = Functions.calculateDistance(
                        currentBallPosition, currentSelfPosition
                    )

                    # Check if nao robot is near the ball
                    if robot_distance > 0.2:
                        self.AppState = RobotState.LOOK_THE_BALL
                        return self.motions.forwards50

                    # Check if the ball is near the goalpost
                    elif ball_distance <= 0.2:
                        return self.motions.shoot

                    else:
                        # Calculate the robot's angle to the goal position
                        dx, dy = (
                            self.TargetgoalPosition[0] - currentSelfPosition[0],
                            self.TargetgoalPosition[1] - currentSelfPosition[1],
                        )
                        targetAngle = math.degrees(math.atan2(dy, dx))

                        # Get the robot's orientation angle
                        robotAngle = math.degrees(self.getRollPitchYaw()[2])

                        # Calculate the turn angle in the range [-180, 180)
                        turnAngle = (targetAngle - robotAngle + 180) % 360 - 180

                        if abs(turnAngle) > 10:
                            if turnAngle > 90:
                                return self.motions.rightSidePass
                            elif turnAngle > 50:
                                return self.motions.rightSidePass
                            elif turnAngle > 30:
                                return self.motions.rightSidePass
                            elif turnAngle < -50:
                                return self.motions.leftSidePass
                            elif turnAngle < -30:
                                return self.motions.leftSidePass
                            else:
                                return self.motions.longShoot

                        return self.motions.forwards50
            case _:
                self.AppState = RobotState.INIT

    def detect_collision(self):
        for robot_name in self.RobotList:
            if robot_name != self.robotName:
                robot_position = self.getSelfPosition(robot_name)

                # Calculate the distance between robots
                robot_distance = Functions.calculateDistance(
                    robot_position, self.getSelfPosition(self.robotName)
                )

                # Check ultrasound values for collision detection
                if robot_distance < 0.4 and self.ultrasound[0].getValue() < 0.75:
                    self.interruptMotion()
                    return self.motions.sideStepRight

                if robot_distance < 0.4 and self.ultrasound[1].getValue() < 0.75:
                    self.interruptMotion()
                    return self.motions.sideStepLeft

        # Return a default motion or None if no collision is detected
        return None


def main():
    # Create the robot and initialize the camera
    robot = SoccerRobot()
    robot.run()


if __name__ == "__main__":
    main()
