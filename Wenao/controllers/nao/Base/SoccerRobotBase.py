"""
The Basic Robot behaviour and feature class.
All robots should be derived from this class.
"""
import struct
from abc import ABC, abstractmethod

from controller import Motion
from Utils.Consts import TIME_STEP, Motions


class SoccerRobot(ABC):
    def __init__(self, robot):
        self.robot = robot
        self.name = robot.getName()

        self.supervisorData = None

        # GPS
        self.gps = robot.getDevice("gps")
        self.gps.enable(TIME_STEP)

        # Receiver
        self.receiver = robot.getDevice("receiver")
        self.receiver.enable(TIME_STEP)

        # Emitter
        self.emitter = robot.getDevice("emitter")

        # InertialUnit
        self.inertialUnit = robot.getDevice("inertial unit")
        self.inertialUnit.enable(TIME_STEP)

        # Ultrasound Sensors
        self.ultrasound = []
        self.ultrasound.append(robot.getDevice("Sonar/Left"))
        self.ultrasound.append(robot.getDevice("Sonar/Right"))
        self.ultrasound[0].enable(TIME_STEP)
        self.ultrasound[1].enable(TIME_STEP)

        # Obstacle Avoidance Option
        self.obstacleAvoidance = True

        # Foot Bumpers
        self.bumpers = {
            "bumperLL": robot.getDevice("LFoot/Bumper/Left"),
            "bumperLR": robot.getDevice("LFoot/Bumper/Right"),
            "bumperRL": robot.getDevice("RFoot/Bumper/Left"),
            "bumperRR": robot.getDevice("RFoot/Bumper/Right"),
        }

        self.bumpers["bumperLL"].enable(TIME_STEP)
        self.bumpers["bumperLR"].enable(TIME_STEP)
        self.bumpers["bumperRL"].enable(TIME_STEP)
        self.bumpers["bumperRR"].enable(TIME_STEP)

        # Camera
        self.cameraTop = robot.getDevice("CameraTop")
        self.cameraBottom = robot.getDevice("CameraBottom")
        self.cameraTop.enable(TIME_STEP)
        self.cameraBottom.enable(TIME_STEP)

        # Load motion files
        self.motions = Motions()

        self.currentlyMoving = False
        self.motionQueue = [self.motions.standInit]
        self.startMotion()

    @abstractmethod
    def decideMotion(self, ballCoordinate, selfCoordinate) -> Motion:
        """Returns the next motion of the robot according to the role.

        Args:
            ballCoordinate (list): x, y, z coordinates of the ball.
            selfCoordinate (list): x, y coordinates of the robot.

        Returns:
            Motion: Decided motion.
        """
        pass

    def printSelf(self) -> None:
        print("Hello! This is robot ", self.name)

    def getSelfCoordinate(self) -> list:
        """Get the robot coordinate on the field.

        Returns:
            list: x, y coordinates.
        """
        gps_values = self.gps.getValues()
        return [gps_values[0], gps_values[1], gps_values[2]]

    def getRollPitchYaw(self) -> list:
        """Get the Roll, Pitch and Yaw angles of robot.

        Returns:
            list: Roll, Pitch and Yaw angles.
        """
        return self.inertialUnit.getRollPitchYaw()

    def isNewBallDataAvailable(self) -> bool:
        """Check if there is a new ball data available.

        Returns:
            bool: Is there any new ball data available?
        """
        return self.receiver.getQueueLength() > 0

    def getSupervisorData(self) -> None:
        """Get the latest supervisor data."""
        message = self.receiver.getData()
        self.supervisorData = struct.unpack("dd9cc24d", message)
        self.receiver.nextPacket()

    def getBallData(self) -> list:
        """Get the latest coordinates of the ball and robots.

        Returns:
            list: x, y coordinates.
        """

        return [self.supervisorData[0], self.supervisorData[1]]

    def getLeftSonarValue(self) -> float:
        """Get the left sonar distance.

        Returns:
            float: Distance (0 - 2.55)
        """

        return self.ultrasound[0].getValue()

    def getRightSonarValue(self) -> float:
        """Get the right sonar distance.

        Returns:
            float: Distance (0 - 2.55)
        """

        return self.ultrasound[1].getValue()

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

    def checkGoal(self) -> int:
        """Check if the goal scored.

        Returns:
            int: Returns 1 if the goal scored by the team,
                         -1 if the goal scored by the oponent,
                          0 otherwise.
        """
        ballCoordinate = self.getBallData()
        if abs(ballCoordinate[0]) > 4.5 and abs(ballCoordinate[1]) < 1.35:
            if 4.5 < ballCoordinate[0]:
                if self.name[0] == "R":
                    return 1
                else:
                    return -1

            else:
                if self.name[0] == "B":
                    return 1
                else:
                    return -1

        return 0

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

        if turningAngle > 50:
            return self.motions.turnLeft60
        elif turningAngle > 20:
            return self.motions.turnLeft30
        elif turningAngle < -50:
            return self.motions.turnRight60
        elif turningAngle < -30:
            return self.motions.turnRight40
        else:
            return None
