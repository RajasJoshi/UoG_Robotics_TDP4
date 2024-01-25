"""
The Basic Robot behaviour and feature class.
All robots should be derived from this class.
"""
import os
import sys

currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)

import queue
import time
import math
from threading import Thread

import cv2  # Import OpenCV library
import numpy as np
from controller import Keyboard, Robot
from Utils.Consts import Motions
from Utils import Functions


class ImageServer:
    def __init__(self, width, height, camera, robot_name, position):
        self.camera = camera  # camera
        self.width = width
        self.height = height
        self.robot_name = robot_name
        self.position = position
        self.running = True

        self.queue = queue.Queue(maxsize=3)
        self.thread = Thread(target=self.run, daemon=True)
        self.thread.start()

    def send(self, image):
        self.queue.put(image)

    def stop(self):
        self.running = False
        self.thread.join()

    def run(self):
        while self.running:
            try:
                img = self.queue.get(timeout=0.1)
                cvimg = np.frombuffer(img, dtype=np.uint8).reshape(
                    (self.height, self.width, 4)
                )
                # Display the image using OpenCV
                cv2.imshow(f"Image Stream - {self.robot_name} - {self.position}", cvimg)
                cv2.waitKey(1)
                self.queue.task_done()
            except queue.Empty:
                continue

            time.sleep(1 / 30.0)


class SoccerRobot(Robot):
    PHALANX_MAX = 8

    def __init__(self):
        Robot.__init__(self)
        self.robotName = self.getName()
        self.currentlyPlaying = False
        self.supervisorDataPos = {
            "ballPosition": [0.0, 0.0, 0.0],
            "RedGoalkeeper": [0.0, 0.0, 0.0],
            "RedDefenderLeft": [0.0, 0.0, 0.0],
            "RedDefenderRight": [0.0, 0.0, 0.0],
            "RedForward": [0.0, 0.0, 0.0],
            "BlueGoalkeeper": [0.0, 0.0, 0.0],
            "BlueDefenderLeft": [0.0, 0.0, 0.0],
            "BlueDefenderRight": [0.0, 0.0, 0.0],
            "BlueForward": [0.0, 0.0, 0.0],
            # Add more keys for other data as needed
        }

        self.supervisorDataRot = {
            "RedGoalkeeper": [0.0, 0.0, 0.0],
            "RedDefenderLeft": [0.0, 0.0, 0.0],
            "RedDefenderRight": [0.0, 0.0, 0.0],
            "RedForward": [0.0, 0.0, 0.0],
            "BlueGoalkeeper": [0.0, 0.0, 0.0],
            "BlueDefenderLeft": [0.0, 0.0, 0.0],
            "BlueDefenderRight": [0.0, 0.0, 0.0],
            "BlueForward": [0.0, 0.0, 0.0],
            # Add more keys for other data as needed
        }

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
        # until a key is pressed
        try:
            while self.step(self.timeStep) != -1:
                self.clearMotionQueue()

                key = self.keyboard.getKey()

                if key == Keyboard.LEFT:
                    self.addMotionToQueue(self.motions.sideStepLeft)
                elif key == Keyboard.RIGHT:
                    self.addMotionToQueue(self.motions.sideStepRight)
                elif key == Keyboard.UP:
                    self.addMotionToQueue(self.motions.forwards)
                elif key == Keyboard.DOWN:
                    self.addMotionToQueue(self.motions.backwards)
                elif key == Keyboard.LEFT | Keyboard.SHIFT:
                    self.addMotionToQueue(self.motions.turnLeft60)
                elif key == Keyboard.RIGHT | Keyboard.SHIFT:
                    self.addMotionToQueue(self.motions.turnRight60)
                elif key == Keyboard.UP | Keyboard.CONTROL:
                    self.addMotionToQueue(self.motions.standUpFromFront)
                elif key == Keyboard.DOWN | Keyboard.CONTROL:
                    self.addMotionToQueue(self.motions.standUpFromBack)
                elif key == Keyboard.ALT:
                    self.addMotionToQueue(self.motions.shoot)
                elif key == Keyboard.ALT | Keyboard.SHIFT:
                    self.addMotionToQueue(self.motions.longShoot)

                self.startMotion()

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

    def printSelf(self) -> None:
        print("Hello! This is robot ", self.name)

    def getSelfPosition(self, robotName) -> list:
        """Get the robot coordinate on the field.

        Returns:
            list: x, y coordinates.
        """
        for key, value in self.supervisorDataPos.items():
            # Compare search_string with the keys (case-insensitive)
            if robotName.lower() == key.lower():
                return value

    def getSelfOrientation(self, robotName) -> list:
        """Get the robot coordinate on the field.

        Returns:
            list: x, y coordinates.
        """
        for key, value in self.supervisorDataRot.items():
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
        self.supervisorDataPos["ballPosition"] = [
            float(values[0]),
            float(values[1]),
            float(values[2]),
        ]
        self.supervisorDataPos["RedGoalkeeper"] = [
            float(values[3]),
            float(values[4]),
            float(values[5]),
        ]
        self.supervisorDataPos["RedDefenderLeft"] = [
            float(values[6]),
            float(values[7]),
            float(values[8]),
        ]
        self.supervisorDataPos["RedDefenderRight"] = [
            float(values[9]),
            float(values[10]),
            float(values[11]),
        ]
        self.supervisorDataPos["RedForward"] = [
            float(values[12]),
            float(values[13]),
            float(values[14]),
        ]
        self.supervisorDataPos["BlueGoalkeeper"] = [
            float(values[15]),
            float(values[16]),
            float(values[17]),
        ]
        self.supervisorDataPos["BlueDefenderLeft"] = [
            float(values[18]),
            float(values[19]),
            float(values[20]),
        ]
        self.supervisorDataPos["BlueDefenderRight"] = [
            float(values[21]),
            float(values[22]),
            float(values[23]),
        ]
        self.supervisorDataPos["BlueForward"] = [
            float(values[24]),
            float(values[25]),
            float(values[26]),
        ]
        self.supervisorDataRot["RedGoalkeeper"] = [
            float(values[27]),
            float(values[28]),
            float(values[29]),
        ]
        self.supervisorDataRot["RedDefenderLeft"] = [
            float(values[30]),
            float(values[31]),
            float(values[32]),
        ]
        self.supervisorDataRot["RedDefenderRight"] = [
            float(values[33]),
            float(values[34]),
            float(values[35]),
        ]
        self.supervisorDataRot["RedForward"] = [
            float(values[36]),
            float(values[37]),
            float(values[38]),
        ]
        self.supervisorDataRot["BlueGoalkeeper"] = [
            float(values[39]),
            float(values[40]),
            float(values[41]),
        ]
        self.supervisorDataRot["BlueDefenderLeft"] = [
            float(values[42]),
            float(values[43]),
            float(values[44]),
        ]
        self.supervisorDataRot["BlueDefenderRight"] = [
            float(values[45]),
            float(values[46]),
            float(values[47]),
        ]
        self.supervisorDataRot["BlueForward"] = [
            float(values[48]),
            float(values[49]),
            float(values[50]),
        ]

        # Extract additional string (assuming it's the last element)
        ballOwner = values[-1]

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

        if self.ultrasound[0].getValue() < 0.5:
            return self.motions.sideStepRightLoop
        elif self.ultrasound[1].getValue() < 0.5:
            return self.motions.sideStepLeftLoop

        # Calculate the difference between the current and target positions
        difference_x = self.getBallData()[0] - self.getSelfPosition(self.robotName)[0]
        difference_y = self.getBallData()[1] - self.getSelfPosition(self.robotName)[1]

        # Calculate the angle to the target position
        targetAngle = math.degrees(math.atan2(difference_y, difference_x))
        robotAngle = math.degrees(self.getRollPitchYaw()[2])
        turnAngle = targetAngle - robotAngle
        turnAngle = (turnAngle + 180) % 360 - 180

        if abs(turnAngle) > 10:
            if turnAngle > 90:
                return self.motions.turnLeft180
            elif turnAngle > 50:
                return self.motions.turnLeft60
            elif turnAngle > 30:
                return self.motions.turnLeft40
            elif turnAngle > 20:
                return self.motions.turnLeft30
            elif turnAngle > 10:
                return self.motions.turnLeft20
            elif turnAngle < -50:
                return self.motions.turnRight60
            elif turnAngle < -30:
                return self.motions.turnRight40
            elif turnAngle < -10:
                return self.motions.turnRight20

        return self.motions.standInit


def main():
    # Create the robot and initialize the camera
    robot = SoccerRobot()
    robot.run()


if __name__ == "__main__":
    main()
