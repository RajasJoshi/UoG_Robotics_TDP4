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
import struct
import time
from threading import Thread

import cv2  # Import OpenCV library
import numpy as np
from controller import Keyboard, Robot
from PIL import Image
from Utils.Consts import Motions
import matplotlib.pyplot as plt


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
        self.supervisorData = None
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
            # else:
            #     self.addMotionToQueue(self.motions.handWave)

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
        self.gps.enable(self.timeStep)

        # ultrasound sensors
        self.us = []
        usNames = ["Sonar/Left", "Sonar/Right"]
        for i in range(0, len(usNames)):
            self.us.append(self.getDevice(usNames[i]))
            self.us[i].enable(self.timeStep)

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


def main():
    # Create the robot and initialize the camera
    robot = SoccerRobot()
    robot.run()


if __name__ == "__main__":
    main()