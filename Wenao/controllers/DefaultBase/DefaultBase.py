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
from threading import Thread

from controller import Keyboard, Motion, Robot
from PIL import Image


class ImageServer:
    def __init__(self, width, height, camera):
        self.camera = camera  # camera
        self.width = width
        self.height = height
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
                self.save_image(img, "output_image.jpg")
                self.queue.task_done()
            except queue.Empty:
                continue

            time.sleep(1 / 30.0)

    def save_image(self, img, filename):
        image = Image.frombytes("RGB", (self.width, self.height), bytes(img))
        image.save(filename)


class SoccerRobot(Robot):
    PHALANX_MAX = 8

    def __init__(self):
        Robot.__init__(self)
        self.currentlyPlaying = False
        self.supervisorData = None

        self.enableDevices()
        self.loadMotionFiles()

        self.TopCamServer = ImageServer(
            self.cameraTop.getWidth(), self.cameraTop.getHeight(), self.cameraTop
        )
        self.BottomCamServer = ImageServer(
            self.cameraBottom.getWidth(),
            self.cameraBottom.getHeight(),
            self.cameraBottom,
        )

    def run(self):
        self.handWave.setLoop(True)
        self.handWave.play()
        self.currentlyPlaying = self.handWave

        # until a key is pressed

        while self.step(self.timeStep) != -1:
            key = self.keyboard.getKey()

            if key == Keyboard.LEFT:
                self.startMotion(self.sideStepLeft)
            elif key == Keyboard.RIGHT:
                self.startMotion(self.sideStepRight)
            elif key == Keyboard.UP:
                self.startMotion(self.forwards)
            elif key == Keyboard.UP | Keyboard.SHIFT:
                self.startMotion(self.forwardSprint)
            elif key == Keyboard.DOWN:
                self.startMotion(self.backwards)
            elif key == Keyboard.LEFT | Keyboard.SHIFT:
                self.startMotion(self.turnLeft60)
            elif key == Keyboard.RIGHT | Keyboard.SHIFT:
                self.startMotion(self.turnRight60)
            elif key == ord("T"):
                self.startMotion(self.taiChi)
            elif key == ord("W"):
                self.startMotion(self.wipeForhead)
            elif key == ord("K"):
                self.startMotion(self.shoot)
            elif key == ord("P"):
                self.startMotion(self.longPass)
            elif key == Keyboard.UP | Keyboard.CONTROL:
                self.startMotion(self.standUpFront)
            elif key == Keyboard.DOWN | Keyboard.CONTROL:
                self.startMotion(self.standUpBack)

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

    # load motion files
    def loadMotionFiles(self):
        self.handWave = Motion("../../motions/HandWave.motion")
        self.forwards = Motion("../../motions/Forwards.motion")
        self.forwardsSprint = Motion("../../motions/ForwardsSprint.motion")
        self.forwards50 = Motion("../../motions/Forwards50.motion")
        self.backwards = Motion("../../motions/Backwards.motion")
        self.shoot = Motion("../../motions/Shoot.motion")
        self.rightShoot = Motion("../../motions/RightShoot.motion")
        self.longShoot = Motion("../../motions/LongPass.motion")
        self.leftSidePass = Motion("../../motions/SidePass_Left.motion")
        self.rightSidePass = Motion("../../motions/SidePass_Right.motion")
        self.sideStepLeft = Motion("../../motions/SideStepLeft.motion")
        self.sideStepRight = Motion("../../motions/SideStepRight.motion")
        self.standUpFromFront = Motion("../../motions/StandUpFromFront.motion")
        self.standUpFromBack = Motion("../../motions/StandUpFromBack.motion")
        self.turnLeft10 = Motion("../../motions/TurnLeft10.motion")
        self.turnLeft20 = Motion("../../motions/TurnLeft20.motion")
        self.turnLeft30 = Motion("../../motions/TurnLeft30.motion")
        self.turnLeft40 = Motion("../../motions/TurnLeft40.motion")
        self.turnLeft60 = Motion("../../motions/TurnLeft60.motion")
        self.turnLeft180 = Motion("../../motions/TurnLeft180.motion")
        self.turnRight10 = Motion("../../motions/TurnRight10.motion")
        self.turnRight10_V2 = Motion("../../motions/TurnRight10_V2.motion")
        self.turnRight40 = Motion("../../motions/TurnRight40.motion")
        self.turnRight60 = Motion("../../motions/TurnRight60.motion")
        self.standInit = Motion("../../motions/StandInit.motion")

    def startMotion(self, motion):
        # interrupt current motion
        if self.currentlyPlaying:
            self.currentlyPlaying.stop()

        # start new motion
        motion.play()
        self.currentlyPlaying = motion


def main():
    # Create the robot and initialize the camera
    robot = SoccerRobot()
    robot.run()


if __name__ == "__main__":
    main()
