# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Example of Python controller for Nao robot.
   This demonstrates how to access sensors and actuators"""

from controller import Keyboard, Motion, Robot


class Nao(Robot):
    PHALANX_MAX = 8

    # load motion files
    def loadMotionFiles(self):
        self.handWave = Motion("../../motions/HandWave.motion")
        self.forwards = Motion("../../motions/Forwards50.motion")
        self.forwardSprint = Motion("../../motions/ForwardsSprint.motion")
        self.backwards = Motion("../../motions/Backwards.motion")
        self.sideStepLeft = Motion("../../motions/SideStepLeft.motion")
        self.sideStepRight = Motion("../../motions/SideStepRight.motion")
        self.turnLeft60 = Motion("../../motions/TurnLeft60.motion")
        self.turnRight60 = Motion("../../motions/TurnRight60.motion")
        self.shoot = Motion("../../motions/Shoot.motion")
        self.longPass = Motion("../../motions/LongPass.motion")
        self.standUpFront = Motion("../../motions/StandUpFromFront.motion")
        self.standUpBack = Motion("../../motions/StandUpFromBack.motion")

    def startMotion(self, motion):
        # interrupt current motion
        if self.currentlyPlaying:
            self.currentlyPlaying.stop()

        # start new motion
        motion.play()
        self.currentlyPlaying = motion

    # the accelerometer axes are oriented as on the real robot
    # however the sign of the returned values may be opposite
    def printAcceleration(self):
        acc = self.accelerometer.getValues()
        print("----------accelerometer----------")
        print("acceleration: [ x y z ] = [%f %f %f]" % (acc[0], acc[1], acc[2]))

    # the gyro axes are oriented as on the real robot
    # however the sign of the returned values may be opposite
    def printGyro(self):
        vel = self.gyro.getValues()
        print("----------gyro----------")
        # z value is meaningless due to the orientation of the Gyro
        print("angular velocity: [ x y ] = [%f %f]" % (vel[0], vel[1]))

    def printGps(self):
        p = self.gps.getValues()
        print("----------gps----------")
        print("position: [ x y z ] = [%f %f %f]" % (p[0], p[1], p[2]))

    # the InertialUnit roll/pitch angles are equal to naoqi's AngleX/AngleY
    def printInertialUnit(self):
        rpy = self.inertialUnit.getRollPitchYaw()
        print("----------inertial unit----------")
        print("roll/pitch/yaw: [%f %f %f]" % (rpy[0], rpy[1], rpy[2]))

    def printFootSensors(self):
        fsv = []  # force sensor values

        fsv.append(self.fsr[0].getValues())
        fsv.append(self.fsr[1].getValues())

        left = []
        right = []

        newtonsLeft = 0
        newtonsRight = 0

        # The coefficients were calibrated against the real
        # robot so as to obtain realistic sensor values.
        left.append(
            fsv[0][2] / 3.4 + 1.5 * fsv[0][0] + 1.15 * fsv[0][1]
        )  # Left Foot Front Left
        left.append(
            fsv[0][2] / 3.4 + 1.5 * fsv[0][0] - 1.15 * fsv[0][1]
        )  # Left Foot Front Right
        left.append(
            fsv[0][2] / 3.4 - 1.5 * fsv[0][0] - 1.15 * fsv[0][1]
        )  # Left Foot Rear Right
        left.append(
            fsv[0][2] / 3.4 - 1.5 * fsv[0][0] + 1.15 * fsv[0][1]
        )  # Left Foot Rear Left

        right.append(
            fsv[1][2] / 3.4 + 1.5 * fsv[1][0] + 1.15 * fsv[1][1]
        )  # Right Foot Front Left
        right.append(
            fsv[1][2] / 3.4 + 1.5 * fsv[1][0] - 1.15 * fsv[1][1]
        )  # Right Foot Front Right
        right.append(
            fsv[1][2] / 3.4 - 1.5 * fsv[1][0] - 1.15 * fsv[1][1]
        )  # Right Foot Rear Right
        right.append(
            fsv[1][2] / 3.4 - 1.5 * fsv[1][0] + 1.15 * fsv[1][1]
        )  # Right Foot Rear Left

        for i in range(0, len(left)):
            left[i] = max(min(left[i], 25), 0)
            right[i] = max(min(right[i], 25), 0)
            newtonsLeft += left[i]
            newtonsRight += right[i]

        print("----------foot sensors----------")
        print("+ left ---- right +")
        print("+-------+ +-------+")
        print(
            "|"
            + str(round(left[0], 1))
            + "  "
            + str(round(left[1], 1))
            + "| |"
            + str(round(right[0], 1))
            + "  "
            + str(round(right[1], 1))
            + "|  front"
        )
        print("| ----- | | ----- |")
        print(
            "|"
            + str(round(left[3], 1))
            + "  "
            + str(round(left[2], 1))
            + "| |"
            + str(round(right[3], 1))
            + "  "
            + str(round(right[2], 1))
            + "|  back"
        )
        print("+-------+ +-------+")
        print(
            "total: %f Newtons, %f kilograms"
            % ((newtonsLeft + newtonsRight), ((newtonsLeft + newtonsRight) / 9.81))
        )

    def printFootBumpers(self):
        ll = self.lfootlbumper.getValue()
        lr = self.lfootrbumper.getValue()
        rl = self.rfootlbumper.getValue()
        rr = self.rfootrbumper.getValue()
        print("----------foot bumpers----------")
        print("+ left ------ right +")
        print("+--------+ +--------+")
        print("|" + str(ll) + "  " + str(lr) + "| |" + str(rl) + "  " + str(rr) + "|")
        print("|        | |        |")
        print("|        | |        |")
        print("+--------+ +--------+")

    def printUltrasoundSensors(self):
        dist = []
        for i in range(0, len(self.us)):
            dist.append(self.us[i].getValue())

        print("-----ultrasound sensors-----")
        print("left: %f m, right %f m" % (dist[0], dist[1]))

    def printCameraImage(self, camera):
        scaled = 2  # defines by which factor the image is subsampled
        width = camera.getWidth()
        height = camera.getHeight()

        # read rgb pixel values from the camera
        image = camera.getImage()

        print("----------camera image (gray levels)---------")
        print(
            "original resolution: %d x %d, scaled to %d x %f"
            % (width, height, width / scaled, height / scaled)
        )

        for y in range(0, height // scaled):
            line = ""
            for x in range(0, width // scaled):
                gray = (
                    camera.imageGetGray(image, width, x * scaled, y * scaled) * 9 / 255
                )  # rescale between 0 and 9
                line = line + str(int(gray))
            print(line)

    def setAllLedsColor(self, rgb):
        # these leds take RGB values
        for i in range(0, len(self.leds)):
            self.leds[i].set(rgb)

        # ear leds are single color (blue)
        # and take values between 0 - 255
        self.leds[5].set(rgb & 0xFF)
        self.leds[6].set(rgb & 0xFF)

    def setHandsAngle(self, angle):
        for i in range(0, self.PHALANX_MAX):
            clampedAngle = angle
            if clampedAngle > self.maxPhalanxMotorPosition[i]:
                clampedAngle = self.maxPhalanxMotorPosition[i]
            elif clampedAngle < self.minPhalanxMotorPosition[i]:
                clampedAngle = self.minPhalanxMotorPosition[i]

            if len(self.rphalanx) > i and self.rphalanx[i] is not None:
                self.rphalanx[i].setPosition(clampedAngle)
            if len(self.lphalanx) > i and self.lphalanx[i] is not None:
                self.lphalanx[i].setPosition(clampedAngle)

    def printHelp(self):
        print("----------nao_demo_python----------")
        print("Use the keyboard to control the robots (one at a time)")
        print("(The 3D window need to be focused)")
        print("[Up][Down]: move one step forward/backwards")
        print("[<-][->]: side step left/right")
        print("[Shift] + [<-][->]: turn left/right")
        print("[Shift] + [Up]: sprint forward")
        print("[U]: print ultrasound sensors")
        print("[A]: print accelerometers")
        print("[G]: print gyros")
        print("[S]: print gps")
        print("[I]: print inertial unit (roll/pitch/yaw)")
        print("[F]: print foot sensors")
        print("[B]: print foot bumpers")
        print("[Home][End]: print scaled top/bottom camera image")
        print("[PageUp][PageDown]: open/close hands")
        print("[7][8][9]: change all leds RGB color")
        print("[0]: turn all leds off")
        print("[T]: perform Tai chi movements")
        print("[W]: wipe its forehead")
        print("[K]: Kick")
        print("[P]: Pass")
        print("[Ctrl] + [Up]: stand up from front")
        print("[Ctrl] + [Down]: stand up from back")
        print("[H]: print this help message")

    def findAndEnableDevices(self):
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

        # gyro
        self.gyro = self.getDevice("gyro")
        self.gyro.enable(4 * self.timeStep)

        # gps
        self.gps = self.getDevice("gps")
        self.gps.enable(4 * self.timeStep)

        # inertial unit
        self.inertialUnit = self.getDevice("inertial unit")
        self.inertialUnit.enable(self.timeStep)

        # ultrasound sensors
        self.us = []
        usNames = ["Sonar/Left", "Sonar/Right"]
        for i in range(0, len(usNames)):
            self.us.append(self.getDevice(usNames[i]))
            self.us[i].enable(self.timeStep)

        # foot sensors
        self.fsr = []
        fsrNames = ["LFsr", "RFsr"]
        for i in range(0, len(fsrNames)):
            self.fsr.append(self.getDevice(fsrNames[i]))
            self.fsr[i].enable(self.timeStep)

        # foot bumpers
        self.lfootlbumper = self.getDevice("LFoot/Bumper/Left")
        self.lfootrbumper = self.getDevice("LFoot/Bumper/Right")
        self.rfootlbumper = self.getDevice("RFoot/Bumper/Left")
        self.rfootrbumper = self.getDevice("RFoot/Bumper/Right")
        self.lfootlbumper.enable(self.timeStep)
        self.lfootrbumper.enable(self.timeStep)
        self.rfootlbumper.enable(self.timeStep)
        self.rfootrbumper.enable(self.timeStep)

        # there are 7 controlable LED groups in Webots
        self.leds = []
        self.leds.append(self.getDevice("ChestBoard/Led"))
        self.leds.append(self.getDevice("RFoot/Led"))
        self.leds.append(self.getDevice("LFoot/Led"))
        self.leds.append(self.getDevice("Face/Led/Right"))
        self.leds.append(self.getDevice("Face/Led/Left"))
        self.leds.append(self.getDevice("Ears/Led/Right"))
        self.leds.append(self.getDevice("Ears/Led/Left"))

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

        # keyboard
        self.keyboard = self.getKeyboard()
        self.keyboard.enable(10 * self.timeStep)

    def __init__(self):
        Robot.__init__(self)
        self.currentlyPlaying = False

        # initialize stuff
        self.findAndEnableDevices()
        self.loadMotionFiles()
        self.printHelp()

    def run(self):
        self.handWave.setLoop(True)
        self.handWave.play()
        self.currentlyPlaying = self.handWave

        # until a key is pressed
        key = -1
        while robot.step(self.timeStep) != -1:
            key = self.keyboard.getKey()
            if key > 0:
                break

        self.handWave.setLoop(False)

        while True:
            key = self.keyboard.getKey()

            match key:
                case Keyboard.LEFT:
                    self.startMotion(self.sideStepLeft)
                case Keyboard.RIGHT:
                    self.startMotion(self.sideStepRight)
                case Keyboard.UP:
                    self.startMotion(self.forwards)
                case Keyboard.UP | Keyboard.SHIFT:
                    self.startMotion(self.forwardSprint)
                case Keyboard.DOWN:
                    self.startMotion(self.backwards)
                case Keyboard.LEFT | Keyboard.SHIFT:
                    self.startMotion(self.turnLeft60)
                case Keyboard.RIGHT | Keyboard.SHIFT:
                    self.startMotion(self.turnRight60)
                case ord("A"):
                    self.printAcceleration()
                case ord("G"):
                    self.printGyro()
                case ord("S"):
                    self.printGps()
                case ord("I"):
                    self.printInertialUnit()
                case ord("F"):
                    self.printFootSensors()
                case ord("B"):
                    self.printFootBumpers()
                case ord("U"):
                    self.printUltrasoundSensors()
                case ord("K"):
                    self.startMotion(self.shoot)
                case ord("P"):
                    self.startMotion(self.longPass)
                case Keyboard.UP | Keyboard.CONTROL:
                    self.startMotion(self.standUpFront)
                case Keyboard.DOWN | Keyboard.CONTROL:
                    self.startMotion(self.standUpBack)
                case Keyboard.HOME:
                    self.printCameraImage(self.cameraTop)
                case Keyboard.END:
                    self.printCameraImage(self.cameraBottom)
                case Keyboard.PAGEUP:
                    self.setHandsAngle(0.96)
                case Keyboard.PAGEDOWN:
                    self.setHandsAngle(0.0)
                case ord("7"):
                    self.setAllLedsColor(0xFF0000)  # red
                case ord("8"):
                    self.setAllLedsColor(0x00FF00)  # green
                case ord("9"):
                    self.setAllLedsColor(0x0000FF)  # blue
                case ord("0"):
                    self.setAllLedsColor(0x000000)  # off
                case ord("H"):
                    self.printHelp()
                case _:
                    print("Wrong Key")

            if robot.step(self.timeStep) == -1:
                break
