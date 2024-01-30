"""
The Basic Robot behaviour and feature class.
All robots should be derived from this class.
"""
import os
import sys

currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)


from collections import defaultdict
from ultralytics import YOLO
import queue
import time
import math
from threading import Thread
from enum import Enum

import cv2  # Import OpenCV library
import numpy as np
from controller import Robot
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
        self.track_history = defaultdict(lambda: [])
        self.track = []
        self.model = YOLO("../Utils/best.pt",verbose=False)
        self.queue = queue.Queue(maxsize=3)
        self.thread = Thread(target=self.run, daemon=True)
        self.thread.start()
        
        # Ball distance Parameters
        self.ball_diameter = 0.48 #meters
        self.vertical_fov = 47.64 #degrees
        self.horizontal_fov = 60.97 #degrees

        self.distance_at_100_vfov = (self.ball_diameter / 2) / np.tan(np.deg2rad(self.vertical_fov/2))
        self.distance_at_100_hfov = (self.ball_diameter / 2) / np.tan(np.deg2rad(self.horizontal_fov/2))

    def send(self, image):
        self.queue.put(image)

    def stop(self):
        self.running = False
        self.thread.join()
        

       

        
    def ball_distance(self, ball_width):
        #focal_length = (ball_width * supervisor_distance) / self.ball_diameter
        #this is a paramter that has been tuned to webots and needs to be retuned for the real robot
        if self.position == "Bottom":
            focal_length = 80
   
        elif self.position == "Top":
            focal_length = 180
            
        distance = self.ball_diameter * focal_length / ball_width
        
        print(f'distance estimate: {min(4.5,distance)}') #stop noise from being larger than field
        
    def ball_heading(self, ball_center_x, ball_center_y):
        y = self.height - ball_center_y
        x = ball_center_x - self.width/2
        angle = np.arctan2(x, y)
        print(f'angle: {np.rad2deg(angle)}')
        

    def run(self):
        while self.running:
            try:
                img = self.queue.get(timeout=0.1)
                cvimg = np.frombuffer(img, dtype=np.uint8).reshape((self.height, self.width, 4))
                
                frame = cv2.cvtColor(cvimg, cv2.COLOR_BGR2RGB)
                # if self.position == "Bottom":
                if self.position == "Top":
                    results = self.model.predict(frame,verbose=False)[0]
                    threshold = 0.5
                    
                    for result in results.boxes.data.tolist():
                        x1, y1, x2, y2, score, class_id = result

                        if score > threshold:
                            if results.names[int(class_id)] == 'Ball':
                                ball_width = x2 - x1
                                ball_height = y2 - y1
                                ball_center_x = (x1 + x2) / 2
                                ball_center_y = (y1 + y2) / 2
                                self.ball_distance(ball_width)
                                self.ball_heading(ball_center_x, ball_center_y)   
                                


                            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 4)
                            cv2.putText(frame, results.names[int(class_id)].upper(), (int(x1), int(y1 - 10)),
                                        cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 255, 0), 3, cv2.LINE_AA)
                
                    cv2.imshow('FRAME',frame)
                    cv2.waitKey(1)
                self.queue.task_done()
            except queue.Empty:
                continue


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
            "RedDefenderLeft": [0, 0, 0],
            "RedDefenderRight": [0, 0, 0],
            "RedForward": [0, 0, 0],
            "BlueGoalkeeper": [0, 0, 0],
            "BlueDefenderLeft": [0, 0, 0],
            "BlueDefenderRight": [0, 0, 0],
            "BlueForward": [0, 0, 0],
        }
        self.ROassistantS = [
            "RedGoalkeeper",
            "RedDefenderLeft",
            "RedDefenderRight",
            "RedForward",
            "BlueGoalkeeper",
            "BlueDefenderLeft",
            "BlueDefenderRight",
            "BlueForward",
        ]

        self.AppState = RobotState.INIT
        self.StartLocation = [-4.36819, 0.0499058]

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
                    distance = Functions.calculateDistance(
                    self.getBallData(), self.getSelfPosition(self.robotName)
                )
   
                    print(f'real supervisor distance: {distance}')
                    
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

        for i, robot in enumerate(self.ROassistantS):
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

        if self.ultrasound[0].getValue() < 0.5:
            return self.motions.sideStepRightLoop
        elif self.ultrasound[1].getValue() < 0.5:
            return self.motions.sideStepLeftLoop

        # Get the current position
        currentPosition = self.getSelfPosition(self.robotName)

        match self.AppState:
            case RobotState.INIT:
                # Calculate the distance to the goal position
                distance = Functions.calculateDistance(
                    self.getBallData(), self.getSelfPosition(self.robotName)
                )

                # Calculate the angle to the target position
                dx, dy = (
                    self.getBallData()[0] - currentPosition[0],
                    self.getBallData()[1] - currentPosition[1],
                )
                targetAngle = math.degrees(math.atan2(dy, dx))

                # Get the robot's orientation angle
                robotAngle = math.degrees(self.getRollPitchYaw()[2])

                # Calculate the turn angle in the range [-180, 180)
                turnAngle = (targetAngle - robotAngle + 180) % 360 - 180

                if abs(turnAngle) > 10:
                    return self.getTurningMotion(turnAngle)

                if distance <= 0.2 and abs(turnAngle) < 10:
                    self.AppState = RobotState.BE_A_FORWARD
                    return self.motions.standInit

                return self.motions.forwardLoop

            case RobotState.LOOK_THE_BALL:
                # Calculate the angle to the target position
                dx, dy = (
                    self.getBallData()[0] - currentPosition[0],
                    self.getBallData()[1] - currentPosition[1],
                )
                targetAngle = math.degrees(math.atan2(dy, dx))

                # Get the robot's orientation angle
                robotAngle = math.degrees(self.getRollPitchYaw()[2])

                # Calculate the turn angle in the range [-180, 180)
                turnAngle = (targetAngle - robotAngle + 180) % 360 - 180

                if abs(turnAngle) > 10:
                    return self.getTurningMotion(turnAngle)

                    # Calculate the distance to the goal position
                distance = Functions.calculateDistance(
                    self.StartLocation, self.getSelfPosition(self.robotName)
                )

                if distance <= 0.2 and abs(turnAngle) < 10:
                    self.AppState = RobotState.BE_A_FORWARD

                return self.motions.standInit

            case RobotState.BE_A_FORWARD:
                distance = Functions.calculateDistance(
                    self.getBallData(), self.getSelfPosition(self.robotName)
                )

                if distance <= 0.2:
                    return self.motions.longShoot
            case _:
                self.AppState = RobotState.INIT


def main():
    # Create the robot and initialize the camera
    robot = SoccerRobot()
    robot.run()


if __name__ == "__main__":
    main()