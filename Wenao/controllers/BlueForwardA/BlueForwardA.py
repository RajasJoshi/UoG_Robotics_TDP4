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
    BE_A_FORWARD = 1
    SCORE_GOAL = 2


class SoccerRobot(Robot):
    PHALANX_MAX = 8

    def __init__(self, config):
        Robot.__init__(self)
        self.robotName = self.getName()
        self.currentlyPlaying = False
        self.config = config
        self.AppState = RobotState.INIT

        self.StartLocation = [0.8537437331388491, -0.05814318321955188]
        self.TargetgoalPosition = [-3.25978, 0.0196566]
        self.bVisionUsed = config.getboolean("BlueTeam", "Vision")
        self.enableDevices()
        # Load motion files
        self.motions = Motions()
        self.currentlyMoving = False
        self.motionQueue = [self.motions.standInit]
        self.startMotion()

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
        if self.bVisionUsed:
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

        # Receiver
        self.receiver = self.getDevice("receiver")
        self.receiver.enable(self.timeStep)

        # ultrasound sensors
        self.ultrasound = []
        self.ultrasound.append(self.getDevice("Sonar/Left"))
        self.ultrasound.append(self.getDevice("Sonar/Right"))
        self.ultrasound[0].enable(self.timeStep)
        self.ultrasound[1].enable(self.timeStep)


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

    def handle_state(self,distance, turnAngle, state_to_switch, motion_to_return):
        if distance <= 0.2:
            self.AppState = state_to_switch
            return self.motions.standInit
        elif abs(turnAngle) > 10:
            return self.getTurningMotion(turnAngle)
        else:  
            return motion_to_return 

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
        
        collision = self.detect_collision()
        if self.isNewMotionValid(collision):
            self.addMotionToQueue(collision)
            self.startMotion()

        # Get the current position
        currentSelfPosition = self.Supervisor.getSelfPosition()
        currentBallPosition = self.Supervisor.getBallData()

        # Get the robot's orientation angle
        robotAngle = np.degrees(self.getRollPitchYaw()[2])

        # Calculate the distance to the starting position
        start_distance = Functions.calculateDistance(
            self.StartLocation, currentSelfPosition
        )

        # Calculate the ball distance to the goal position
        ball_distance = Functions.calculateDistance(
            self.TargetgoalPosition, currentBallPosition
        )

        # Calculate the robot's distance to the ball position
        robot_distance = Functions.calculateDistance(
            currentBallPosition, currentSelfPosition
        )

        #print(self.AppState)

        match self.AppState:
            case RobotState.INIT:          
                # Calculate the Robot's angle to the start position
                targetAngle_start = Functions.calculate_target_angle(self.StartLocation, currentSelfPosition)                
                turnAngle_start = Functions.calculate_turn_angle(targetAngle_start, robotAngle)

                if start_distance <= 0.2:
                    self.AppState = RobotState.BE_A_FORWARD
                    return self.motions.standInit
                
                elif abs(turnAngle_start) > 18:
                    return self.getTurningMotion(turnAngle_start)       
                return self.motions.forwardLoop

            case RobotState.BE_A_FORWARD:   
                # Calculate the robot angle to the goal position
                targetAngle_goal = Functions.calculate_target_angle(self.TargetgoalPosition, currentSelfPosition)
                turnAngle_goal = Functions.calculate_turn_angle(targetAngle_goal, robotAngle)
                        
                # Check if nao robot is away from the ball
                if robot_distance > 0.2:
                     #Calculate the Robot's angle to the ball position
                    targetAngle_ball = Functions.calculate_target_angle(currentBallPosition, currentSelfPosition)
                    turnAngle_ball = Functions.calculate_turn_angle(targetAngle_ball, robotAngle)

                    if abs(turnAngle_ball) > 18:
                        return self.getTurningMotion(turnAngle_ball)
                    return self.motions.forwardLoop

                # Check if the ball is near the goalpost
                elif ball_distance <= 0.2 and abs(currentSelfPosition[1]) > abs(self.TargetgoalPosition[1]):
                    self.interruptMotion()
                    self.AppState = RobotState.SCORE_GOAL

                # if detect another robot, decided to pass left or right
                # elif self.robot_distance < 0.8:
                #     return self.motions.rightSidePass
                
                elif abs(turnAngle_goal) > 18:
                    if turnAngle_goal > 20:
                        return self.motions.rightSidePass
                    elif turnAngle_goal < -30:
                        return self.motions.leftSidePass
                    else:
                        return self.motions.shoot
                else:
                    return self.motions.forwards50
                             
            case RobotState.SCORE_GOAL:
                # Calculate the Robot's angle to the goalkeeper position
                targetAngle = Functions.calculate_target_angle(self.Supervisor.data["RedGoalkeeper"], currentSelfPosition)
                turnAngle = Functions.calculate_turn_angle(targetAngle, robotAngle)

                # Check if nao robot is away from the ball
                if robot_distance > 0.2:
                     #Calculate the Robot's angle to the ball position
                    targetAngle_ball = Functions.calculate_target_angle(currentBallPosition, currentSelfPosition)
                    turnAngle_ball = Functions.calculate_turn_angle(targetAngle_ball, robotAngle)

                    if abs(turnAngle_ball) > 18:
                        return self.getTurningMotion(turnAngle_ball)
                    return self.motions.forwardLoop
                
                # If the robot is facing the goal, kick
                elif abs(turnAngle) < 18:
                    return self.motions.shoot

                # Otherwise, turn towards the goal
                elif turnAngle > 0:
                    return self.motions.rightSidePass
                else:
                    return self.motions.leftSidePass
                
            # case RobotState.BE_A_FORWARD:
            #     # waypoints values can be change according to strategy
            #     waypoints = [(-0.6, -1.6), (-1.7, -1.6), (-3, -0.4)]

            #     # Calculate the robot's distance to the ball position
            #     robot_distance = Functions.calculateDistance(
            #         currentBallPosition, currentSelfPosition
            #     )

            #     for i in range(len(waypoints)):
            #         print(i)
            #         waypoint_distance = Functions.calculateDistance(waypoints[i], currentSelfPosition)

            #         if i + 1 < len(waypoints):
            #             targetAngle = np.degrees(
            #                 np.arctan2(
            #                     waypoints[i + 1][1] - currentSelfPosition[1],
            #                     waypoints[i + 1][0] - currentSelfPosition[0],
            #                 )
            #             )
            #             robotAngle = np.degrees(self.getRollPitchYaw()[2])
            #             turnAngle = (targetAngle - robotAngle + 180) % 360 - 180

            #         #Check if nao robot is away from the ball
            #         if robot_distance > 0.2:
            #             self.AppState = RobotState.LOOK_THE_BALL
            #             return self.motions.forwardLoop
                    
            #         elif waypoint_distance < 0.2:
            #             if i == len(waypoints) - 1:
            #                 self.AppState = RobotState.LOOK_GOAL
            #                 return self.motions.shoot
                        
            #         elif abs(turnAngle) > 10:
            #             if turnAngle > 20:
            #                 return self.motions.rightSidePass
            #             elif turnAngle < -30:
            #                 return self.motions.leftSidePass
            #         else:
            #             return self.motions.forwardLoop  

            case _:
                self.AppState = RobotState.INIT              


    def detect_collision(self):
        for robot_name in self.Supervisor.robot_list:
            if robot_name != self.robotName: 
                robot_position = self.Supervisor.data[robot_name]

                # Calculate the distance between robots
                self.robot_distance = Functions.calculateDistance(
                    robot_position, self.Supervisor.getSelfPosition()
                )

                # Define a safe distance threshold
                safe_distance = 0.4
                ultrasound_threshold = 0.75

                # Check ultrasound values for collision detection
                ultrasound_left = self.ultrasound[0].getValue()
                ultrasound_right = self.ultrasound[1].getValue()

                # Check if the robot is too close to an obstacle
                if self.robot_distance < safe_distance:
                    # If both ultrasound sensors detect an obstacle, reverse
                    if ultrasound_right < ultrasound_threshold and ultrasound_left < ultrasound_threshold:
                        self.interruptMotion()
                        return self.motions.backwards

                    # If only the right sensor detects an obstacle, sidestep left
                    elif ultrasound_right < ultrasound_threshold:
                        self.interruptMotion()
                        return self.motions.sideStepLeft

                    # If only the left sensor detects an obstacle, sidestep right
                    elif ultrasound_right < ultrasound_threshold:
                        self.interruptMotion()
                        return self.motions.sideStepRight
        # Return a default motion or None if no collision is detected
        return None


def main():
    # Create the robot and initialize the camera
    config = configparser.ConfigParser()
    config.read("../Utils/globalconfig.ini")
    robot = SoccerRobot(config)
    robot.run()


if __name__ == "__main__":
    main()
