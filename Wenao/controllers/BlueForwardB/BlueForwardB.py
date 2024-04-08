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
    PASS_TO_PLAYER = 3  # Add a new state for passing to a player


class SoccerRobot(Robot):
    PHALANX_MAX = 8

    def __init__(self, config):
        Robot.__init__(self)
        self.robotName = self.getName()
        self.currentlyPlaying = False
        self.config = config
        self.AppState = RobotState.INIT
        self.initial_ball_position = None
        self.game_started = False
        self.navigation_functions = NavigationFunctions(self)
        self.navigation_manager = SimpleNavigationManager(self, self.navigation_functions)

        self.bVisionUsed = config.getboolean("BlueTeam", "Vision")
        self.bAvoidCollision = config.getboolean("BlueTeam", "Avoidance")
        self.PlayerMode = config.get("BlueForwardB", "PlayerMode")
        self.Strategy = config.get("BlueForwardB", "Strategy")
        TargetPosition = config.get("BlueForwardB", "TargetPos")
        self.TargetPosition = list(map(float, TargetPosition.split(",")))
        StartLocation = config.get("BlueForwardB", "StartPos")
        self.StartLocation = list(map(float, StartLocation.split(",")))

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

        # Emitter
        self.emitter = self.getDevice("emitter")

        # keyboard
        self.keyboard = self.getKeyboard()
        self.keyboard.enable(10 * self.timeStep)

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

        collision = self.navigation_functions.detect_collision()
        if self.isNewMotionValid(collision):
            self.addMotionToQueue(collision)
            self.startMotion()

        # Get the current position
        currentSelfPosition = self.Supervisor.getSelfPosition()
        currentBallPosition = self.Supervisor.getBallData()

        # Calculate the ball distance to the robot position
        StartToRobotdist = Functions.calculateDistance(
            self.StartLocation, currentSelfPosition
        )
        
        # Calculate the ball distance to the robot position
        BallToRobotdist = Functions.calculateDistance(
            currentBallPosition, currentSelfPosition
        )
        
        # Calculate the ball distance to the goal position
        BallToGoaldist = Functions.calculateDistance(
            currentBallPosition, self.TargetPosition
        )

        # Get the goalkeeper position
        goalkeeperPosition = self.Supervisor.data["RedGoalkeeper"]

        # Get the player's position
        playerPosition = self.Supervisor.data["BlueForwardA"]

        # Get the robot's orientation angle
        robotAngle = np.degrees(self.getRollPitchYaw()[2])

        # If the initial ball position is not set, set it to the current position
        if self.initial_ball_position is None:
            self.initial_ball_position = currentBallPosition

        # Calculate the ball's velocity
        ball_distance = Functions.calculateDistance(currentBallPosition, self.initial_ball_position)

        # Update the previous ball position
        self.initial_ball_position = currentBallPosition

        if ball_distance > 0.0003:
            self.game_started = True

        #print(f"Blue Forward B Current State: {self.AppState}")
        #print(f"Game Started: {self.game_started}")

        # Add Strategies here
        match self.AppState:
            case RobotState.INIT:
                return self.navigation_manager.init_state(currentSelfPosition, self.StartLocation, StartToRobotdist, robotAngle)
            case RobotState.BE_A_FORWARD:
                return self.navigation_manager.be_a_forward_state(currentSelfPosition, currentBallPosition, self.TargetPosition, BallToRobotdist, BallToGoaldist, robotAngle)
            case RobotState.SCORE_GOAL:
                return self.navigation_manager.score_goal_state(currentSelfPosition, currentBallPosition, goalkeeperPosition, self.TargetPosition, BallToRobotdist, robotAngle)
            case RobotState.PASS_TO_PLAYER:
                return self.navigation_manager.pass_to_player_state(currentSelfPosition, playerPosition)
            case _:
                self.AppState = RobotState.INIT

    def calculatescore(self, player_name):
        """
        Calculate the score based on the number of enemy players around a team player.

        Args:
            player_name (str): The name of the player.

        Returns:
            float: The score.
        """
        # Get the player's position
        player_position = self.Supervisor.data[player_name]

        # Initialize the count of enemy players
        enemy_count = 0

        # Iterate over all the robots
        for robot_name in self.Supervisor.robot_list:
            # Skip if the robot is the player itself or a teammate
            if robot_name == player_name or robot_name.startswith("Blue"):
                continue

            # Get the robot's position
            robot_position = self.Supervisor.data[robot_name]

            # Calculate the distance between the player and the robot
            distance = Functions.calculateDistance(player_position, robot_position)

            # If the distance is less than a threshold, increment the enemy count
            if distance < 1:  # You can adjust this threshold as needed
                enemy_count += 1

        # Calculate the score as the inverse of the enemy count, add 1 to avoid division by zero
        score = 1 / (enemy_count + 1)

        return score
    
class NavigationFunctions:
    def __init__(self, soccer_robot):
        self.soccer_robot = soccer_robot
        self.motions = Motions()
        self.obstacles = []

    def detect_collision(self):
        for robot_name in self.soccer_robot.Supervisor.robot_list:
            if robot_name != self.soccer_robot.robotName: 
                robot_position = self.soccer_robot.Supervisor.data[robot_name]

                # Add the robot to the list of obstacles
                self.obstacles.append(robot_position)

                # Calculate the distance between robots
                self.robot_distance = Functions.calculateDistance(
                    robot_position, self.soccer_robot.Supervisor.getSelfPosition()
                )

                # Define a safe distance threshold
                safe_distance = 0.4
                ultrasound_threshold = 0.75

                # Check ultrasound values for collision detection
                ultrasound_left = self.soccer_robot.ultrasound[0].getValue()
                ultrasound_right = self.soccer_robot.ultrasound[1].getValue()

                # Check if the robot is too close to an obstacle
                if self.robot_distance < safe_distance:
                    # If both ultrasound sensors detect an obstacle, reverse
                    if ultrasound_right < ultrasound_threshold and ultrasound_left < ultrasound_threshold:
                        self.soccer_robot.interruptMotion()
                        return self.motions.backwards

                    # If only the right sensor detects an obstacle, sidestep left
                    elif ultrasound_right < ultrasound_threshold:
                        self.soccer_robot.interruptMotion()
                        return self.motions.sideStepLeft

                    # If only the left sensor detects an obstacle, sidestep right
                    elif ultrasound_right < ultrasound_threshold:
                        self.soccer_robot.interruptMotion()
                        return self.motions.sideStepRight
        # Return a default motion or None if no collision is detected
        return None
    
    def openGoalSection(self, goalkeeper_position, goal_position, goal_width):
            # Calculate the distance from the goalkeeper to the left and right posts
            left_post_position = (goal_position[0] - goal_width / 2, goal_position[1])
            right_post_position = (goal_position[0] + goal_width / 2, goal_position[1])
            dist_to_left_post = Functions.calculateDistance(goalkeeper_position, left_post_position)
            dist_to_right_post = Functions.calculateDistance(goalkeeper_position, right_post_position)

            # Determine which part of the goal the goalkeeper is closer to
            if dist_to_left_post < dist_to_right_post:
                return 'right'  # Goalkeeper is closer to the left post, so the right section is open
            elif dist_to_right_post < dist_to_left_post:
                return 'left'  # Goalkeeper is closer to the right post, so the left section is open
            else:
                return 'center'  # Goalkeeper is in the center, so both sides are equally open
            
    def calculateOptimalAngle(self, target_position, current_position):
        # Calculate the direct angle to the target
        direct_angle = Functions.calculateTargetAngle(target_position, current_position)

        # Calculate the angles to each obstacle
        obstacle_angles = [Functions.calculateTargetAngle(obstacle, current_position) for obstacle in self.obstacles]

        # Find the closest obstacle angle to the direct angle
        closest_obstacle_angle = min(obstacle_angles, key=lambda x: abs(x - direct_angle))

        # If the closest obstacle angle is too close to the direct angle, adjust the angle
        if abs(closest_obstacle_angle - direct_angle) < 15:
            if closest_obstacle_angle < direct_angle:
                return direct_angle + 10
            else:
                return direct_angle - 10

        # If there are no obstacles in the way, return the direct angle
        return direct_angle
    
    def detect_opponent(self, current_position):
        # Initialize the closest opponent distance and angle to large values
        closest_opponent_distance = float('inf')
        closest_opponent_angle = 0

        # Loop through the positions of the opponents
        for opponent_position in self.obstacles:
            # Calculate the distance and angle to the opponent
            opponent_distance = Functions.calculateDistance(current_position, opponent_position)
            opponent_angle = Functions.calculateTargetAngle(current_position, opponent_position)

            # If this opponent is closer than the previous closest opponent, update the closest opponent distance and angle
            if opponent_distance < closest_opponent_distance:
                closest_opponent_distance = opponent_distance
                closest_opponent_angle = opponent_angle

        return closest_opponent_distance, closest_opponent_angle

class SimpleNavigationManager:
    def __init__(self, soccer_robot, navigation_functions):
        self.soccer_robot = soccer_robot
        self.navigation_functions = navigation_functions
        self.motions = Motions()
        self.goal_width = 2.6

    def init_state(self, current_position, start_location, start_distance, robot_angle):  
        targetAngle_start = self.navigation_functions.calculateOptimalAngle(start_location, current_position)                
        turnAngle_start = Functions.calculateTurnAngle(targetAngle_start, robot_angle)
        
        if start_distance <= 0.2:
            self.soccer_robot.AppState = RobotState.BE_A_FORWARD
            return self.motions.standInit
        
        if abs(turnAngle_start) > 18:
            return self.soccer_robot.getTurningMotion(turnAngle_start)       
        return self.motions.forwardLoop 
    
    def be_a_forward_state(self, current_position, ball_position, target_position, ball_to_robot_dist, ball_to_goal_dist, robot_angle):
        if self.soccer_robot.Supervisor.data["ballOwner"][0] != "B" and ball_position[0] < 0:
            #Calculate the Robot's angle to the ball position
            targetAngle_ball = self.navigation_functions.calculateOptimalAngle(ball_position, current_position)
            turnAngle_ball = Functions.calculateTurnAngle(targetAngle_ball, robot_angle)

            if abs(turnAngle_ball) > 18:
                return self.soccer_robot.getTurningMotion(turnAngle_ball)

            if not self.soccer_robot.game_started:
                # Game has not started yet, stand and wait
                return self.motions.standInit
                    
            # Check if nao robot is away from the ball
            if ball_to_robot_dist > 0.2 and self.soccer_robot.game_started == True:
                return self.motions.forwardLoop
            
            # Detect the closest opponent
            closest_opponent_distance, closest_opponent_angle = self.navigation_functions.detect_opponent(current_position)\
            
            # Calculate the robot angle to the goal position
            targetAngle_goal = self.navigation_functions.calculateOptimalAngle(target_position, current_position)
            turnAngle_goal = Functions.calculateTurnAngle(targetAngle_goal, robot_angle)

            # Check if an opponent is nearby the current player
            if closest_opponent_distance < 0.8 and abs(closest_opponent_angle - robot_angle) < 30:
                # Trigger the pass to another player
                self.soccer_robot.AppState = RobotState.PASS_TO_PLAYER
                return

            # Check if the ball is near the goalpost
            elif ball_to_goal_dist <= 0.2 and abs(current_position[1]) > abs(target_position[1]):
                self.soccer_robot.interruptMotion()
                self.soccer_robot.AppState = RobotState.SCORE_GOAL
            
            elif abs(turnAngle_goal) > 18:
                if turnAngle_goal > 20:
                    return self.motions.rightSidePass
                elif turnAngle_goal < -30:
                    return self.motions.leftSidePass
                else:
                    return self.motions.shoot
            else:
                return self.motions.forwards50

    def score_goal_state(self, current_position, ball_position, goalkeeper_position, target_position, ball_to_robot_dist, robot_angle):
        # Calculate the Robot's angle to the goalkeeper position
        targetAngle = self.navigation_functions.calculateOptimalAngle(goalkeeper_position, current_position)
        turnAngle = Functions.calculateTurnAngle(targetAngle, robot_angle)

        # Check if nao robot is away from the ball
        if ball_to_robot_dist > 0.2:
            #Calculate the Robot's angle to the ball position
            targetAngle_ball = self.navigation_functions.calculateOptimalAngle(ball_position, current_position)
            turnAngle_ball = Functions.calculateTurnAngle(targetAngle_ball, robot_angle)

            if abs(turnAngle_ball) > 18:
                return self.soccer_robot.getTurningMotion(turnAngle_ball)
            return self.motions.forwardLoop
        
        # If the robot is facing the goal, kick
        elif abs(turnAngle) < 18:
            # Determine which part of the goal the goalkeeper is not covering
            open_goal_section = self.navigation_functions.openGoalSection(goalkeeper_position, target_position, goal_width=self.goal_width)
           
            # Aim for the open section of the goal
            if open_goal_section == 'left':
                return self.motions.rightSidePass
            elif open_goal_section == 'right':
                return self.motions.leftSidePass
            else:
                return self.motions.shoot

        # Otherwise, turn towards the goal
        elif turnAngle > 0:
            return self.motions.rightSidePass
        else:
            return self.motions.leftSidePass

    def pass_to_player_state(self, current_position, player_position):
        # Determine whether to pass to the right or left
        if player_position[1] > current_position[1]:
            self.soccer_robot.interruptMotion()
            # The player is to the right of the robot, so pass to the right
            if self.soccer_robot.isNewMotionValid(self.motions.rightSidePass):
                self.soccer_robot.addMotionToQueue(self.motions.rightSidePass)
                self.soccer_robot.startMotion()
            self.soccer_robot.AppState = RobotState.BE_A_FORWARD

        else:
            if self.soccer_robot.isNewMotionValid(self.motions.leftSidePass):
                self.soccer_robot.addMotionToQueue(self.motions.leftSidePass)
                self.soccer_robot.startMotion()
            self.soccer_robot.AppState = RobotState.BE_A_FORWARD


def main():
    # Create the robot and initialize the camera
    config = configparser.ConfigParser()
    config.read("../Utils/globalconfig.ini")
    robot = SoccerRobot(config)
    robot.run()


if __name__ == "__main__":
    main()
