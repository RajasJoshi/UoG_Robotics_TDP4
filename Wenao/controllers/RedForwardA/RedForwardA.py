"""
The Basic Robot behaviour and feature class.
All robots should be derived from this class.
"""

import os
import sys
import random

from matplotlib import animation

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import configparser
from enum import Enum
import numpy as np
from controller import Robot
from Utils import Functions
from Utils.Consts import Motions
from Utils.ImageServer import ImageServer
from Utils.ProcessSupervisor import SupervisorData
from Utils.pathPlanning import aStar
import matplotlib.pyplot as plt


class RobotState(Enum):
    START_POSITION = 0
    TRACK_BALL = 1
    BE_A_FORWARD = 2
    SCORE_GOAL = 3
    PASS_TO_PLAYER = 4  # Add a new state for passing to a player


class SoccerRobot(Robot):
    PHALANX_MAX = 8

    def __init__(self, config):
        Robot.__init__(self)
        self.robotName = self.getName()
        self.currentlyPlaying = False
        self.config = config
        self.AppState = RobotState.START_POSITION
        self.initial_ball_position = None
        self.game_started = False
        red_team_starts, blue_team_starts = Functions.shared_state.game_start()
        self.red_team_start = red_team_starts
        self.blue_team_start = blue_team_starts
        self.navigation_manager = PathPlanningManager(self)
        self.robot_state_manager = RobotStateManager(self, self.navigation_manager)

        self.bVisionUsed = True
        self.bAvoidCollision = config.getboolean("RedTeam", "Avoidance")
        self.PlayerMode = config.get("RedForwardA", "PlayerMode")
        self.Strategy = config.get("RedForwardA", "Strategy")
        TargetPosition = config.get("RedForwardA", "TargetPos")
        self.TargetPosition = list(map(float, TargetPosition.split(",")))
        StartLocation = config.get("RedForwardA", "StartPos")
        self.StartLocation = list(map(float, StartLocation.split(",")))

        # Write the result to a file
        with open('globalconfig.ini', 'w') as f:
            f.write(f'{red_team_starts},{blue_team_starts}')

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

        # ultrasonic sensors
        self.ultrasound = []
        self.ultrasound.append(self.getDevice("Sonar/Left"))
        self.ultrasound.append(self.getDevice("Sonar/Right"))
        self.ultrasound[0].enable(self.timeStep)
        self.ultrasound[1].enable(self.timeStep)

        # accelerometer
        self.accelerometer = self.getDevice("accelerometer")
        self.accelerometer.enable(4 * self.timeStep)

        # inertial unit
        self.inertialUnit = self.getDevice("inertial unit")
        self.inertialUnit.enable(self.timeStep)

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

        #if self.bAvoidCollision:
        collision = self.navigation_manager.avoidcollision()
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
        goalkeeperPosition = self.Supervisor.data["BlueGoalkeeper"]

        # Get the player's position
        playerPosition = self.Supervisor.data["RedForwardB"]

        # Get the robot's orientation angle
        robotAngle = np.degrees(self.getRollPitchYaw()[2])

        # Create grid and obstacles
        self.navigation_manager.create_grid()

        # If the initial ball position is not set, set it to the current position
        if self.initial_ball_position is None:
            self.initial_ball_position = currentBallPosition

        # Calculate the ball's velocity
        ball_distance = Functions.calculateDistance(currentBallPosition, self.initial_ball_position)

        # Update the previous ball position
        self.initial_ball_position = currentBallPosition

        if ball_distance > 0.0003:
            self.game_started = True

        #print(f"Red Forward A Current State: {self.AppState}")
        if self.red_team_start == True:
            print(f"Red Team Start Status: {self.red_team_start}")
            print("Red Team Starts")

        print(f"Game Started: {self.game_started}")
        #print(f"currentSelfPosition: {currentSelfPosition}")

        # Add Strategies Here
        match self.AppState:
            case RobotState.START_POSITION:
                return self.robot_state_manager.start_position_state(currentSelfPosition, self.StartLocation, StartToRobotdist, robotAngle)
            case RobotState.TRACK_BALL:
                return self.robot_state_manager.track_ball_state(currentSelfPosition, currentBallPosition, BallToRobotdist, robotAngle)
            case RobotState.BE_A_FORWARD:
                return self.robot_state_manager.be_a_forward_state(currentSelfPosition, currentBallPosition ,self.TargetPosition, BallToRobotdist, BallToGoaldist, robotAngle)
            case RobotState.SCORE_GOAL:
                return self.robot_state_manager.score_goal_state(currentSelfPosition, BallToRobotdist, goalkeeperPosition, self.TargetPosition, robotAngle)
            case RobotState.PASS_TO_PLAYER:
                return self.robot_state_manager.pass_to_player_state(currentSelfPosition, playerPosition)
            case _:
                self.AppState = RobotState.START_POSITION

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
            if robot_name == player_name or robot_name.startswith("Red"):
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
    
class PathPlanningManager:
    def __init__(self,soccer_robot):
        self.motions = Motions()
        self.soccer_robot = soccer_robot
        self.x_range = 4.5 - (-4.5)
        self.y_range = 2.8 - (-2.8)
        self.resolution = 0.05      # Define the resolution of the grid

    def detect_collision(self):
        obstacles = []
        for robot_name in self.soccer_robot.Supervisor.robot_list:
            if robot_name != self.soccer_robot.robotName:
                robot_position = self.soccer_robot.Supervisor.data[robot_name]

                # Calculate the distance between robots
                self.robot_distance = Functions.calculateDistance(
                    robot_position, self.soccer_robot.Supervisor.getSelfPosition()
                )

                # Define a safe distance threshold
                safe_distance = 0.5

                # Check if the robot is too close to an obstacle
                if self.robot_distance < safe_distance:
                    # Convert the robot position to grid position
                    grid_position = (
                        int((robot_position[0] - (-4.5)) / 0.1),
                        int((robot_position[1] - (-2.8)) / 0.1),
                    )
                    obstacles.append(grid_position)

        return obstacles
    
    def avoidcollision(self):
        if self.soccer_robot.ultrasound[1].getValue() < 0.5 and self.soccer_robot.ultrasound[0].getValue() < 0.5:
            self.soccer_robot.interruptMotion()
            return self.motions.backwards

        # If only the right sensor detects an obstacle, sidestep left
        elif self.soccer_robot.ultrasound[1].getValue() < 0.5:
            self.soccer_robot.interruptMotion()
            return self.motions.sideStepLeft

        # If only the left sensor detects an obstacle, sidestep right
        elif self.soccer_robot.ultrasound[0].getValue() < 0.5:
            self.soccer_robot.interruptMotion()
            return self.motions.sideStepRight
        else:
            return None
    
    def create_grid(self):
        # Detect collision
        obstacles = self.detect_collision()

        # Calculate the grid size
        grid_size_x = int(self.x_range / self.resolution)
        grid_size_y = int(self.y_range / self.resolution)

        # Create the grid
        self.grid = np.zeros((grid_size_y, grid_size_x))

        # Add the obstacles to the grid
        for obstacle in obstacles:
            self.grid[obstacle[1]][obstacle[0]] = 1

    def get_grid_position(self, position):
        grid_position = (
            int((position[0] - (-4.5)) / 0.1),
            int((position[1] - (-2.8)) / 0.1),
        )
        return grid_position
    
    def get_node_position(self, node):
        node_position = (
            node[0] * 0.1 + (-4.5),
            node[1] * 0.1 + (-2.8),
        )
        return node_position
    
    def get_next_move(self, current_position, target_position):

        # Print the grid dimensions
        #print(f"Grid dimensions: {self.grid.shape[0]} rows, {self.grid.shape[1]} columns")
        
        gridselfpose = self.get_grid_position(current_position)
        gridtargetpose = self.get_grid_position(target_position)

        #print(f"Grid Self Pose: {gridselfpose}")
        #print(f"Grid Target Pose: {gridtargetpose}")

        # Use the A* algorithm to find the shortest path to the ball
        path = aStar(self.grid, gridselfpose, gridtargetpose)

        if path and len(path) > 0:
            # Pop the next node from the path
            node = path.pop(0)
            node_position = self.get_node_position(node)
            return node_position
        else:
            print("Error: No path found or invalid path")
            return None
        
# RobotStateManager class
class RobotStateManager:
    def __init__(self, soccer_robot, navigation_manager):
        self.soccer_robot = soccer_robot
        self.navigation_manager = navigation_manager
        self.motions = Motions()

    def start_position_state(self, current_position, start_position, start_to_robot_dist, robot_angle):
        # Check if nao robot is away from the ball
        if start_to_robot_dist > 0.2:
            node_position = self.navigation_manager.get_next_move(current_position, start_position)
            if node_position:
                # Calculate the angle to the next node
                target_angle = Functions.calculateTargetAngle(node_position, current_position)
                turn_angle = Functions.calculateTurnAngle(target_angle, robot_angle)
                
                # Turn the robot towards the next node
                if abs(turn_angle) > 10:
                    return self.soccer_robot.getTurningMotion(turn_angle)
                else:
                    return self.motions.forwardLoop
                
        else:
            self.soccer_robot.AppState = RobotState.BE_A_FORWARD
            return self.motions.standInit

    def track_ball_state(self, current_position, ball_position, ball_to_robot_dist, robot_angle):
        # Check if nao robot is away from the ball
        if ball_to_robot_dist > 0.2:
            node_position = self.navigation_manager.get_next_move(current_position, ball_position)
            if node_position:
                # Calculate the angle to the next node
                target_angle = Functions.calculateTargetAngle(node_position, current_position)
                turn_angle = Functions.calculateTurnAngle(target_angle, robot_angle)
                
                # Turn the robot towards the next node
                if abs(turn_angle) > 10:
                    return self.soccer_robot.getTurningMotion(turn_angle)
                else:
                    return self.motions.forwardLoop
                
        else:
            self.soccer_robot.AppState = RobotState.PASS_TO_PLAYER
            return self.motions.standInit
        
    def be_a_forward_state(self, current_position, ball_position, target_position, ball_to_robot_dist, ball_to_goal_dist, robot_angle):
        # Game has not started yet, turn to face the ball and wait
        targetAngle_ball = Functions.calculateTargetAngle(ball_position, current_position)
        turnAngle_ball = Functions.calculateTurnAngle(targetAngle_ball, robot_angle)
        
        if abs(turnAngle_ball) > 18:
            return self.soccer_robot.getTurningMotion(turnAngle_ball)
    
        if not self.soccer_robot.red_team_start and not self.soccer_robot.game_started:
            return self.motions.standInit

        # Check if nao robot is away from the ball
        if ball_to_robot_dist > 0.2:
            if self.soccer_robot.red_team_start or (not self.soccer_robot.red_team_start and self.soccer_robot.game_started):
                self.soccer_robot.AppState = RobotState.TRACK_BALL
                return self.motions.standInit
            
            elif ball_to_goal_dist <= 0.2 and abs(current_position[1]) > abs(target_position[1]):
                self.soccer_robot.AppState = RobotState.SCORE_GOAL
                return self.motions.standInit
        
        else:
            node_position = self.navigation_manager.get_next_move(current_position, target_position)
            if node_position:
                # Calculate the angle to the next node
                target_angle = target_angle = Functions.calculateTargetAngle(node_position, current_position)
                turn_angle = Functions.calculateTurnAngle(target_angle, robot_angle)
                
                # Kick ball according to the turn angle
                if abs(turn_angle) > 18:
                    if turn_angle > 20:
                        return self.motions.rightSidePass
                    elif turn_angle < -30:
                        return self.motions.leftSidePass
                    else:
                        return self.motions.shoot
                else:              
                    return self.motions.forwards 

                
    def score_goal_state(self, current_position, ball_to_robot_dist, goalkeeper_position, target_position, robot_angle):
        # Check if nao robot is away from the ball
        if ball_to_robot_dist > 0.2:
            self.soccer_robot.AppState = RobotState.TRACK_BALL
            return self.motions.standInit
        
        else:
            # Calculate the angle to the goalkeeper position
            target_angle = Functions.calculateTargetAngle(goalkeeper_position, current_position)
            turn_angle = Functions.calculateTurnAngle(target_angle, robot_angle)

            # If the robot is facing the goal, kick
            if abs(turn_angle) < 18:
                # Determine which part of the goal the goalkeeper is not covering
                open_goal_section = self.openGoalSection(goalkeeper_position, target_position, goal_width=self.goal_width)
            
                # Aim for the open section of the goal
                if open_goal_section == 'left':
                    return self.motions.rightSidePass
                elif open_goal_section == 'right':
                    return self.motions.leftSidePass
                else:
                    return self.motions.shoot

            # Otherwise, turn towards the goal
            elif turn_angle > 0:
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


def main():
    # Create the robot and initialize the camera
    config = configparser.ConfigParser()
    config.read("../Utils/globalconfig.ini")
    robot = SoccerRobot(config)
    robot.run()


if __name__ == "__main__":
    main()
