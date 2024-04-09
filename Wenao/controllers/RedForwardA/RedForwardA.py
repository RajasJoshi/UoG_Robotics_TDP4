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
from Utils.pathPlanning import aStar


class RobotState(Enum):
    INIT = 0
    GO_TO_BALL = 1
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
        self.AppState = RobotState.INIT

        self.bVisionUsed = config.getboolean("RedTeam", "Vision")
        self.bAvoidCollision = config.getboolean("RedTeam", "Avoidance")
        self.PlayerMode = config.get("RedForwardA", "PlayerMode")
        self.Strategy = config.get("RedForwardA", "Strategy")
        TargetPosition = config.get("RedForwardA", "TargetPos")
        self.TargetPosition = list(map(float, TargetPosition.split(",")))
        StartLocation = config.get("RedForwardA", "StartPos")
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

        self.emitter = self.getDevice("emitter")

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

        if self.bAvoidCollision:
            collision = self.avoidcollision()
            if self.isNewMotionValid(collision):
                self.addMotionToQueue(collision)
                self.startMotion()

        # Get the current position
        currentSelfPosition = self.Supervisor.getSelfPosition()
        currentBallPosition = self.Supervisor.getBallData()

        # Get the robot's orientation angle
        robotAngle = np.degrees(self.getRollPitchYaw()[2])

        BallToRobotdist = Functions.calculateDistance(
            currentBallPosition, currentSelfPosition
        )

        # Calculate the ball distance to the goal position
        BallToGoaldist = Functions.calculateDistance(
            currentBallPosition, self.TargetPosition
        )

        # Detect collision
        obstacles = self.detect_collision()

        x_range = 4.5 - (-4.5)
        y_range = 2.8 - (-2.8)

        # Define the resolution of the grid
        resolution = 0.05

        # Calculate the size of the grid
        grid_size_x = int(x_range / resolution)
        grid_size_y = int(y_range / resolution)

        # Create the grid
        grid = np.zeros((grid_size_y, grid_size_x))

        # Add the obstacles to the grid
        for obstacle in obstacles:
            grid[obstacle[1]][obstacle[0]] = 1

        # Convert the coordinates from meters to grid cells
        gridselfpose = (
            int((currentSelfPosition[0] - (-4.5)) / 0.1),
            int((currentSelfPosition[1] - (-2.8)) / 0.1),
        )

        match self.AppState:
            case RobotState.INIT:

                # Check if nao robot is away from the ball
                if BallToRobotdist > 0.2:
                    gridtargetpose = (
                        int((currentBallPosition[0] - (-4.5)) / 0.1),
                        int((currentBallPosition[1] - (-2.8)) / 0.1),
                    )

                    # Use the A* algorithm to find the shortest path to the ball
                    self.path = aStar(grid, gridselfpose, gridtargetpose)
                    # Calculate the Robot's angle to the ball position
                    if self.path and len(self.path) > 0:
                        # Pop the next node from the path
                        node = self.path.pop(0)
                        node_position = (
                            node[0] * 0.1 + (-4.5),
                            node[1] * 0.1 + (-2.8),
                        )
                        # Calculate the angle to the next node in the path
                        targetAngle = np.degrees(
                            np.arctan2(
                                node_position[1] - currentSelfPosition[1],
                                node_position[0] - currentSelfPosition[0],
                            )
                        )

                        # Calculate the turn angle in the range [-180, 180)
                        turnAngle = Functions.calculateTurnAngle(
                            targetAngle, robotAngle
                        )

                        # Turn the robot towards the next node in the path
                        if abs(turnAngle) > 10:
                            return self.getTurningMotion(turnAngle)

                        # Move the robot towards the next node in the path
                        return self.motions.forwardLoop

                # Check if the ball is near the goalpost
                else:
                    msgId = 2
                    GameState = 1
                    message = f"{msgId},{GameState}"
                    self.emitter.send(message.encode("utf-8"))

                    self.AppState = RobotState.BE_A_FORWARD
                    return self.motions.standInit

            case RobotState.GO_TO_BALL:

                # Check if nao robot is away from the ball
                if BallToRobotdist > 0.2:
                    gridtargetpose = (
                        int((currentBallPosition[0] - (-4.5)) / 0.1),
                        int((currentBallPosition[1] - (-2.8)) / 0.1),
                    )

                    # Use the A* algorithm to find the shortest path to the ball
                    self.path = aStar(grid, gridselfpose, gridtargetpose)
                    # Calculate the Robot's angle to the ball position
                    if self.path and len(self.path) > 0:
                        # Pop the next node from the path
                        node = self.path.pop(0)
                        node_position = (
                            node[0] * 0.1 + (-4.5),
                            node[1] * 0.1 + (-2.8),
                        )
                        # Calculate the angle to the next node in the path
                        targetAngle = np.degrees(
                            np.arctan2(
                                node_position[1] - currentSelfPosition[1],
                                node_position[0] - currentSelfPosition[0],
                            )
                        )

                        # Calculate the turn angle in the range [-180, 180)
                        turnAngle = Functions.calculateTurnAngle(
                            targetAngle, robotAngle
                        )

                        # Turn the robot towards the next node in the path
                        if abs(turnAngle) > 10:
                            return self.getTurningMotion(turnAngle)

                        # Move the robot towards the next node in the path
                        return self.motions.forwardLoop

                # Check if the ball is near the goalpost
                else:
                    self.AppState = RobotState.BE_A_FORWARD
                    return self.motions.standInit

            case RobotState.BE_A_FORWARD:

                # Check if nao robot is away from the ball
                if BallToRobotdist > 0.2:
                    self.AppState = RobotState.GO_TO_BALL
                    return self.motions.standInit

                # Check if the ball is near the goalpost
                elif BallToGoaldist <= 0.2 and abs(currentSelfPosition[1]) > abs(
                    self.TargetPosition[1]
                ):
                    self.AppState = RobotState.SCORE_GOAL
                    return self.motions.standInit

                else:
                    gridtargetpose = (
                        int((self.TargetPosition[0] - (-4.5)) / 0.1),
                        int((self.TargetPosition[1] - (-2.8)) / 0.1),
                    )

                    # Use the A* algorithm to find the shortest path to the ball
                    self.path = aStar(grid, gridselfpose, gridtargetpose)
                    # Calculate the Robot's angle to the ball position
                    if self.path and len(self.path) > 0:
                        # Pop the next node from the path
                        node = self.path.pop(0)
                        node_position = (
                            node[0] * 0.1 + (-4.5),
                            node[1] * 0.1 + (-2.8),
                        )
                        # Calculate the angle to the next node in the path
                        targetAngle = np.degrees(
                            np.arctan2(
                                node_position[1] - currentSelfPosition[1],
                                node_position[0] - currentSelfPosition[0],
                            )
                        )

                        # Calculate the turn angle in the range [-180, 180)
                        turnAngle = Functions.calculateTurnAngle(
                            targetAngle, robotAngle
                        )

                        if abs(turnAngle) > 18:
                            if turnAngle > 20:
                                return self.motions.rightSidePass
                            elif turnAngle < -30:
                                return self.motions.leftSidePass
                            else:
                                return self.motions.shoot
                        else:
                            return self.motions.forwards

            case RobotState.SCORE_GOAL:

                # Check if nao robot is away from the ball
                if BallToRobotdist > 0.2:
                    self.AppState = RobotState.GO_TO_BALL
                    return self.motions.standInit

                else:
                    # Calculate the Robot's angle to the goalkeeper position
                    targetAngle = Functions.calculateAngle(
                        self.Supervisor.data["BlueGoalkeeper"], currentSelfPosition
                    )
                    turnAngle = Functions.calculateTurnAngle(targetAngle, robotAngle)

                    # If the robot is facing the goal, kick
                    if abs(turnAngle) < 18:
                        return self.motions.shoot

                    # Otherwise, turn towards the goal
                    elif turnAngle > 0:
                        return self.motions.rightSidePass
                    else:
                        return self.motions.leftSidePass

            case RobotState.PASS_TO_PLAYER:
                currentSelfPosition = self.Supervisor.getSelfPosition()
                # Get the player's position
                playerPosition = self.Supervisor.data["RedForwardB"]

                # Determine whether to pass to the right or left
                if playerPosition[1] > currentSelfPosition[1]:
                    self.interruptMotion()
                    # The player is to the right of the robot, so pass to the right
                    if self.isNewMotionValid(self.motions.rightSidePass):
                        self.addMotionToQueue(self.motions.rightSidePass)
                        self.startMotion()
                    self.AppState = RobotState.BE_A_FORWARD

                else:
                    if self.isNewMotionValid(self.motions.leftSidePass):
                        self.addMotionToQueue(self.motions.leftSidePass)
                        self.startMotion()
                    self.AppState = RobotState.BE_A_FORWARD

            case _:
                self.AppState = RobotState.INIT

    def detect_collision(self):
        obstacles = []
        for robot_name in self.Supervisor.robot_list:
            if robot_name != self.robotName:
                robot_position = self.Supervisor.data[robot_name]

                # Calculate the distance between robots
                self.robot_distance = Functions.calculateDistance(
                    robot_position, self.Supervisor.getSelfPosition()
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
        if self.ultrasound[1].getValue() < 0.5 and self.ultrasound[0].getValue() < 0.5:
            self.interruptMotion()
            return self.motions.backwards

        # If only the right sensor detects an obstacle, sidestep left
        elif self.ultrasound[1].getValue() < 0.5:
            self.interruptMotion()
            return self.motions.sideStepLeft

        # If only the left sensor detects an obstacle, sidestep right
        elif self.ultrasound[0].getValue() < 0.5:
            self.interruptMotion()
            return self.motions.sideStepRight
        else:
            return None

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


def main():
    # Create the robot and initialize the camera
    config = configparser.ConfigParser()
    config.read("../Utils/globalconfig.ini")
    robot = SoccerRobot(config)
    robot.run()


if __name__ == "__main__":
    main()
