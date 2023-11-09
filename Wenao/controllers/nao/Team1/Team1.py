from controller import Robot
from DefaultBase import Nao
from RedDefenderLeft import DefenderLeft
from RedDefenderRight import DefenderRight
from RedForward import Forward
from RedGoalkeeper import Goalkeeper

# Create a dictionary to map robot names to controller classes.
controller_mapping = {
    "NAOR1": Goalkeeper,
    "NAOR4": Forward,
    "NAOR2": DefenderLeft,
    "NAOR3": DefenderRight,
}

# Create the Robot instance.
robot = Robot()

# Get the Robot Name to find the role.
robotName = robot.getName()

# Try to find the appropriate controller class based on the robot name.
robotController = controller_mapping.get(robotName, Nao)

# Create an instance of the selected controller class.
controller_instance = robotController(robot)

# Run the Robot Controller.
controller_instance.run()
