from BlueDefenderLeft import DefenderLeft
from BlueDefenderRight import DefenderRight
from BlueForward import Forward
from BlueGoalkeeper import Goalkeeper
from controller import Robot
from DefaultBase import Nao

# Create a dictionary to map robot names to controller classes.
controller_mapping = {
    "NAOB1": Goalkeeper,
    "NAOB4": Forward,
    "NAOB2": DefenderLeft,
    "NAOB3": DefenderRight,
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
