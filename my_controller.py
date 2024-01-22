"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import cv2
import numpy as np
from ultralytics import YOLO
# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

cameraTop = robot.getDevice('CameraTop')
cameraBot = robot.getDevice('CameraBottom')
cameraBot.enable(timestep)
cameraTop.enable(timestep)

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    image = cameraTop.getImage()
    width, height = cameraTop.getWidth(), cameraTop.getHeight()
    
    # Convert the image to a format usable by OpenCV
    image = np.frombuffer(image, np.uint8).reshape((height, width, 4))

    # Perform some OpenCV operations (e.g., grayscale conversion)
    frame = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    
    model = YOLO('/Users/sovikghosh/Desktop/best.pt')

    results = model.predict(frame)[0]# Example: Display the grayscale image
    threshold = 0.5
    
    for result in results.boxes.data.tolist():
        x1, y1, x2, y2, score, class_id = result

        if score > threshold:
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 4)
            cv2.putText(frame, results.names[int(class_id)].upper(), (int(x1), int(y1 - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 255, 0), 3, cv2.LINE_AA)
    cv2.imshow('FRAME',frame)
    cv2.waitKey(1)

cv2.destroyAllWindows()
