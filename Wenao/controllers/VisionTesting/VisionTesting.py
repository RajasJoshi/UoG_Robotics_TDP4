"""my_controller controller."""


from controller import Robot
import cv2
import numpy as np
from ultralytics import YOLO

robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

cameraTop = robot.getDevice('CameraTop')
cameraBot = robot.getDevice('CameraBottom')
cameraBot.enable(timestep)
cameraTop.enable(timestep)

ball_diameter = 0.48 #meters
vertical_fov = 47.64 #degrees
horizontal_fov = 60.97 #degrees

distance_at_100_vfov = (ball_diameter / 2) / np.tan(np.deg2rad(vertical_fov/2))
distance_at_100_hfov = (ball_diameter / 2) / np.tan(np.deg2rad(horizontal_fov/2))

def ball_distance(ball_height, ball_width):
    
    verticalFovRatio = ball_height / cameraTop.getHeight()
    horizontalFovRatio = ball_width / cameraTop.getWidth()
    
    if (ball_height > 0.9 * ball_width) and (ball_height < ball_width * 1.1):
        vertical_estimate = distance_at_100_vfov * verticalFovRatio
        horizontal_estimate = distance_at_100_hfov * horizontalFovRatio
        distance_estimate = (vertical_estimate + horizontal_estimate) / 2
    
    elif ball_height < 0.9 * ball_width:
        horizontal_estimate = distance_at_100_hfov * horizontalFovRatio
        distance_estimate = horizontal_estimate
        
    else:
        vertical_estimate = distance_at_100_vfov * verticalFovRatio
        distance_estimate = vertical_estimate
    print(distance_estimate)
        

while robot.step(timestep) != -1:
    image = cameraTop.getImage()
    width, height = cameraTop.getWidth(), cameraTop.getHeight()
    
    # Convert the image to a format usable by OpenCV
    image = np.frombuffer(image, np.uint8).reshape((height, width, 4))

    # Perform some OpenCV operations (e.g., grayscale conversion)
    frame = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    
    model = YOLO('/home/jamie-linux/Documents/Robotics_MSc/UoG_Robotics_TDP4/Wenao/controllers/best.pt')

    results = model.predict(frame)[0]# Example: Display the grayscale image
    threshold = 0.5
    
    for result in results.boxes.data.tolist():
        x1, y1, x2, y2, score, class_id = result

        if score > threshold:
            if results.names[int(class_id)] == 'Ball':
                ball_height = x2 - x1
                ball_width = y2 - y1
                
                ball_distance(ball_height, ball_width)


            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 4)
            cv2.putText(frame, results.names[int(class_id)].upper(), (int(x1), int(y1 - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 255, 0), 3, cv2.LINE_AA)
    cv2.imshow('FRAME',frame)
    cv2.waitKey(1)

cv2.destroyAllWindows()
