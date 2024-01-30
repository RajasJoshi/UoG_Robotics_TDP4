
import pip
pip.main(['install', 'lapx>=0.5.2'])
pip.main(['install', 'torch', 'torchaudio', 'torchvision', '--index-url', 'https://download.pytorch.org/whl/cu118'])

from controller import Robot
from collections import defaultdict
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

track_history = defaultdict(lambda: [])
track = []



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
   
    model = YOLO(r"F:\best.pt")

    results = model.predict(frame, device = 'cuda', classes = [0,1,2,3], conf = 0.5)
    results1 = model.track(frame, persist = True, device = 'cuda', tracker = 'bytetrack.yaml', classes = [0], conf = 0.5)

    # Get the boxes and track IDs
    boxes = results1[0].boxes.xywh.cpu().tolist()
    names = results1[0].names
    try:
        track_ids = results1[0].boxes.id.cpu().tolist()
    except:
        track_ids = None
    clss = results1[0].boxes.cls.cpu().tolist()

    # Visualize the results on the frame
    annotated_frame = results[0].plot()

    # Plot the tracks
    if track_ids:
        for box, track_id in zip(boxes, track_ids):
            x, y, w, h = box
            track = track_history[track_id]
            track.append((float(x), float(y)))  # x, y center point
            if len(track) > 30:  # retain 90 tracks for 90 frames
                track.pop(0)

            # Draw the tracking lines
            if track:
                points = np.hstack(track).astype(np.int32).reshape((-1, 1, 2))
                cv2.polylines(annotated_frame, [points], isClosed=False, color=(230, 230, 230), thickness=10)
            else: continue
    else: continue

    # Display the annotated frame
    cv2.namedWindow('Window', cv2.WINDOW_NORMAL)

    # Resize the window to your desired size
    cv2.resizeWindow('Window', 400, 300)
    
    
    cv2.imshow("Window", annotated_frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
cv2.destroyAllWindows()