import queue
from collections import defaultdict
from threading import Thread
from ultralytics import YOLO

import cv2
import numpy as np


class ImageServer:
    def __init__(self, width, height, camera, robot_name, position):
        self.camera = camera  # camera
        self.width = width
        self.height = height
        self.robot_name = robot_name
        self.position = position
        self.running = True
        self.model = YOLO(r"C:\Users\thima\OneDrive\Documents\TDP\UoG_Robotics_TDP4\Wenao\controllers\Utils\best.pt",verbose=False)

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
                                print(f'ball width: {ball_width}')
                                print(f'ball height: {ball_height}')
                                print(f'x1: {x1}')
                                print(f'y1: {y1}')
                                print(f'x2: {x2}')
                                print(f'y2: {y2}')
                                print(f'score: {score}')
                                


                            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 4)
                            cv2.putText(frame, results.names[int(class_id)].upper(), (int(x1), int(y1 - 10)),
                                        cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 255, 0), 3, cv2.LINE_AA)
                
                    cv2.imshow('FRAME',frame)
                    cv2.waitKey(1)
                self.queue.task_done()
            except queue.Empty:
                continue


# Example usage:
# Uncomment and modify the following lines as needed for testing
# if __name__ == "__main__":
#     server = ImageServer(width=640, height=480, camera="front", robot_name="Robot1", position=(0, 0))
#     # Simulate sending images to the server
#     for i in range(5):
#         fake_image = np.random.randint(0, 255, (480, 640, 4), dtype=np.uint8).tobytes()
#         server.send(fake_image)
#     server.stop()
