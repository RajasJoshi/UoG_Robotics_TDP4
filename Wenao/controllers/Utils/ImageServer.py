import queue
import torch
from collections import defaultdict
from threading import Thread
from ultralytics import YOLO

import cv2
import os
import sys
import numpy as np

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

class ImageServer:
    def __init__(self, width, height, camera, robot_name, position):
        self.camera = camera  # camera
        self.width = width
        self.height = height
        self.robot_name = robot_name
        self.position = position
        self.running = True
        self.track_history = defaultdict(lambda: [])
        self.track = []

        self.device = 0 if torch.cuda.is_available() else 'cpu'

        self.queue = queue.Queue(maxsize=3)
        self.thread = Thread(target=self.run, daemon=True)
        self.thread.start()

    def send(self, image):
        self.queue.put(image)

    def stop(self):
        self.running = False
        self.thread.join()

    def run(self):
        while self.running:
            try:
                img = self.queue.get(timeout=0.1)
                torch.cuda.empty_cache()
                cvimg = np.frombuffer(img, dtype=np.uint8).reshape(
                    (self.height, self.width, 4)
                )
                # Perform some OpenCV operations (e.g., grayscale conversion)
                frame = cv2.cvtColor(cvimg, cv2.COLOR_BGR2RGB)
            
                model = YOLO(r"../Utils/best.engine")

                results = model.predict(frame, device = self.device, conf = 0.5, verbose=False)
                '''results1 = model.track(frame, persist = True, device = 'cuda', tracker = 'bytetrack.yaml', classes = [0], conf = 0.5)

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
                        self.track = self.track_history[track_id]
                        self.track.append((float(x), float(y)))  # x, y center point
                        if len(self.track) > 30:  # retain 90 tracks for 90 frames
                            self.track.pop(0)

                        # Draw the tracking lines
                        if self.track:
                            points = np.hstack(self.track).astype(np.int32).reshape((-1, 1, 2))
                            cv2.polylines(annotated_frame, [points], isClosed=False, color=(230, 230, 230), thickness=10)
                        else: continue
                else: continue'''
                # Display the image using OpenCV
                # cv2.imshow(f"Image Stream - {self.robot_name} - {self.position}", cvimg)
                # cv2.waitKey(1)
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

