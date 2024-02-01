import queue
from collections import defaultdict
from threading import Thread

import cv2
import numpy as np
from ultralytics import YOLO


class ImageServer:
    def __init__(self, width, height, camera, robot_name, position):
        self.camera = camera  # camera
        self.width = width
        self.height = height
        self.robot_name = robot_name
        self.position = position
        self.running = True

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
                cvimg = np.frombuffer(img, dtype=np.uint8).reshape(
                    (self.height, self.width, 4)
                )
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
