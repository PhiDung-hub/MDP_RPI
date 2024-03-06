from ultralytics import YOLO
# import subprocess
import numpy as np
import cv2
import math
import os

# Initialize the YOLOv5 model
model = YOLO("best.pt")

# Object classes
classNames = ["arrow-bullseye", "bullseye-arrow", "bullseye-arrow-bullseye", "id11", "id12", "id13", "id14", "id15", "id16", "id17", "id18", "id19",
              "id20", "id21", "id22", "id23", "id24", "id25", "id26", "id27", "id28", "id29", "id30", "id31", "id32", "id33", "id34", "id35", "id36",
              "id37", "id38", "id39", "id40", "id99"]

# Function to capture image from libcamera using libcamera-still command
def capture_image():
    # Capture image using libcamera-still command
    # command = ["libcamera-still", "-o", "image.jpg"]
    # subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    os.system("libcamera-still -e jpg -n -t 500 -o image.jpg --awb auto")

# Function to perform object detection on captured image
def detect_objects_in_image():
    # Read the captured image
    img = cv2.imread("image.jpg")

    # Perform object detection
    results = model(img, stream=True)

    # Coordinates
    for r in results:
        boxes = r.boxes

        for box in boxes:
            # Bounding box
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)  # Convert to int values

            # Draw box on the image
            cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 3)

            # Confidence
            confidence = math.ceil((box.conf[0] * 100)) / 100
            print("Confidence --->", confidence)

            # Class name
            cls = int(box.cls[0])
            if cls == 38:
                cls = 39
            elif cls == 39:
                cls = 38
            print("Class name -->", classNames[cls])

            # Object details
            org = [x1, y1]
            font = cv2.FONT_HERSHEY_SIMPLEX
            fontScale = 7
            color = (0, 255, 0)
            thickness = 12

            cv2.putText(img, classNames[cls], org, font, fontScale, color, thickness)

    # Show the image
    cv2.imshow('Image', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    # Capture image
    capture_image()

    # Perform object detection on captured image
    detect_objects_in_image()
