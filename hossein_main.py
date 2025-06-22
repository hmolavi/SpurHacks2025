#!/usr/bin/env python3

"""
pip install ultralytics opencv-python pyserial
"""

import cv2
from ultralytics import YOLO
import serial, time, sys

SERIAL_PORT = "COM3"
CONFIDENCE_THRESHOLD = 0.5
BAUD_RATE = 115200
SEND_HZ = 0.2

MODEL_PATH = "runs/detect/tennis_v6_nano/weights/best.pt"

PIXEL_TO_DEGREE_X = 0.017
PIXEL_TO_DEGREE_Y = 0.005

DEADBAND_X_PERCENT = 0.05
DEADBAND_Y_PERCENT = 0.05


ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0)
model = YOLO(MODEL_PATH)
cap = cv2.VideoCapture(1)  # Hossein - camera 1, Ali - Camera 0

last_msg = None
last_tx = 0
tx_period = 1 / SEND_HZ

print("Remember: press q in the pop-up window to quit.")
while True:
    ret, frame = cap.read()
    if not ret:
        break

    h, w, _ = frame.shape
    cx0, cy0 = w // 2, h // 2

    msg = "0,0"

    results = model(frame, verbose=False)[0]

    if results.boxes:
        confidences = results.boxes.conf
        if confidences[0] >= CONFIDENCE_THRESHOLD:
            x1, y1, x2, y2 = results.boxes.xyxy[0][:4]
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            cv2.circle(frame, (cx0, cy0), 5, (255, 0, 0), -1)

            dx_pixels = cx - cx0
            dy_pixels = cy - cy0

            if abs(dx_pixels) < w * DEADBAND_X_PERCENT:
                dx_pixels = 0
            if abs(dy_pixels) < h * DEADBAND_Y_PERCENT:
                dy_pixels = 0

            horiz_adjust_degrees = -dx_pixels * PIXEL_TO_DEGREE_X
            vert_adjust_degrees = dy_pixels * PIXEL_TO_DEGREE_Y

            msg = f"{horiz_adjust_degrees:.2f},{vert_adjust_degrees:.2f}"
            
            label = f"H:{horiz_adjust_degrees:.2f} V:{vert_adjust_degrees:.2f}"
            cv2.putText(frame, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)

    now = time.time()
    if msg != last_msg or (msg != "0,0" and now - last_tx >= tx_period):
        ser.write((msg + "\n").encode("ascii"))
        last_msg, last_tx = msg, now
        print("→", msg)

    cv2.imshow("YOLO Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
ser.close()
