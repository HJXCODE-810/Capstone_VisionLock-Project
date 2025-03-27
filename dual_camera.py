import cv2
import serial
import threading
import time
from ultralytics import YOLO

# Initialize YOLOv8 model
model = YOLO("runs/detect/train2/weights/best.onnx")  # Use your trained model

# Initialize cameras
static_cam = cv2.VideoCapture(0)  # Wide-angle camera (horizontal tracking)
dynamic_cam = cv2.VideoCapture(1)  # Narrow-angle camera (vertical tracking)

# Arduino serial communication
arduino = serial.Serial('/dev/ttyUSB0', 9600)  # Adjust port as needed
time.sleep(2)  # Allow Arduino to initialize

# Frame center values (adjust based on your camera resolution)
frame_center_x = 640  # Horizontal frame center (static camera)
frame_center_y = 360  # Vertical frame center (dynamic camera)

# Stepper motor scaling factors (adjust based on your setup)
step_scale_x = 2.0  # Steps per pixel for horizontal movement
step_scale_y = 2.0  # Steps per pixel for vertical movement

def detect_person(frame):
    """Detect a person in the frame using YOLOv8."""
    results = model(frame, conf=0.5)
    
    for result in results:
        boxes = result.boxes
        for box in boxes:
            if int(box.cls[0]) == 0:  # Person class
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                return (x1 + x2) // 2, (y1 + y2) // 2
    
    return None, None

def send_to_arduino(x_steps, y_steps):
    """Send step commands to Arduino."""
    command = f"{x_steps},{y_steps}\n"
    arduino.write(command.encode())

def static_camera_thread():
    """Thread to handle horizontal tracking using the static camera."""
    while True:
        ret_static, frame_static = static_cam.read()
        if ret_static:
            cX, _ = detect_person(frame_static)
            if cX is not None:
                x_offset = cX - frame_center_x
                x_steps = int(x_offset * step_scale_x)

                # Get vertical position from dynamic camera
                ret_dyn, frame_dyn = dynamic_cam.read()
                if ret_dyn:
                    _, cY = detect_person(frame_dyn)
                    if cY is not None:
                        y_offset = cY - frame_center_y
                        y_steps = int(y_offset * step_scale_y)

                        # Send calculated steps to Arduino
                        send_to_arduino(x_steps, y_steps)

                        # Draw crosshair on static camera frame
                        cv2.line(frame_static, (cX - 20, frame_center_y), (cX + 20, frame_center_y), (0, 0, 255), 2)
                        cv2.line(frame_static, (cX, frame_center_y - 20), (cX, frame_center_y + 20), (0, 0, 255), 2)

                        # Draw crosshair on dynamic camera frame
                        cv2.line(frame_dyn, (frame_center_x, cY - 20), (frame_center_x, cY + 20), (0, 255, 0), 2)
                        cv2.line(frame_dyn, (frame_center_x - 20, cY), (frame_center_x + 20, cY), (0, 255, 0), 2)

            # Display video feeds
            cv2.imshow("Static Camera", frame_static)
            cv2.imshow("Dynamic Camera", frame_dyn)

        if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to exit
            break

# Start thread for detection and tracking
threading.Thread(target=static_camera_thread, daemon=True).start()

try:
    while True:
        time.sleep(0.1)  # Keep main thread alive
except KeyboardInterrupt:
    print("Exiting...")

# Release resources
static_cam.release()
dynamic_cam.release()
cv2.destroyAllWindows()
arduino.close()
