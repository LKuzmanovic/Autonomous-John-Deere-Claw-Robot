from ultrasonic import Ultrasonic   
from picamera2 import Picamera2
import numpy as np
from PIL import Image
from led import Led
from motor import tankMotor  
import time  
from servo import Servo #to lock the claw
import time 
import cv2
import sys
sys.path.append("/home/pi/myvenv/lib/python3.11/site-packages")
from ultralytics import YOLO
from textWriting import create_readable_image
servo = Servo()

def downGrab(servo):  # Pass servo instance
    try:
        for i in range(150, 89, -1):  # Open claw outward
            servo.setServoAngle('0', i)
            time.sleep(0.01)

        for i in range(150, 89, -1):  # Put claw down
            servo.setServoAngle('1', i)
            time.sleep(0.01)

        for i in range(90, 150, 1):  # Close claw
            servo.setServoAngle('0', i)
            time.sleep(0.01)

        for i in range(90, 150, 1):  # Put claw up
            servo.setServoAngle('1', i)
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nEnd of program")

# Initialize hardware
camera = Picamera2()
PWM = tankMotor()
ultrasonic_sensor = Ultrasonic()
led = Led()

# Set camera configuration
camera_config = camera.create_preview_configuration()
camera.configure(camera_config)
camera.start()

def find_red_x_coordinate(image):
    # Convert image to NumPy array
    img_array = np.array(image)
    
    # Extract RGB channels
    red_channel = img_array[:, :, 0]
    green_channel = img_array[:, :, 1]
    blue_channel = img_array[:, :, 2]

    # Create a mask for red color (adjust threshold as needed)
    red_mask = (red_channel > 150) & (green_channel < 100) & (blue_channel < 100)

    # Get x-coordinates of detected red pixels
    x_coords = np.where(red_mask)[1]

    if len(x_coords) == 0:
        return None  # No red object detected

    # Find the mean x-coordinate of red pixels
    return int(np.mean(x_coords))

def move_robot():
    servo = Servo()
    SPEED = 660

    for i in range(90, 150, 1):  # Put claw up
            servo.setServoAngle('1', i)
            time.sleep(0.01)
    #Give the motors a little push
    PWM.setMotorModel(800,800)
    time.sleep(0.4)

    while True:
        # Capture image
        frame = camera.capture_array()
        image = Image.fromarray(frame)
        
        # Find red object position
        x_coord = find_red_x_coordinate(image)

        if x_coord is not None:
            img_width = image.width
            center_x = img_width // 2

            if x_coord < center_x - 5:  # Object is left of center (Responsible for reacting to the X coordinates and how far it is to the center)

                PWM.setMotorModel(-(SPEED+10), SPEED)

                # moving right


                led.colorWipe((0, 0, 0), 10)
            elif x_coord > center_x + 5:  # Object is right of center
                # PWM.setMotorModel(SPEED, -500)
                PWM.setMotorModel(SPEED+10, -SPEED)

                # moving left
                led.ledIndex(0x01, 0, 255, 0)      # Set LED 1 to green
                led.ledIndex(0x02, 0, 255, 0)      # Set LED 2 to green
                # led.ledIndex(0x04, 0, 255, 0)      # Set LED 3 to green
                # led.ledIndex(0x08, 0, 255, 0)          # set led 0 green

                led.colorWipe((0, 0, 0), 10)
            else:  # Object is centered
                # led.theaterChaseRainbow()  ? 

                led.colorWipe((0, 0, 0), 10)
                PWM.setMotorModel(SPEED, SPEED)
        
        # Check distance
        distance = ultrasonic_sensor.get_distance()
        if distance <= 9.55:
            print(ultrasonic_sensor.get_distance())
            PWM.setMotorModel(0, 0)
            time.sleep(2)
            break

        time.sleep(0.0025)  # Small delay for smooth processing

def classifyObject():
    model = YOLO("yolov8m.pt")

    image = camera.capture_array()

    flipped_image = np.flipud(np.fliplr(image))
    image_rgb = Image.fromarray(flipped_image).convert("RGB")
    image_rgb.save("object.jpg")
    camera.stop()

    image_path = "/home/pi/object.jpg"  # Replace with your image path
    image = cv2.imread(image_path)

    # Perform object detection
    results = model(image)

    # Draw bounding boxes on the image
    for result in results:
        for box in result.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])  # Get bounding box coordinates
            conf = box.conf[0].item()  # Confidence score
            cls = int(box.cls[0].item())  # Class ID
            label = f"{model.names[cls]} {conf:.2f}"  # Get label with confidence

            # Draw rectangle and label
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 255, 0), 2)

    # Save and display the image
    cv2.imwrite("/Server/classification.jpg", image)

# Run the movement function
move_robot()

# run the model
classifyObject()
create_readable_image()

downGrab(servo)