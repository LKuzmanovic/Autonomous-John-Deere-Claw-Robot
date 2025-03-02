
def takeImage(object):
    import cv2
    import sys
    sys.path.append("/home/pi/myvenv/lib/python3.11/site-packages")
    from ultralytics import YOLO

    from picamera2 import Picamera2
    import numpy as np
    from PIL import Image

    # Initialize the camera
    picam2 = Picamera2()

    # Start the camera
    picam2.start()

    # Capture an image
    image = picam2.capture_array()

    # Flip the image both vertically and horizontally (180-degree rotation)
    flipped_image = np.flipud(np.fliplr(image))

    # Convert the flipped image to RGB
    image_rgb = Image.fromarray(flipped_image).convert("RGB")

    # Save the flipped image as jpg
    image_rgb.save("object.jpg")

    # Stop the camera
    picam2.stop()



    model = YOLO("yolov8m.pt")

    # Load the image

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
            cv2.putText(image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)


            

    # Save and display the image
    cv2.imwrite("output.jpg", image)




    #cv2.imshow("YOLO Object Detection", image) #idk what these lines do but they cause a crash
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
takeImage()


def moveClawDown():
    from servo import Servo
    import time
    servo = Servo()

    for i in range(150, 90, -1):
        servo.setServoAngle('0', i)
        time.sleep(0.01)
    for i in range(150, 90, -1):
        servo.setServoAngle('1', i)
        time.sleep(0.01)

#moveClawDown()
    