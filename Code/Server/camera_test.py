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
