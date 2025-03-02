from PIL import Image, ImageDraw, ImageFont

# File paths
IMAGE_PATH = "/home/pi/object.jpg"  # Your source image
OUTPUT_IMAGE = "output_image.jpg"  # Image with text overlay
TEXT_MESSAGE = "Your item has been acquired"

def create_readable_image():
    """Creates a new image with large text overlay for normal viewing."""
    
    try:
        # Load the image
        image = Image.open(IMAGE_PATH)
        draw = ImageDraw.Draw(image)
        img_width, img_height = image.size

        # Dynamically adjust font size based on image dimensions
        font_size = int(img_width * 0.16)  # 8% of image width
        try:
            font = ImageFont.truetype("arial.ttf", font_size)  # Windows
        except IOError:
            font = ImageFont.load_default()  # Fallback font

        # Get text size using textbbox() (better than textsize())
        bbox = draw.textbbox((0, 0), TEXT_MESSAGE, font=font)
        text_width = bbox[2] - bbox[0]
        text_height = bbox[3] - bbox[1]

        # Define text position (centered)
        text_position = ((img_width - text_width) // 2, img_height - text_height - 50)

        # Draw text with an outline for better visibility
        outline_color = "black"
        text_color = "white"
        outline_range = 3  # Thickness of the outline

        for x_offset in range(-outline_range, outline_range + 1):
            for y_offset in range(-outline_range, outline_range + 1):
                if x_offset != 0 or y_offset != 0:
                    draw.text((text_position[0] + x_offset, text_position[1] + y_offset), 
                              TEXT_MESSAGE, font=font, fill=outline_color)

        # Draw the main text
        draw.text(text_position, TEXT_MESSAGE, font=font, fill=text_color)

        # Save the new image
        image.save(OUTPUT_IMAGE)
        print(f"Image saved as '{OUTPUT_IMAGE}' with large readable text!")

    except Exception as e:
        print(f"Error: {e}")

# Run the function
create_readable_image()