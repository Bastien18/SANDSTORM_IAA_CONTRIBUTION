import random
from PIL import Image, ImageDraw
import os

# Function to generate a single drone camera image with lines progressing towards the middle
def generate_drone_image_pov(width, height):
    # Create a black image
    image = Image.new('L', (width, height), color=0)  # 'L' mode for grayscale
    draw = ImageDraw.Draw(image)
    
    # Generate a starting point at the bottom of the image
    start_point = (random.randint(0, width), height)
    points = [start_point]

    # Determine direction towards the middle
    direction = 1 if start_point[0] < width // 2 else -1

    # Generate intermediate points towards the middle with varying line width
    while points[-1][1] > height // 2:
        x = points[-1][0] + direction * random.randint(0, 10)  # Adjust randomness to control the path
        y = points[-1][1] - random.randint(5, 15)             # Adjust randomness to control the path
        x = max(0, min(width, x))  # Ensure x stays within image bounds
        y = max(height // 2, min(height, y))  # Ensure y moves towards the middle
        
        # Randomly decide if the line turns
        if random.random() < 0.05:  # Adjust probability of turning
            direction *= -1
        
        points.append((x, y))

        # Calculate line width based on y position
        line_width = 5 + (y - height // 2) // 10
        line_width = max(1, line_width)  # Ensure minimum line width
        
        # Draw line segment with calculated width
        draw.line([points[-2], points[-1]], fill=255, width=line_width)

    # Add noise to the other half of the image
    for y in range(height // 2):
        for x in range(width):
            # Check if the pixel is above the line
            if image.getpixel((x, y)) == 0:
                # Add random noise to the pixel
                noise = random.randint(0, 255)
                image.putpixel((x, y), noise)

    return image

# Generate multiple images with the new perspective
num_images = 100
width, height = 324, 244
new_images = [generate_drone_image_pov(width, height) for _ in range(num_images)]

# Save images to disk and collect filenames
output_dir = 'drone_images'
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

for i, img in enumerate(new_images):
    file_path = os.path.join(output_dir, f'new_drone_image_{i}.png')
    img.save(file_path)
    print(f"Image saved: {file_path}")

print(f"{num_images} images have been generated and saved in the '{output_dir}' directory.")
