import cv2
import numpy as np

def measure_and_highlight_rectangles(image_path, output_path, min_size=(20, 20), adaptive=False):
    """
    Detect rectangles or distorted squares in the image, highlight them, and output the image with annotations.

    Args:
        image_path (str): Path to the input image.
        output_path (str): Path to save the output image.
        min_size (tuple): Minimum width and height in pixels to consider as valid shapes.
        adaptive (bool): Use adaptive thresholding for better detection under uneven lighting.

    Returns:
        list: Detected rectangle sizes in pixels [(w1, h1), (w2, h2), ...].
    """
    # Load the image
    image = cv2.imread(image_path)
    
    # Convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Apply thresholding
    if adaptive:
        binary = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                       cv2.THRESH_BINARY, 11, 2)
    else:
        _, binary = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

    # Morphological operations to close small gaps
    kernel = np.ones((3, 3), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

    # Find contours
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    rectangle_sizes = []
    
    # Loop through contours to find rectangles
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)  # Get bounding box of the contour
        aspect_ratio = float(w) / h  # Calculate the aspect ratio
        if 0.8 <= aspect_ratio <= 1.2 and w >= min_size[0] and h >= min_size[1]:  # Allow distorted squares
            rectangle_sizes.append((w, h))
            # Draw the bounding box around the detected rectangle
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            # Annotate the rectangle size on the image
            cv2.putText(image, f"{w}x{h}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            print(f"Rectangle found at ({x}, {y}) with size {w}x{h} pixels")
    
    # Save and display the image with highlighted rectangles
    cv2.imwrite(output_path, image)
    cv2.imshow("Detected Rectangles", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    return rectangle_sizes

# Example usage
image_path = "lanefittingrelative.png"  # Replace with your input image path
output_path = "highlightedSquares.png"  # Replace with your desired output path
rectangle_sizes = measure_and_highlight_rectangles(image_path, output_path, min_size=(20, 20), adaptive=True)
print("Rectangle Sizes (in pixels):", rectangle_sizes)
