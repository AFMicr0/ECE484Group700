import cv2
import numpy as np
import pyautogui

def get_hls_from_screen():
    # Get the current mouse position
    x, y = pyautogui.position()
    
    # Capture the screenshot of the screen at the mouse position
    screenshot = pyautogui.screenshot(region=(x, y, 1, 1))  # Capture only one pixel

    # Convert the screenshot to a numpy array (from PIL image)
    screenshot_np = np.array(screenshot)

    # Convert RGB (PIL) to BGR (OpenCV)
    bgr = screenshot_np[0, 0, ::-1]  # Convert the pixel from RGB to BGR

    # Convert BGR to HLS
    hls = cv2.cvtColor(np.uint8([[bgr]]), cv2.COLOR_BGR2HLS)[0][0]
    
    return hls

def main():
    print("Click anywhere on the screen to get the HLS value")
    while True:
        if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to exit
            break
        
        hls = get_hls_from_screen()
        h, l, s = hls
        print(f"H: {h}, L: {l}, S: {s}")

if __name__ == "__main__":
    main()
