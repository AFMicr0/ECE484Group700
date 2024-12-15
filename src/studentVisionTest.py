import time
import math
import numpy as np
import cv2
import rospy

from Line import Line
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32
from skimage import morphology

def gradient_thresh(img, thresh_min=70, thresh_max=200):
        """
        Apply sobel edge detection on input image in x, y direction
        """
        #1. Convert the image to gray scale
        #2. Gaussian blur the image
        #3. Use cv2.Sobel() to find derievatives for both X and Y Axis
        #4. Use cv2.addWeighted() to combine the results
        #5. Convert each pixel to unint8, then apply threshold to get binary image

        ## TODO
        # for row in range(len(img)):
        #     for col in range(len(img[0])):
        #         img[row][col] = 0.299*img[row][col][0] + 0.587*img[row][col][1] + 0.114*img[row][col][2]
        gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        median = cv2.medianBlur(gray_image,9)
        blur = cv2.bilateralFilter(median,70,29,190)

        gX = cv2.Sobel(blur, ddepth=-1, dx=1, dy=0, ksize=5)
        gY = cv2.Sobel(blur, ddepth=-1, dx=0, dy=1, ksize=5)

        combined = cv2.addWeighted(gX, 0.5, gY, 0.5, 0)

        absolute = cv2.convertScaleAbs(combined)

        binary_output = np.zeros(absolute.shape)
        for row in range(len(absolute)):
            for col in range(len(absolute[0])):
                pixel = absolute[row][col]
                if pixel <= thresh_max and pixel > thresh_min:
                    binary_output[row][col] = 255
                else:
                    binary_output[row][col] = 0

        # binary_output = absolute
        # print(binary_output)
        ####

        return binary_output

def color_thresh(img, thresh=(14, 40)):
        """
        Convert RGB to HSL and threshold to binary image using S channel
        """
        #1. Convert the image from RGB to HSL
        #2. Apply threshold on S channel to get binary image
        #Hint: threshold on H to remove green grass
        ## TODO
        hls_image = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        
        # Split the HLS channels for easier access
        hue = hls_image[:, :, 0]
        lum = hls_image[:, :, 1]
        sat = hls_image[:, :, 2]
        
        # Create a binary mask based on the threshold conditions
        binary_output = np.zeros_like(hue)
        binary_output[
            (hue > thresh[0]) & (hue <= thresh[1]) & 
            (sat > 90) & (lum > 20)
        ] = 255
        return binary_output

def combinedBinaryImage(img):
        """
        Get combined binary image from color filter and sobel filter
        """
        #1. Apply sobel filter and color filter on input image
        #2. Combine the outputs
        ## Here you can use as many methods as you want.

        ## TODO
        ColorOutput = color_thresh(img)
        print(ColorOutput.sum())
        SobelOutput = gradient_thresh(img)
        print(SobelOutput.sum())
        ####

        binaryImage = np.zeros_like(SobelOutput)
        binaryImage[(ColorOutput==255)] = 1
        # Remove noise from binary image
        binaryImage = morphology.remove_small_objects(binaryImage.astype('bool'),min_size=35,connectivity=2)
        finalBinaryImage = np.zeros_like(SobelOutput)
        finalBinaryImage[binaryImage ==1] = 255
        return finalBinaryImage

def perspective_transform(img, verbose=False):
		"""
		Get bird's eye view from input image
		"""
		#1. Visually determine 4 source points and 4 destination points
		#2. Get M, the transform matrix, and Minv, the inverse using cv2.getPerspectiveTransform()
		#3. Generate warped image in bird view using cv2.warpPerspective()

		## TODO
		shape = img.shape
		print(shape[0])
		print(shape[1])
		original_pts = np.float32([[75, 285], [40, 480], [600, 480], [565, 285]])
		output_pts = np.float32([[-15, -60],[280, 480],[400, 480],[595, -60]])
		M = cv2.getPerspectiveTransform(original_pts,output_pts)
		Minv = cv2.getPerspectiveTransform(output_pts,original_pts)
		warped_img = cv2.warpPerspective(np.float32(img),M,(640, 480),flags=cv2.INTER_LINEAR)

		####
		return warped_img, M, Minv

def generate_waypoints(center_fit, num_points=10, spacing=0.5):
    """
    Generate local waypoints based on the fitted polynomial.

    Args:
        center_fit (np.array): Polynomial coefficients of the centerline.
        num_points (int): Number of waypoints to generate.
        spacing (float): Spacing between consecutive waypoints in meters.

    Returns:
        list: Array of waypoints [[x1, y1], [x2, y2], ...].
    """
    # Real-world scaling factors
    pixels_per_meter_x = 21 / 0.057  # Horizontal scaling factor
    pixels_per_meter_y = 21 / 0.057  # Vertical scaling factor

    # Generate y values (in meters) spaced evenly
    ploty = np.linspace(0, num_points * spacing, num_points) * pixels_per_meter_y  # Convert to pixels

    if center_fit is not None:
        # Generate corresponding x values (in pixels) using the polynomial
        plotx = center_fit[0] * ploty**2 + center_fit[1] * ploty + center_fit[2]
    else:
        plotx = np.zeros_like(ploty)

    # Convert waypoints to meters
    waypoints = [(plotx[i] / pixels_per_meter_x, ploty[i] / pixels_per_meter_y) for i in range(len(ploty))]
    return waypoints


def centerline_fit_with_visualization(binary_warped):
    """
    Detect and fit the centerline in the given binary warped image with visualization overlayed.
    Also overlays generated waypoints.

    Args:
        binary_warped (np.ndarray): Binary warped image of the lane.

    Returns:
        np.ndarray: Visualization image with centerline and waypoints.
    """
    print("reached1")
    # Sum along columns to get the horizontal projection of the centerline
    histogram = np.sum(binary_warped, axis=0)
    
    # Find the x-position of the maximum value in the histogram (centerline position)
    center_x = np.argmax(histogram)
    
    # Create a visualization image
    out_img = np.dstack((binary_warped, binary_warped, binary_warped)) * 255
    
    # Get non-zero pixel coordinates
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    
    # Define the sliding window parameters
    nwindows = 16
    margin = 100
    minpix = 50
    window_height = binary_warped.shape[0] // nwindows
    
    # Current position of the centerline
    current_x = center_x
    
    # Lists to store valid pixel positions for line fitting
    valid_centerx = []
    valid_centery = []

    # Loop through each window
    for window in range(nwindows):
        # Define the boundaries of the current window
        win_y_low = binary_warped.shape[0] - (window + 1) * window_height
        win_y_high = binary_warped.shape[0] - window * window_height
        win_x_low = current_x - margin
        win_x_high = current_x + margin

        # Draw the window on the visualization image
        cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (0, 255, 0), 2)

        # Identify the non-zero pixels within the window
        good_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                     (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]
        
        # If enough pixels are found, use them for line fitting
        if len(good_inds) > minpix:
            valid_centerx.extend(nonzerox[good_inds])  # Collect x-coordinates
            valid_centery.extend(nonzeroy[good_inds])  # Collect y-coordinates
            
            # Recenter the next window based on the mean x-position of pixels
            current_x = int(np.mean(nonzerox[good_inds]))

    # Fit a second-order polynomial (if enough points are detected)
    if len(valid_centerx) > 0:
        center_fit = np.polyfit(valid_centery, valid_centerx, 2)
    else:
        center_fit = None

    # Generate x and y values for plotting
    ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0])
    if center_fit is not None:
        center_fitx = center_fit[0]*ploty**2 + center_fit[1]*ploty + center_fit[2]
    else:
        center_fitx = np.zeros_like(ploty)

    # Draw the fitted line onto the visualization image
    for i in range(len(ploty) - 1):
        cv2.line(out_img, 
                 (int(center_fitx[i]), int(ploty[i])), 
                 (int(center_fitx[i+1]), int(ploty[i+1])), 
                 (255, 0, 255), 5)

    # Generate waypoints and overlay them
    waypoints = generate_waypoints(center_fit, num_points=10, spacing=0.5)
    print("reached")
    print(waypoints)
    for waypoint in waypoints:
        x, y = int(waypoint[0] * 21 / 0.057), int(waypoint[1] * 21 / 0.057)  # Convert back to pixels for visualization
        cv2.circle(out_img, (x, y), radius=15, color=(0, 0, 255), thickness=-1)  # Red circles for waypoints

    return out_img


def main():
    img = cv2.imread('test.png')
    binary = combinedBinaryImage(img)
    # # print(np.float32(binary).sum())
    # # print(binary.shape)
    # # binary_visual = np.zeros(binary.shape)
    # # for row in range(len(binary)):
    # #	for col in range(len(binary[0])):
    # #		if (binary[row][col] == 1) :
    # #			# print("Value 1")
    # #			binary_visual[row][col] = 255
    # # print(binary_visual.sum())
    binary_warped, M, Minv = perspective_transform(binary)
    # # warped_visual = np.zeros(binary_warped.shape)
    # # for row in range(len(binary)):
    # # 	for col in range(len(binary[0])):
    # # 		if (binary_warped[row][col] == 1) :
    # # 			# print("Value 1")
    # # 			warped_visual[row][col] = 255
    # # print(warped_visual.sum())
    # # cv2.imwrite('./binary_warped.png', np.float32(warped_visual))

    # # warped, M2, M2inv = perspective_transform(img)
    # # cv2.imwrite('./warped.png', warped)
    # output = line_fit(binary_warped)
    final = centerline_fit_with_visualization(binary_warped)
    print("did final")

    cv2.imwrite('./lanefittingrelative.png', final)

if __name__ == "__main__":
	main()