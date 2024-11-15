import time
import math
import numpy as np
import cv2
import rospy

from line_fit import line_fit, tune_fit, bird_fit, final_viz
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

def color_thresh(img, thresh=(20, 30)):
        """
        Convert RGB to HSL and threshold to binary image using S channel
        """
        #1. Convert the image from RGB to HSL
        #2. Apply threshold on S channel to get binary image
        #Hint: threshold on H to remove green grass
        ## TODO
        hls_image = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)

        binary_output = np.zeros((img.shape[0], img.shape[1]))

        for row in range(len(hls_image)):
            for col in range(len(hls_image[0])):
                lum = hls_image[row][col][2]
                sat = hls_image[row][col][1]
                hue = hls_image[row][col][0]
                if hue <= thresh[1] and hue > thresh[0] and sat > 150 and lum >50:
                    binary_output[row][col] = 255
                else:
                    binary_output[row][col] = 0
        ####

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
        binaryImage = morphology.remove_small_objects(binaryImage.astype('bool'),min_size=5,connectivity=2)
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
        print(img.shape)
        original_pts = np.float32([[0, 190], [0, 480], [640, 480], [640, 190]])
        output_pts = np.float32([[0, 50],
                                [0, 1000],
                                [1200, 1000],
                                [1200, 0]])
        tM = cv2.getPerspectiveTransform(original_pts,output_pts)
        warped_img = cv2.warpPerspective(np.float32(img),tM,(640, 480),flags=cv2.INTER_LINEAR)
		
        ####

        return img, img, np.linalg.inv(img)

def line_fit(binary_warped):
	"""
	Find and fit lane lines
	"""
	# Assuming you have created a warped binary image called "binary_warped"
	# Take a histogram of the bottom half of the image
	histogram = np.sum(binary_warped[binary_warped.shape[0]//2:,:], axis=0)
	# Create an output image to draw on and visualize the result
	out_img = (np.dstack((binary_warped, binary_warped, binary_warped))*255).astype('uint8')
	# Find the peak of the left and right halves of the histogram
	# These will be the starting point for the left and right lines
	midpoint = int(histogram.shape[0]/2)
	leftx_base = np.argmax(histogram[100:midpoint]) + 100
	rightx_base = np.argmax(histogram[midpoint:-100]) + midpoint

	# Choose the number of sliding windows
	nwindows = 9
	# Set height of windows
	window_height = int(binary_warped.shape[0]/nwindows)
	# Identify the x and y positions of all nonzero pixels in the image
	nonzero = binary_warped.nonzero()
	
	nonzeroy = np.array(nonzero[0])
	print(nonzeroy)
	nonzerox = np.array(nonzero[1])
	print(nonzerox)
	combined = list(zip(nonzerox, nonzeroy))
	# Current positions to be updated for each window
	leftx_current = leftx_base
	rightx_current = rightx_base
	# Set the width of the windows +/- margin
	margin = 100
	# Set minimum number of pixels found to recenter window
	minpix = 50
	# Create empty lists to receive left and right lane pixel indices
	left_lane_inds = []
	right_lane_inds = []

	# Step through the windows one by one
	for window in range(nwindows):
		# Identify window boundaries in x and y (and right and left)
		##TO DO
		x = window_height*window
		y = window_height*(window + 1)
		left_one = leftx_base
		right_zero = rightx_base
		left_zero = left_one - margin
		right_one = right_zero + margin
		####
		# Draw the windows on the visualization image using cv2.rectangle()
		##TO DO
		left_top_left = (left_zero, x)
		left_bottom_right = (left_one, y)
		color = (255, 0, 0)
		thickness = 2
		out_img = cv2.rectangle(out_img, left_top_left, left_bottom_right, color, thickness)
		right_top_left = (right_zero, x)
		right_bottom_right = (right_one, y)
		out_img = cv2.rectangle(out_img, right_top_left, right_bottom_right, color, thickness)
		####
		# Identify the nonzero pixels in x and y within the window
		##TO DO
		# middle = (right - left) / 2
		# nonzero_window_left = nonzero[x:y, left:middle]
		# nonzero_window_right = nonzero[x:y, middle:right]
		# left_lane = []
		# right_lane = []
	
		# for row in range (x, y) :
		# 	for col in range (left, right) :
		# 		if (row, col) in combined:
		# 			if (col < middle) :
		# 				left_lane.append((row, col))
		# 			else :
		# 				right_lane_inds.append((row, col))
		####
		# Append these indices to the lists
		##TO DO
		# left_lane_inds.append(left_lane)
		# right_lane_inds.append(right_lane)
		####
		# If you found > minpix pixels, recenter next window on their mean position
		##TO DO
		# foundpix = len(left_lane) + len(right_lane)
		# if (foundpix > minpix) :
		# 	midpoint = middle
		# 	leftx_base = np.argmax(histogram[100:midpoint]) + 100
		# 	rightx_base = np.argmax(histogram[midpoint:-100]) + midpoint
		####
		# pass

	# Concatenate the arrays of indices
	return out_img
	left_lane_inds = np.concatenate(left_lane_inds)
	right_lane_inds = np.concatenate(right_lane_inds)

	# Extract left and right line pixel positions
	leftx = nonzerox[left_lane_inds]
	lefty = nonzeroy[left_lane_inds]
	rightx = nonzerox[right_lane_inds]
	righty = nonzeroy[right_lane_inds]

	# Fit a second order polynomial to each using np.polyfit()
	# If there isn't a good fit, meaning any of leftx, lefty, rightx, and righty are empty,
	# the second order polynomial is unable to be sovled.
	# Thus, it is unable to detect edges.
	try:
	##TODO
		left_fit = np.polyfit(leftx, lefty, 2)
		right_fit = np.polyfit(rightx, righty, 2)
	####
	except TypeError:
		print("Unable to detect lanes")
		return out_img


	# Return a dict of relevant variables
	ret = {}
	ret['left_fit'] = left_fit
	ret['right_fit'] = right_fit
	ret['nonzerox'] = nonzerox
	ret['nonzeroy'] = nonzeroy
	ret['out_img'] = out_img
	ret['left_lane_inds'] = left_lane_inds
	ret['right_lane_inds'] = right_lane_inds

	return ret

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


	cv2.imwrite('./lanefitting.png', binary_warped)

if __name__ == "__main__":
	main()