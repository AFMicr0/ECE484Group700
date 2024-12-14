import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import pickle
# from combined_thresh import combined_thresh
# from perspective_transform import perspective_transform

# feel free to adjust the parameters in the code if necessary

def centerline_fit_with_visualization(binary_warped):
    """
    Detect and fit the centerline in the given binary warped image with visualization overlayed.
    """
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

    return out_img
