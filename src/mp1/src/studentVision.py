import time
import math
import numpy as np
import cv2
import rospy

from line_fit import centerline_fit_with_visualization
from Line import Line
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32
from skimage import morphology



class lanenet_detector():
    def __init__(self):

        self.bridge = CvBridge()
        # NOTE
        # Uncomment this line for lane detection of GEM car in Gazebo
        self.sub_image = rospy.Subscriber('/D435I/color/image_raw', Image, self.img_callback, queue_size=1)
        # Uncomment this line for lane detection of videos in rosbag
        #self.sub_image = rospy.Subscriber('camera/image_raw', Image, self.img_callback, queue_size=1)
        self.pub_image = rospy.Publisher("lane_detection/annotate", Image, queue_size=1)
        self.pub_bird = rospy.Publisher("lane_detection/birdseye", Image, queue_size=1)
        self.left_line = Line(n=5)
        self.right_line = Line(n=5)
        self.detected = False
        self.hist = True


    def img_callback(self, data):

        try:
            # Convert a ROS image message into an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        raw_img = cv_image.copy()
        mask_image, bird_image = self.detection(raw_img)

        if mask_image is not None and bird_image is not None:
            # Convert an OpenCV image into a ROS image message
            out_img_msg = self.bridge.cv2_to_imgmsg(mask_image, 'mono8')
            out_bird_msg = self.bridge.cv2_to_imgmsg(bird_image, 'bgr8')

            # Publish image message in ROS
            self.pub_image.publish(out_img_msg)
            self.pub_bird.publish(out_bird_msg)


    def gradient_thresh(self, img, thresh_min=25, thresh_max=100):
        """
        Apply sobel edge detection on input image in x, y direction
        """
        #1. Convert the image to gray scale
        #2. Gaussian blur the image
        #3. Use cv2.Sobel() to find derievatives for both X and Y Axis
        #4. Use cv2.addWeighted() to combine the results
        #5. Convert each pixel to unint8, then apply threshold to get binary image

        ## TODO
        
        gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Apply median and bilateral filters
        median = cv2.medianBlur(gray_image, 9)
        # blur = cv2.bilateralFilter(gray_image, 70, 29, 190)

        # # Compute Sobel gradients in x and y directions
        # gX = cv2.Sobel(blur, ddepth=cv2.CV_16S, dx=1, dy=0, ksize=5)
        # gY = cv2.Sobel(blur, ddepth=cv2.CV_16S, dx=0, dy=1, ksize=5)

        # # Combine gradients
        # combined = cv2.addWeighted(cv2.convertScaleAbs(gX), 0.5, cv2.convertScaleAbs(gY), 0.5, 0)

        # # Thresholding with NumPy for efficiency
        # binary_output = np.where((combined > thresh_min) & (combined <= thresh_max), 255, 0).astype(np.uint8)

        return median



    def color_thresh(self, img, thresh=(20, 30)):
        """
        Convert RGB to HSL and threshold to binary image using S channel
        """
        #1. Convert the image from RGB to HSL
        #2. Apply threshold on S channel to get binary image
        #Hint: threshold on H to remove green grass
        ## TODO

        ####
        # hls_image = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)

        # binary_output = np.zeros((img.shape[0], img.shape[1]))
        

        # for row in range(len(hls_image)):
        #     for col in range(len(hls_image[0])):
        #         lum = hls_image[row][col][2]
        #         sat = hls_image[row][col][1]
        #         hue = hls_image[row][col][0]
        #         if hue <= thresh[1] and hue > thresh[0] and sat > 150 and lum >50:
        #             binary_output[row][col] = 255
        #         else:
        #             binary_output[row][col] = 0
        # ####

        # return binary_output
    
        hls_image = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        
        # Split the HLS channels for easier access
        hue = hls_image[:, :, 0]
        sat = hls_image[:, :, 1]
        lum = hls_image[:, :, 2]
        
        # Create a binary mask based on the threshold conditions
        binary_output = np.zeros_like(hue)
        binary_output[
            (hue > thresh[0]) & (hue <= thresh[1]) & 
            (sat > 150) & (lum > 50)
        ] = 255
        
        return binary_output


    def combinedBinaryImage(self, img):
        """
        Get combined binary image from color filter and sobel filter
        """
        #1. Apply sobel filter and color filter on input image
        #2. Combine the outputs
        ## Here you can use as many methods as you want.

        ## TODO

        ####
        ColorOutput = self.color_thresh(img)
        
        SobelOutput = self.gradient_thresh(img)
        
        ####

        binaryImage = np.zeros_like(ColorOutput)
        
        binaryImage[(ColorOutput==255)|(SobelOutput==255)] = 255
        # Remove noise from binary image
        # binaryImage = morphology.remove_small_objects(binaryImage.astype('bool'),min_size=50,connectivity=2)

        return ColorOutput


    def perspective_transform(self, img, verbose=False):
        """
        Get bird's eye view from input image
        """
        #1. Visually determine 4 source points and 4 destination points
        #2. Get M, the transform matrix, and Minv, the inverse using cv2.getPerspectiveTransform()
        #3. Generate warped image in bird view using cv2.warpPerspective()

        ## TODO
        shape = img.shape
        original_pts = np.float32([[200, 290], [160, 480], [480, 480], [440, 290]])
        output_pts = np.float32([[0, 0],[280, 480],[360, 480],[500, 0]])
        M = cv2.getPerspectiveTransform(original_pts,output_pts)
        Minv = cv2.getPerspectiveTransform(output_pts,original_pts)
        warped_img = cv2.warpPerspective(np.float32(img),M,(640, 480),flags=cv2.INTER_LINEAR)

        ####
        return warped_img, M, Minv


    def detection(self, img):
            
        BinaryImage = self.combinedBinaryImage(img)
        img_birdeye, M, Minv = self.perspective_transform(BinaryImage)
        overlay = centerline_fit_with_visualization(img_birdeye)
        img1 = cv2.convertScaleAbs(BinaryImage, alpha=(255.0/np.max(img)))
        img2 = cv2.convertScaleAbs(overlay, alpha=(255.0/np.max(img)))

        return img1, img2


if __name__ == '__main__':
    # init args
    rospy.init_node('lanenet_node', anonymous=True)
    lanenet_detector()
    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(0.5)


