#!/usr/bin/env python3

#================================================================
# File name: gem_gnss_pp_tracker_pid.py                                                                  
# Description: gnss waypoints tracker using pid and pure pursuit                                                                
# Author: Hang Cui
# Email: hangcui3@illinois.edu                                                                     
# Date created: 08/02/2021                                                                
# Date last modified: 08/15/2022                                                         
# Version: 1.0                                                                   
# Usage: rosrun gem_gnss gem_gnss_pp_tracker.py                                                                      
# Python version: 3.8                                                             
#================================================================


import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class LaneFollowingController:
    def __init__(self):
        # ROS publishers and subscribers
        self.sub_waypoints = rospy.Subscriber('lane_detection/Waypoints', Float32MultiArray, self.waypoints_callback, queue_size=1)
        self.ctrl_pub  = rospy.Publisher("/vesc/low_level/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)
        self.drive_msg = AckermannDriveStamped()
        self.drive_msg.header.frame_id = "f1tenth_control"
        # PID controller gains
        self.kp = 1.0
        self.ki = 0.0
        self.kd = 0.1
        # self.kp = 0.05  # tune
        # self.ki = 0.01  # tune 
        # self.kd = 0.1  # tune

        self.rate = rospy.Rate(50)
        self.drive_msg = AckermannDriveStamped()
        self.drive_msg.header.frame_id = "f1tenth_control"

        self.look_ahead = 0.3 # 4
        self.wheelbase  = 0.325 # meters
        self.offset     = 0.15 # meters 

        # Controller state
        self.integral_error = 0.0
        self.previous_error = 0.0
        self.target_speed = 1.5  # Desired speed in m/s

    def waypoints_callback(self, msg):
        try:
            # Reshape the waypoints into (N, 2) format
            waypoints = np.array(msg.data).reshape(-1, 2)
            print(waypoints)

            # Compute steering angle and throttle
            steering_angle, speed = self.compute_control(waypoints)
            print(steering_angle)
            print(speed)
        

            # Publish the control command
            self.publish_control(steering_angle, speed)
        except Exception as e:
            rospy.logger(f"Error processing waypoints: {e}")

    def compute_control(self, waypoints):
        """
        Compute the control command based on waypoints.

        Args:
            waypoints (np.ndarray): Array of waypoints [[x1, y1], [x2, y2], ...].

        Returns:
            float: Steering angle in radians.
            float: Speed in m/s.
        """
        if len(waypoints) < 1:
        # Not enough waypoints to calculate control
            return 0.0, 0.0

        # Desired lateral position of the track center
        track_center = 0.868571

        # Compute lateral error between the first waypoint and the track center
        current_position = waypoints[0][1]  # y-coordinate of the first waypoint
        error = current_position - track_center

        # PID control for steering
        self.integral_error += error
        derivative_error = error - self.previous_error
        self.previous_error = error

        steering_angle = (self.kp * error +
                        self.ki * self.integral_error +
                        self.kd * derivative_error)

        # Limit steering angle to [-0.4189, 0.4189] radians (roughly ±24 degrees)
        steering_angle = max(min(steering_angle, 0.4189), -0.4189)

        # Speed control (constant for now)
        speed = self.target_speed

        return steering_angle, speed

    def publish_control(self, steering_angle, speed):
        """
        Publish control commands to the vehicle.

        Args:
            steering_angle (float): Steering angle in radians.
            speed (float): Speed in m/s.
        """
        # control_msg = AckermannDriveStamped()
        # control_msg.header.stamp = rospy.Time.now()
        # control_msg.drive.steering_angle = steering_angle
        # control_msg.drive.speed = speed
        # self.pub_control.publish(control_msg)
        # print(control_msg)


        self.drive_msg.header.stamp = rospy.get_rostime()
        self.drive_msg.drive.steering_angle = steering_angle
        self.drive_msg.drive.speed = 1.5
        self.ctrl_pub.publish(self.drive_msg)
        print(self.drive_msg)
        
if __name__ == '__main__':
    rospy.init_node('lane_following_controller', anonymous=True)
    controller = LaneFollowingController()
    rospy.spin()