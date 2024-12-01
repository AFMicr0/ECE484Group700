import rospy
from gazebo_msgs.srv import GetModelState, GetModelStateResponse
from gazebo_msgs.msg import ModelState
from ackermann_msgs.msg import AckermannDrive
import numpy as np
from std_msgs.msg import Float32MultiArray
import math
from util import euler_to_quaternion, quaternion_to_euler
import time

class vehicleController():

    def __init__(self):
        # Publisher to publish the control input to the vehicle model
        self.controlPub = rospy.Publisher("/ackermann_cmd", AckermannDrive, queue_size = 1)
        self.prev_vel = 0
        self.L = 1.75 # Wheelbase, can be get from gem_control.py
        self.log_acceleration = False

    def getModelState(self):
        # Get the current state of the vehicle
        # Input: None
        # Output: ModelState, the state of the vehicle, contain the
        #   position, orientation, linear velocity, angular velocity
        #   of the vehicle
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp = serviceResponse(model_name='gem')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: "+str(exc))
            resp = GetModelStateResponse()
            resp.success = False
        return resp


    # Tasks 1: Read the documentation https://docs.ros.org/en/fuerte/api/gazebo/html/msg/ModelState.html
    #       and extract yaw, velocity, vehicle_position_x, vehicle_position_y
    # Hint: you may use the the helper function(quaternion_to_euler()) we provide to convert from quaternion to euler
    def extract_vehicle_info(self, currentPose):

        ####################### TODO: Your TASK 1 code starts Here #######################
        pos_x, pos_y, vel, yaw = 0, 0, 0, 0
        pos_x = currentPose.pose.position.x
        pos_y = currentPose.pose.position.y
        vel_x = currentPose.twist.linear.x
        vel_y = currentPose.twist.linear.y
        # vel_z = currentPose.twist.linear.z
        vel = math.sqrt(vel_x**2 + vel_y**2)
        ort = currentPose.pose.orientation
        qx = ort.x
        qy = ort.y
        qz = ort.z
        qw = ort.w
        roll, pitch, yaw = quaternion_to_euler(qx, qy, qz, qw)

        ####################### TODO: Your Task 1 code ends Here #######################
        return pos_x, pos_y, vel, yaw # note that yaw is in radian

    # Task 2: Longtitudal Controller
    # Based on all unreached waypoints, and your current vehicle state, decide your velocity
    def longititudal_controller(self, curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints):

        ####################### TODO: Your TASK 2 code starts Here #######################
        target_velocity = 10
        # 12 + 8 : exceed time limit
        target_velocity_str = 13
        target_velocity_trn = 8

        if len(future_unreached_waypoints) < 2:
            return target_velocity_str
        
        wp_cur = np.array([curr_x, curr_y])
        wp_nxt1 = np.array(future_unreached_waypoints[0])
        wp_nxt2 = np.array(future_unreached_waypoints[1])

        vec_cur_nxt1 = wp_nxt1 - wp_cur
        vec_next1_nxt2 = wp_nxt2 - wp_nxt1

        ang_diff = np.arctan2(vec_next1_nxt2[1], vec_next1_nxt2[0]) - np.arctan2(vec_cur_nxt1[1], vec_cur_nxt1[0])
        ang_diff = np.arctan2(np.sin(ang_diff), np.cos(ang_diff)) # normalize to [-pi, pi]

        if abs(ang_diff) < np.pi/6:
            target_velocity = target_velocity_str 
        else:
            target_velocity = target_velocity_trn
        ####################### TODO: Your TASK 2 code ends Here #######################
        return target_velocity


    # Task 3: Lateral Controller (Pure Pursuit)
    def pure_pursuit_lateral_controller(self, curr_x, curr_y, curr_yaw, target_point, future_unreached_waypoints):

        ####################### TODO: Your TASK 3 code starts Here #######################
        target_steering = 0
        look_dis = 5
        look_pot = None
        for wp in future_unreached_waypoints:
            wx, wy = wp
            dis_to_wp = math.sqrt((wx-curr_x)**2 + (wy-curr_y)**2)
            if dis_to_wp >= look_dis:
                look_pot = wp
                break

        if not look_pot:
            look_pot = target_point
        
        lx, ly = look_pot
        ld = math.sqrt((lx-curr_x)**2 + (ly-curr_y)**2)

        ang_lp = math.atan2(ly - curr_y, lx - curr_x)
        ang = ang_lp - curr_yaw

        ang = math.atan2(math.sin(ang), math.cos(ang))

        target_steering = math.atan2(2 * self.L *math.sin(ang), ld)
        

        ####################### TODO: Your TASK 3 code starts Here #######################
        return target_steering


    def execute(self, currentPose, target_point, future_unreached_waypoints):
        # Compute the control input to the vehicle according to the
        # current and reference pose of the vehicle
        # Input:
        #   currentPose: ModelState, the current state of the vehicle
        #   target_point: [target_x, target_y]
        #   future_unreached_waypoints: a list of future waypoints[[target_x, target_y]]
        # Output: None

        curr_x, curr_y, curr_vel, curr_yaw = self.extract_vehicle_info(currentPose)

        # Acceleration Profile
        if self.log_acceleration:
            acceleration = (curr_vel- self.prev_vel) * 100 # Since we are running in 100Hz



        target_velocity = self.longititudal_controller(curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints)
        target_steering = self.pure_pursuit_lateral_controller(curr_x, curr_y, curr_yaw, target_point, future_unreached_waypoints)


        #Pack computed velocity and steering angle into Ackermann command
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = target_velocity
        newAckermannCmd.steering_angle = target_steering

        # Publish the computed control input to vehicle model
        self.controlPub.publish(newAckermannCmd)

    def stop(self):
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = 0
        self.controlPub.publish(newAckermannCmd)
