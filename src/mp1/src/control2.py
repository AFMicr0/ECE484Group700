import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from ackermann_msgs.msg import AckermannDriveStamped

class LaneFollowingController:
    def __init__(self):
        # ROS publishers and subscribers
        self.sub_waypoints = rospy.Subscriber('lane_detection/Waypoints', Float32MultiArray, self.waypoints_callback, queue_size=1)
        self.pub_control = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)

        # PID controller gains
        self.kp = 1.0
        self.ki = 0.0
        self.kd = 0.1

        # Controller state
        self.integral_error = 0.0
        self.previous_error = 0.0
        self.target_speed = 2.0  # Desired speed in m/s

    def waypoints_callback(self, msg):
        try:
            # Reshape the waypoints into (N, 2) format
            waypoints = np.array(msg.data).reshape(-1, 2)

            # Compute steering angle and throttle
            steering_angle, speed = self.compute_control(waypoints)

            # Publish the control command
            self.publish_control(steering_angle, speed)
        except Exception as e:
            rospy.logerr(f"Error processing waypoints: {e}")

    def compute_control(self, waypoints):
        """
        Compute the control command based on waypoints.

        Args:
            waypoints (np.ndarray): Array of waypoints [[x1, y1], [x2, y2], ...].

        Returns:
            float: Steering angle in radians.
            float: Speed in m/s.
        """
        if len(waypoints) < 2:
            # Not enough waypoints to calculate control
            return 0.0, 0.0

        # Use the first two waypoints to compute heading error
        x1, y1 = waypoints[0]
        x2, y2 = waypoints[1]

        # Desired heading angle (based on waypoints)
        desired_heading = np.arctan2(y2 - y1, x2 - x1)

        # Current heading is assumed to be aligned with the car's x-axis
        current_heading = 0.0

        # Heading error
        error = desired_heading - current_heading

        # PID control for steering
        self.integral_error += error
        derivative_error = error - self.previous_error
        self.previous_error = error

        steering_angle = self.kp * error + self.ki * self.integral_error + self.kd * derivative_error

        # Limit steering angle to [-0.4189, 0.4189] radians (roughly ±24 degrees)
        steering_angle = max(min(steering_angle, 0.4189), -0.4189)

        # Speed control (constant for now, can be adjusted based on curvature)
        speed = self.target_speed

        return steering_angle, speed

    def publish_control(self, steering_angle, speed):
        """
        Publish control commands to the vehicle.

        Args:
            steering_angle (float): Steering angle in radians.
            speed (float): Speed in m/s.
        """
        control_msg = AckermannDriveStamped()
        control_msg.header.stamp = rospy.Time.now()
        control_msg.drive.steering_angle = steering_angle
        control_msg.drive.speed = speed
        self.pub_control.publish(control_msg)

if __name__ == '__main__':
    rospy.init_node('lane_following_controller', anonymous=True)
    controller = LaneFollowingController()
    rospy.spin()
