import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import math

class F1TenthController:

    def __init__(self):
        rospy.init_node('f1tenth_controller', anonymous=True)
        self.control_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.current_state = None
        self.L = 0.33  # Wheelbase for F1TENTH (approximate value)

    def odom_callback(self, data):
        self.current_state = data

    def extract_vehicle_info(self):
        if self.current_state is None:
            return None, None, None, None
        position = self.current_state.pose.pose
        orientation = position.orientation
        x = position.position.x
        y = position.position.y

        # Convert quaternion to yaw
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y**2 + orientation.z**2)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        linear = self.current_state.twist.twist.linear
        vel = math.sqrt(linear.x**2 + linear.y**2)

        return x, y, vel, yaw

    def pure_pursuit_lateral_control(self, waypoints):
        """
        Compute the steering angle using pure pursuit based on local waypoints.
        
        Args:
            waypoints: List of waypoints in the vehicle's local frame [(x1, y1), (x2, y2), ...].

        Returns:
            float: Steering angle in radians.
        """
        if not waypoints:
            self.stop()
            return
        
        lookahead_distance = 1.0  # Lookahead distance, adjust based on vehicle dynamics
        # lookahead_distance = 0.5 + 0.1 * velocity 
        L = self.L  # Wheelbase of the vehicle
        target_point = None

        # Find the lookahead point
        for point in waypoints:
            distance = math.sqrt(point[0]**2 + point[1]**2)
            if distance >= lookahead_distance:
                target_point = point
                break

        # If no suitable lookahead point, use the last waypoint
        if target_point is None:
            target_point = waypoints[-1]

        # Compute the steering angle
        x, y = target_point
        alpha = math.atan2(y, x)  # Angle between vehicle's heading and target point
        steering_angle = math.atan2(2 * L * math.sin(alpha), lookahead_distance)

        return steering_angle


    def longitudinal_control(self, current_velocity, waypoints):
        max_velocity = 0.5  # Example max velocity
        return max_velocity  # Simplified constant velocity

    def execute(self, waypoints):
        """
        Compute and publish control commands based on perception-generated waypoints.

        Args:
            waypoints: List of local waypoints [(x1, y1), (x2, y2), ...].
        """
        if not waypoints or len(waypoints) < 2:
            rospy.logwarn("Not enough waypoints to compute control!")
            return

        # Compute steering angle using pure pursuit
        steering_angle = self.pure_pursuit_lateral_control(waypoints)

        # Set constant velocity (e.g., 1 m/s)
        velocity = 1.0

        # Publish control commands
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = velocity
        self.control_pub.publish(drive_msg)


    def stop(self):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = 0
        self.control_pub.publish(drive_msg)

if __name__ == "__main__":
    try:
        controller = F1TenthController()
        rate = rospy.Rate(10)  # 10 Hz
        waypoints = [[2.0, 3.0], [4.0, 6.0], [6.0, 9.0]]  # Example waypoints
        while not rospy.is_shutdown():
            controller.execute(waypoints)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
