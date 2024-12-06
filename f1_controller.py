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

    def pure_pursuit_lateral_control(self, curr_x, curr_y, curr_yaw, waypoints):
        lookahead_distance = 1.0  # Adjust based on F1TENTH's speed
        target_point = None

        for point in waypoints:
            distance = math.sqrt((point[0] - curr_x)**2 + (point[1] - curr_y)**2)
            if distance > lookahead_distance:
                target_point = point
                break

        if target_point is None:
            target_point = waypoints[-1]

        dx = target_point[0] - curr_x
        dy = target_point[1] - curr_y
        angle_to_point = math.atan2(dy, dx)
        alpha = angle_to_point - curr_yaw
        steering_angle = math.atan2(2 * self.L * math.sin(alpha), lookahead_distance)

        return steering_angle

    def longitudinal_control(self, current_velocity, waypoints):
        max_velocity = 0.5  # Example max velocity
        return max_velocity  # Simplified constant velocity

    def execute(self, waypoints):
        curr_x, curr_y, curr_vel, curr_yaw = self.extract_vehicle_info()

        if curr_x is None:
            rospy.logwarn("Waiting for odometry data...")
            return

        target_steering = self.pure_pursuit_lateral_control(curr_x, curr_y, curr_yaw, waypoints)
        target_velocity = self.longitudinal_control(curr_vel, waypoints)

        # Publish control command
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = target_steering
        drive_msg.drive.speed = target_velocity
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
