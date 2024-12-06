import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import math

class F1TenthController:
    def __init__(self):
        rospy.init_node('f1tenth_controller', anonymous=True)
        self.control_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)

        # Subscribe to the odometry and waypoints topics
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/perception/waypoints', Float32MultiArray, self.waypoints_callback)

        self.current_state = None
        self.L = 0.33  # Wheelbase for F1TENTH
        self.waypoints = []  # To store received waypoints

    def waypoints_callback(self, msg):
        """
        Callback to handle incoming waypoints from the perception system.

        Args:
            msg (Float32MultiArray): List of waypoints [x1, y1, x2, y2, ...].
        """
        # Convert the flat array into a list of (x, y) waypoints
        self.waypoints = [(msg.data[i], msg.data[i+1]) for i in range(0, len(msg.data), 2)]

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

    def execute(self):
        """
        Compute and publish control commands based on received waypoints.
        """
        if not self.waypoints or len(self.waypoints) < 2:
            rospy.logwarn("No waypoints received or insufficient waypoints!")
            self.stop()
            return

        # Compute steering angle using pure pursuit
        steering_angle = self.pure_pursuit_lateral_control(self.waypoints)

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
