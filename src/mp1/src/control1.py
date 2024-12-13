import rospy
import math
from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDriveStamped

class PIDController:
    def __init__(self):
        rospy.init_node('pid_controller', anonymous=True)
        self.rate = rospy.Rate(50)  # 50 Hz control loop

        # PID gains (to be tuned)
        self.kp = 0.5
        self.ki = 0.01
        self.kd = 0.1

        # PID variables
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None

        # Steering parameters
        self.max_steering_angle = 0.35  # Max steering angle in radians
        self.min_steering_angle = -0.35

        # Speed parameters
        self.default_speed = 0.4  # Default speed in m/s
        self.recovery_speed = 0.2  # Reduced speed during recovery

        # Lane detection
        self.lane_detected = False
        self.correction_counter = 0
        self.max_correction_steps = 50  # Allow up to 50 iterations of recovery

        # Vehicle control message
        self.drive_msg = AckermannDriveStamped()
        self.drive_msg.header.frame_id = "f1tenth_control"
        self.drive_msg.drive.speed = self.default_speed

        # ROS publishers and subscribers
        self.drive_pub = rospy.Publisher(
            "/vesc/low_level/ackermann_cmd_mux/input/navigation",
            AckermannDriveStamped,
            queue_size=1,
        )
        rospy.Subscriber('lane_detection/error', Float32, self.error_callback)

        # Current error
        self.current_error = 0.0
        self.last_steering_angle = 0.0

    def error_callback(self, msg):
        """Callback to handle lane detection error."""
        self.current_error = msg.data
        if msg.data > 1000:  # Threshold for lane detection loss
            self.lane_detected = False
        else:
            self.lane_detected = True
            self.correction_counter = 0

    def compute_steering_angle(self, error, delta_time):
        """Compute the steering angle using PID control."""
        self.integral += error * delta_time
        derivative = (error - self.prev_error) / delta_time

        # PID formula
        steering_angle = (
            self.kp * error + self.ki * self.integral + self.kd * derivative
        )

        # Clamp to max/min steering angle
        steering_angle = max(
            self.min_steering_angle, min(self.max_steering_angle, steering_angle)
        )

        self.prev_error = error
        return steering_angle

    def run(self):
        """Main control loop."""
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            if self.prev_time is None:
                delta_time = 0.0
            else:
                delta_time = (current_time - self.prev_time).to_sec()

            if delta_time == 0:
                delta_time = 0.0001

            if not self.lane_detected:
                if self.correction_counter < self.max_correction_steps:
                    # Recovery behavior: reverse last steering angle
                    steering_angle = -self.last_steering_angle
                    self.drive_msg.drive.speed = self.recovery_speed
                    self.correction_counter += 1
                else:
                    rospy.logwarn("Lane lost. Stopping the vehicle.")
                    self.drive_msg.drive.speed = 0.0  # Stop the vehicle
                    steering_angle = 0.0
            else:
                # Normal behavior: Compute PID steering angle
                steering_angle = self.compute_steering_angle(
                    self.current_error, delta_time
                )
                self.drive_msg.drive.speed = self.default_speed

            # Publish drive message
            self.last_steering_angle = steering_angle
            self.drive_msg.header.stamp = current_time
            self.drive_msg.drive.steering_angle = -steering_angle  # Invert for control
            self.drive_pub.publish(self.drive_msg)

            self.prev_time = current_time
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = PIDController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
