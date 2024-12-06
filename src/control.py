import rospy
import math
from std_msgs.msg import String, Bool, Float32, Float64, Float64MultiArray
from ackermann_msgs.msg import AckermannDriveStamped

class PIDController:
    def __init__(self):
        self.rate = rospy.Rate(50)  

        self.kp = 0.5  # tune
        self.ki = 0.01  # tune 
        self.kd = 0.1  # tune
        # PID variables
        self.prev_error = 0.0
        self.integral = 0.0
        
        self.max_steering_angle = 0.35  # max steering angle (in radians) add min

        # self.desired_speed = 1.0        #  speed (m/s)
        
        self.prev_time = None

        # #speeds (will need to tune hella) (in m/s)
        # self.max_velocity = 0.5 
        # self.mid_velocity = 1.0 
        # self.min_velocity = 0.5        

        # Subscribe to the lane detection error from studentVision
        rospy.Subscriber('lane_detection/error', Float32, self.error_callback) # sahej error publish topic
        
        self.drive_pub = rospy.Publisher("/vesc/low_level/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1) # need to figure out the structure of this topic, does it just take steering angles?
        
        self.current_error = 0.0
        
        self.drive_msg = AckermannDriveStamped()
        self.drive_msg.header.frame_id = "f1tenth_control"
        self.drive_msg.drive.speed = 0.4

        self.last_steering_angle = 0.0
        self.lane_detected = False

        self.correction_counter = 0 # counts 50 times for a sec when lane is not detected
        
    

    #fetch error from studentVision
    def error_callback(self, msg): 
        self.current_error = msg.data
        if msg.data > 1000:
            self.lane_detected = False
        else:
            self.lane_detected = True
            self.correction_counter = 0
    

    def run(self):
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            if self.prev_time is None:
                delta_time = 0.0
            else:
                delta_time = (current_time - self.prev_time).to_sec()

            if delta_time == 0:
                delta_time = 0.0001
            

            # if not self.lane_detected:
            #     if self.correction_counter < 50:
            #         steering_angle = (-self.last_steering_angle)
            #         self.correction_counter += 1
            # else:
            #     # PID calculations
            error = self.current_error
            self.integral += error * delta_time
            derivative = (error - self.prev_error) / delta_time
            
            steering_angle = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
            
            steering_angle = max(-self.max_steering_angle, min(self.max_steering_angle, steering_angle))
            self.prev_error = error

            
            # publish the drive message
            self.last_steering_angle = steering_angle
            print(steering_angle)
            self.drive_msg.header.stamp = current_time
            self.drive_msg.drive.steering_angle = -steering_angle
            self.drive_pub.publish(self.drive_msg)
            self.prev_time = current_time
            
            self.rate.sleep()
            
if __name__ == '__main__':

    rospy.init_node('pid_controller', anonymous=True)
    controller = PIDController()
    try:
        controller.run()
    except rospy.ROSInterruptException:
        pass
