import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import numpy as np

class WaypointPublisher:
    def __init__(self):
        rospy.init_node('waypoint_publisher', anonymous=True)

        # Publisher for the waypoints as a Path message
        self.waypoint_pub = rospy.Publisher('/localized_waypoints', Path, queue_size=10)

        # Parameters
        self.scale_x = 0.1  # Real-world distance per pixel in x direction (meters per pixel)
        self.scale_y = 0.1  # Real-world distance per pixel in y direction (meters per pixel)
        self.frequency = 10  # Hz

    def generate_waypoints(self, center_fit, ploty):
        """
        Generate waypoints based on the detected centerline fit.
        """
        waypoints = []

        if center_fit is not None:
            # Compute x values for the polynomial fit
            center_fitx = center_fit[0] * ploty ** 2 + center_fit[1] * ploty + center_fit[2]

            for i in range(len(ploty)):
                # Convert from pixel coordinates to real-world distances
                real_x = center_fitx[i] * self.scale_x
                real_y = ploty[i] * self.scale_y

                # Append the waypoint (real_x, real_y)
                waypoints.append((real_x, real_y))
        else:
            rospy.logwarn("No centerline detected. Waypoints cannot be generated.")

        return waypoints

    def publish_waypoints(self, waypoints):
        """
        Publish the waypoints as a ROS Path message.
        """
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()

        for waypoint in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()

            # Assign waypoint position
            pose.pose.position.x = waypoint[0]  # x in meters
            pose.pose.position.y = waypoint[1]  # y in meters
            pose.pose.position.z = 0.0  # Assume flat ground

            # Orientation is not defined for waypoints, so use default
            pose.pose.orientation.w = 1.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0

            path.poses.append(pose)

        # Publish the waypoints
        self.waypoint_pub.publish(path)
        rospy.loginfo(f"Published {len(waypoints)} waypoints.")

    def run(self, center_fit, ploty):
        """
        Main loop to generate and publish waypoints.
        """
        rate = rospy.Rate(self.frequency)

        while not rospy.is_shutdown():
            # Generate waypoints
            waypoints = self.generate_waypoints(center_fit, ploty)

            # Publish the waypoints
            self.publish_waypoints(waypoints)

            rate.sleep()

if __name__ == "__main__":
    # Example usage
    # Assume `center_fit` is the second-order polynomial coefficients for the centerline,
    # and `ploty` is the y-coordinate array for the plot.

    # Example data (Replace these with your actual detection results)
    center_fit = [0.001, -0.5, 300]  # Example polynomial coefficients
    ploty = np.linspace(0, 480, num=480)  # Example y-coordinates for the detected line

    # Initialize the WaypointPublisher
    waypoint_publisher = WaypointPublisher()

    # Run the waypoint generation and publishing
    waypoint_publisher.run(center_fit, ploty)
