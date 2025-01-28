import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, Point
import time

class TelloControl(Node):
    def __init__(self):
        super().__init__('simple_control')

        # Initialize publishers
        self.publisher_land = self.create_publisher(Empty, 'land', 1)
        self.publisher_takeoff = self.create_publisher(Empty, 'takeoff', 1)
        self.publisher_velocity = self.create_publisher(Twist, 'control', 1)

        # Create a subscriber to get position data
        self.create_subscription(Point, 'aruco_marker_position', self.position_callback, 10)

        # Variable to store current position data
        self.current_position = Point()
        self.y_tolerance = 0.1  # Tolerance range for the y-coordinate
        self.position_received = False  # Flag to indicate if position is received

    def position_callback(self, msg):
        """Update the current position from the ArUco marker."""
        self.current_position = msg
        # self.get_logger().info(f"Updated position: x={msg.x}, y={msg.y}, z={msg.z}")

    # def reposition(self):
    #     self.get_logger().info(f"Repositioning... Current y={self.current_position.y}")

    #     msg = Twist()

    #     # Adjust movement based on current y position
    #     if self.current_position.y > self.y_tolerance:
    #         msg.linear.x = 10.0  # Move forward (positive direction)
    #     elif self.current_position.y < -self.y_tolerance:
    #         msg.linear.x = -10.0  # Move backward (negative direction)

    #     # Publish the velocity command
    #     self.publisher_velocity.publish(msg)
    #     time.sleep(0.5)  # Allow time for movement

    def control_loop(self):
        """Main control logic."""
        self.get_logger().info("Starting drone operation...")

        # Proceed with takeoff and repositioning
        self.get_logger().info("Sending takeoff command...")
        self.publisher_takeoff.publish(Empty())
        time.sleep(5)  # Wait for the drone to take off

        msg = Twist()

        if self.current_position.y > self.y_tolerance:        
            msg.linear.y = 50.0
            self.publisher_velocity.publish(msg)

        elif self.current_position.y < self.y_tolerance:        
                msg.linear.y = 50.0
                self.publisher_velocity.publish(msg)
        else:
            msg.linear.x = 20.0
            self.publisher_velocity.publish(msg)

        self.get_logger().info("Reposition complete...")

        self.get_logger().info("Sending land command...")
        self.publisher_land.publish(Empty())

        self.get_logger().info("Operation completed. Shutting down...")

        # rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    tello_control_node = TelloControl()

    # Start control loop
    tello_control_node.control_loop()

    # Spin the node to keep it alive
    rclpy.spin(tello_control_node)

if __name__ == '__main__':
    main()
