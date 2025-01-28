import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import time

class TelloControl(Node):
    def __init__(self):
        super().__init__('simple_control')

        # Initialize publishers
        self.publisher_land = self.create_publisher(Empty, 'land', 1)
        self.publisher_takeoff = self.create_publisher(Empty, 'takeoff', 1)
        self.publisher_velocity = self.create_publisher(Twist, 'control', 1)

        # A short delay before taking action
        self.timer = self.create_timer(0.1, self.timer_callback)

    def move_forward(self, duration=3.0):
        self.get_logger().info(f"Moving forward for {duration} seconds.")
        msg = Twist()
        msg.linear.y = 50.0  # Move forward at speed 50
        self.get_logger().info(f"Publishing velocity: {msg.linear.x}, {msg.linear.y}, {msg.linear.z}")
        self.publisher_velocity.publish(msg)
        time.sleep(duration)  # Move forward for 'duration' seconds
        self.stop_drone()

    def stop_drone(self):
        self.get_logger().info("Stopping the drone.")
        msg = Twist()
        self.publisher_velocity.publish(msg)

    def timer_callback(self):
        # Step 1: Takeoff
        self.publisher_takeoff.publish(Empty())
        self.get_logger().info("Taking off...")

        # Step 2: Move forward
        time.sleep(6)  # Wait for takeoff to complete
        self.get_logger().info("Moving forward...")
        self.move_forward(duration=3)  # Move forward for 3 seconds

        # Step 3: Land
        self.publisher_land.publish(Empty())
        self.get_logger().info("Landing...")

        # Shutdown the node after completing the task
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    tello_control_node = TelloControl()

    # Spin the node to keep it alive
    rclpy.spin(tello_control_node)

if __name__ == '__main__':
    main()
