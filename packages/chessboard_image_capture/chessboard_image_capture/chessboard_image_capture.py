import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time

class ChessboardImageCapture(Node):
    def __init__(self):
        super().__init__('chessboard_image_capture')

        # Parameters
        self.output_dir = "captured_images"  # Directory to save images
        self.chessboard_size = (9, 6)  # Chessboard size (number of internal corners)
        self.image_prefix = "chessboard_"  # Prefix for saved images
        self.image_format = ".jpg"  # Image file format
        self.capture_delay = 2  # Minimum time (in seconds) between consecutive captures
        self.image_count = 0
        self.last_capture_time = 0  # Track the time of the last capture

        # Create output directory if it doesn't exist
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Subscribe to the image topic
        self.image_sub = self.create_subscription(
            Image,
            '/camera',  # Update with your image topic
            self.image_callback,
            10
        )
        self.get_logger().info("Chessboard image capture node initialized. Listening for images...")

    def image_callback(self, msg):
        # Convert ROS2 Image message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Try to find chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, self.chessboard_size, None)

        # If chessboard corners are found
        if ret:
            # Check time elapsed since last capture
            current_time = time.time()
            if current_time - self.last_capture_time > self.capture_delay:
                # Save the raw, unaltered frame
                raw_image_path = os.path.join(self.output_dir, f"{self.image_prefix}{self.image_count}{self.image_format}")
                cv2.imwrite(raw_image_path, frame)
                self.get_logger().info(f"Raw image captured and saved as {raw_image_path}")
                self.image_count += 1
                self.last_capture_time = current_time

            # Draw the corners for visualization
            frame_with_corners = cv2.drawChessboardCorners(frame.copy(), self.chessboard_size, corners, ret)

            # Display the frame with corners
            cv2.imshow("Camera Feed", frame_with_corners)
        else:
            # Display the original frame
            cv2.imshow("Camera Feed", frame)

        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ChessboardImageCapture()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node...")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
