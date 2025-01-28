import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# Function to calculate distance to ArUco marker
def calculate_distance_to_aruco(frame, aruco_dict, aruco_params, camera_matrix, dist_coeffs, marker_length, target_id, scaling_factor=1.0):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    
    distances = []
    if ids is not None:
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)
        
        for i, marker_id in enumerate(ids.flatten()):
            if marker_id == target_id:
                tvec = tvecs[i][0]
                distance = np.linalg.norm(tvec) * scaling_factor
                distances.append(distance)
                
                cv2.aruco.drawDetectedMarkers(frame, [corners[i]], ids[i])
                cv2.aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvecs[i], tvec, marker_length / 2)
                
                cv2.putText(frame, f"ID: {marker_id} Dist: {distance:.2f}m",
                            tuple(corners[i][0][0].astype(int)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    return distances, frame


class VideoSubscriber(Node):
    def __init__(self):
        super().__init__('video_subscriber')
        self.bridge = CvBridge()

        # Subscribe to the topic where the video stream is published
        self.subscription = self.create_subscription(
            Image,
            '/camera',  # ROS2 topic name (change this to your topic)
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Camera parameters
        self.camera_matrix = np.array([
            [955.07368215, 0.0, 456.76903303],
            [0.0, 952.34660273, 352.62128764],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.array([-0.205534411, 1.76303224, -0.00861027428, -0.000840521784, -4.46735135])
        self.marker_length = 0.096  # Marker side length in meters
        self.target_id = 10  # Target marker ID
        self.scaling_factor = 0.5  # Scaling factor for distance correction
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

    def listener_callback(self, msg):
        try:
            # Convert ROS2 Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Calculate the distance to the target ArUco marker
            distances, frame = calculate_distance_to_aruco(frame, self.aruco_dict, self.aruco_params, 
                                                           self.camera_matrix, self.dist_coeffs,
                                                           self.marker_length, self.target_id,
                                                           self.scaling_factor)
            
            # Display the frame with distance info
            cv2.imshow("Video Stream", frame)

            # Press 'q' to quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info('Exiting...')
                cv2.destroyAllWindows()
                return

        except Exception as e:
            self.get_logger().info('Error: %s' % str(e))


def main(args=None):
    rclpy.init(args=args)
    video_subscriber = VideoSubscriber()

    try:
        rclpy.spin(video_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        video_subscriber.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
