# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# import numpy as np

# # Function to calculate distance to ArUco marker
# def calculate_distance_to_aruco(frame, aruco_dict, aruco_params, camera_matrix, dist_coeffs, marker_length, target_id, scaling_factor=1.0):
#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#     corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    
#     distances = []
#     if ids is not None:
#         rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)
        
#         for i, marker_id in enumerate(ids.flatten()):
#             if marker_id == target_id:
#                 tvec = tvecs[i][0]
#                 distance = np.linalg.norm(tvec) * scaling_factor
#                 distances.append(distance)
                
#                 cv2.aruco.drawDetectedMarkers(frame, [corners[i]], ids[i])
#                 cv2.aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvecs[i], tvec, marker_length / 2)
                
#                 cv2.putText(frame, f"ID: {marker_id} Dist: {distance:.2f}m",
#                             tuple(corners[i][0][0].astype(int)),
#                             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

#     return distances, frame


# class VideoSubscriber(Node):
#     def __init__(self):
#         super().__init__('video_subscriber')
#         self.bridge = CvBridge()

#         # Subscribe to the topic where the video stream is published
#         self.subscription = self.create_subscription(
#             Image,
#             '/camera',  # ROS2 topic name (change this to your topic)
#             self.listener_callback,
#             10
#         )
#         self.subscription  # prevent unused variable warning

#         # Camera parameters
#         self.camera_matrix = np.array([
#             [955.07368215, 0.0, 456.76903303],
#             [0.0, 952.34660273, 352.62128764],
#             [0.0, 0.0, 1.0]
#         ])
#         self.dist_coeffs = np.array([-0.205534411, 1.76303224, -0.00861027428, -0.000840521784, -4.46735135])
#         self.marker_length = 0.096  # Marker side length in meters
#         self.target_id = 10  # Target marker ID
#         self.scaling_factor = 0.9174  # Scaling factor for distance correction
#         self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
#         self.aruco_params = cv2.aruco.DetectorParameters_create()

#     def listener_callback(self, msg):
#         try:
#             # Convert ROS2 Image message to OpenCV image
#             frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

#             # Calculate the distance to the target ArUco marker
#             distances, frame = calculate_distance_to_aruco(frame, self.aruco_dict, self.aruco_params, 
#                                                            self.camera_matrix, self.dist_coeffs,
#                                                            self.marker_length, self.target_id,
#                                                            self.scaling_factor)
            
#             # Display the frame with distance info
#             cv2.imshow("Video Stream", frame)

#             # Press 'q' to quit
#             if cv2.waitKey(1) & 0xFF == ord('q'):
#                 self.get_logger().info('Exiting...')
#                 cv2.destroyAllWindows()
#                 return

#         except Exception as e:
#             self.get_logger().info('Error: %s' % str(e))


# def main(args=None):
#     rclpy.init(args=args)
#     video_subscriber = VideoSubscriber()

#     try:
#         rclpy.spin(video_subscriber)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         video_subscriber.destroy_node()
#         rclpy.shutdown()
#         cv2.destroyAllWindows()


# if __name__ == '__main__':
#     main()


# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# import numpy as np
# from geometry_msgs.msg import Point  # For publishing x, y, z distance

# # Function to calculate distance to ArUco marker and get the x, y, z components
# def calculate_distance_to_aruco(frame, aruco_dict, aruco_params, camera_matrix, dist_coeffs, marker_length, target_id, scaling_factor=1.0):
#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#     corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    
#     distances = []
#     marker_positions = []  # To store the positions (x, y, z)
#     if ids is not None:
#         rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)
        
#         for i, marker_id in enumerate(ids.flatten()):
#             if marker_id == target_id:
#                 tvec = tvecs[i][0]
#                 x, y, z = tvec[0], tvec[1], tvec[2]
#                 distance = np.linalg.norm(tvec) * scaling_factor
#                 distances.append(distance)
#                 marker_positions.append((x, y, z))  # Store the x, y, z coordinates
                
#                 # Draw the marker and its pose on the frame
#                 cv2.aruco.drawDetectedMarkers(frame, [corners[i]], ids[i])
#                 cv2.aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvecs[i], tvec, marker_length / 2)
                
#                 # Display the distance on the frame
#                 cv2.putText(frame, f"ID: {marker_id} Dist: {distance:.2f}m",
#                             tuple(corners[i][0][0].astype(int)),
#                             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

#     return marker_positions, distances, frame


# class VideoSubscriber(Node):
#     def __init__(self):
#         super().__init__('video_subscriber')
#         self.bridge = CvBridge()

#         # Subscribe to the topic where the video stream is published
#         self.subscription = self.create_subscription(
#             Image,
#             '/camera',  # ROS2 topic name (change this to your topic)
#             self.listener_callback,
#             10
#         )
#         self.subscription  # prevent unused variable warning

#         # Create a publisher to publish the x, y, z distances
#         self.publisher = self.create_publisher(Point, '/aruco_marker_position', 10)

#         # Camera parameters
#         self.camera_matrix = np.array([
#             [955.07368215, 0.0, 456.76903303],
#             [0.0, 952.34660273, 352.62128764],
#             [0.0, 0.0, 1.0]
#         ])
#         self.dist_coeffs = np.array([-0.205534411, 1.76303224, -0.00861027428, -0.000840521784, -4.46735135])
#         self.marker_length = 0.096  # Marker side length in meters
#         self.target_id = 10  # Target marker ID
#         self.scaling_factor = 0.5  # Scaling factor for distance correction
#         self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
#         self.aruco_params = cv2.aruco.DetectorParameters_create()

#     def listener_callback(self, msg):
#         try:
#             # Convert ROS2 Image message to OpenCV image
#             frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

#             # Get the x, y, z positions of the target marker
#             marker_positions, distances, frame = calculate_distance_to_aruco(
#                 frame, self.aruco_dict, self.aruco_params, 
#                 self.camera_matrix, self.dist_coeffs,
#                 self.marker_length, self.target_id,
#                 self.scaling_factor
#             )
            
#             # Publish the x, y, z coordinates if a target marker is found
#             if marker_positions:
#                 for position in marker_positions:
#                     x, y, z = position
#                     msg = Point()
#                     msg.x = x
#                     msg.y = y
#                     msg.z = z
#                     self.publisher.publish(msg)
#                     self.get_logger().info(f"Published position: x={x:.2f}, y={y:.2f}, z={z:.2f}")

#             # Display the frame with distance info
#             cv2.imshow("Video Stream", frame)

#             # Press 'q' to quit
#             if cv2.waitKey(1) & 0xFF == ord('q'):
#                 self.get_logger().info('Exiting...')
#                 cv2.destroyAllWindows()
#                 return

#         except Exception as e:
#             self.get_logger().info('Error: %s' % str(e))


# def main(args=None):
#     rclpy.init(args=args)
#     video_subscriber = VideoSubscriber()

#     try:
#         rclpy.spin(video_subscriber)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         video_subscriber.destroy_node()
#         rclpy.shutdown()
#         cv2.destroyAllWindows()


# if __name__ == '__main__':
#     main()


# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# import numpy as np
# from geometry_msgs.msg import Point  # For publishing x, y, z distance

# # Function to calculate distance to ArUco marker and get the x, y, z components
# def calculate_distance_to_aruco(frame, aruco_dict, aruco_params, camera_matrix, dist_coeffs, marker_length, target_id, scaling_factor=0.9174):
#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#     corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    
#     distances = []
#     marker_positions = []  # To store the positions (x, y, z)
#     if ids is not None:
#         rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)
        
#         for i, marker_id in enumerate(ids.flatten()):
#             if marker_id == target_id:
#                 tvec = tvecs[i][0]
#                 # Swap the coordinates as requested
#                 x, y, z = tvec[2], tvec[0], tvec[1]  # Swap x, y, z
#                 distance = np.linalg.norm(tvec) * scaling_factor
#                 distances.append(distance)
#                 marker_positions.append((x, y, z))  # Store the swapped x, y, z coordinates
                
#                 # Draw the marker and its pose on the frame
#                 cv2.aruco.drawDetectedMarkers(frame, [corners[i]], ids[i])
#                 cv2.aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvecs[i], tvec, marker_length / 2)
                
#                 # Display the distance on the frame
#                 cv2.putText(frame, f"ID: {marker_id} Dist: {distance:.2f}m",
#                             tuple(corners[i][0][0].astype(int)),
#                             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

#     return marker_positions, distances, frame


# class VideoSubscriber(Node):
#     def __init__(self):
#         super().__init__('video_subscriber')
#         self.bridge = CvBridge()

#         # Subscribe to the topic where the video stream is published
#         self.subscription = self.create_subscription(
#             Image,
#             '/camera',  # ROS2 topic name (change this to your topic)
#             self.listener_callback,
#             10
#         )
#         self.subscription  # prevent unused variable warning

#         # Create a publisher to publish the x, y, z distances
#         self.publisher = self.create_publisher(Point, '/aruco_marker_position', 10)

#         # Camera parameters
#         self.camera_matrix = np.array([
#             [955.07368215, 0.0, 456.76903303],
#             [0.0, 952.34660273, 352.62128764],
#             [0.0, 0.0, 1.0]
#         ])
#         self.dist_coeffs = np.array([-0.205534411, 1.76303224, -0.00861027428, -0.000840521784, -4.46735135])
#         self.marker_length = 0.096  # Marker side length in meters
#         self.target_id = 10  # Target marker ID
#         self.scaling_factor = 0.9174  # Scaling factor for distance correction
#         self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
#         self.aruco_params = cv2.aruco.DetectorParameters_create()

#     def listener_callback(self, msg):
#         try:
#             # Convert ROS2 Image message to OpenCV image
#             frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

#             # Get the x, y, z positions of the target marker
#             marker_positions, distances, frame = calculate_distance_to_aruco(
#                 frame, self.aruco_dict, self.aruco_params, 
#                 self.camera_matrix, self.dist_coeffs,
#                 self.marker_length, self.target_id,
#                 self.scaling_factor
#             )
            
#             # Publish the x, y, z coordinates if a target marker is found
#             if marker_positions:
#                 for position in marker_positions:
#                     x, y, z = position
#                     msg = Point()
#                     msg.x = x
#                     msg.y = y
#                     msg.z = z
#                     self.publisher.publish(msg)
#                     self.get_logger().info(f"Published position: x={x:.2f}, y={y:.2f}, z={z:.2f}")

#             # Display the frame with distance info
#             cv2.imshow("Video Stream", frame)

#             # Press 'q' to quit
#             if cv2.waitKey(1) & 0xFF == ord('q'):
#                 self.get_logger().info('Exiting...')
#                 cv2.destroyAllWindows()
#                 return

#         except Exception as e:
#             self.get_logger().info('Error: %s' % str(e))


# def main(args=None):
#     rclpy.init(args=args)
#     video_subscriber = VideoSubscriber()

#     try:
#         rclpy.spin(video_subscriber)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         video_subscriber.destroy_node()
#         rclpy.shutdown()
#         cv2.destroyAllWindows()


# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Point  # For publishing x, y, z distance

class VideoSubscriber(Node):
    def __init__(self):
        super().__init__('video_subscriber')
        self.bridge = CvBridge()

        # Configuration Variables - Defined in one place
        self.config = {
            "camera_matrix": np.array([
                [955.07368215, 0.0, 456.76903303],
                [0.0, 952.34660273, 352.62128764],
                [0.0, 0.0, 1.0]
            ]),
            "dist_coeffs": np.array([-0.205534411, 1.76303224, -0.00861027428, -0.000840521784, -4.46735135]),
            "marker_length": 0.096,  # Marker side length in meters
            "target_id": 10,  # Target marker ID
            "scaling_factor": 0.9174,  # Scaling factor for distance correction
            "aruco_dict": cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50),
            "aruco_params": cv2.aruco.DetectorParameters_create()
        }

        # Subscribe to the topic where the video stream is published
        self.subscription = self.create_subscription(
            Image,
            '/camera',  # ROS2 topic name (change this to your topic)
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Create a publisher to publish the x, y, z distances
        self.publisher = self.create_publisher(Point, '/aruco_marker_position', 10)

    def listener_callback(self, msg):
        try:
            # Convert ROS2 Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Get the x, y, z positions of the target marker
            marker_positions, distances, frame = calculate_distance_to_aruco(
                frame, self.config["aruco_dict"], self.config["aruco_params"],
                self.config["camera_matrix"], self.config["dist_coeffs"],
                self.config["marker_length"], self.config["target_id"],
                self.config["scaling_factor"]
            )
            
            # Publish the x, y, z coordinates if a target marker is found
            if marker_positions:
                for position in marker_positions:
                    x, y, z = position
                    msg = Point()
                    msg.x = x
                    msg.y = y
                    msg.z = z
                    self.publisher.publish(msg)
                    self.get_logger().info(f"Published position: x={x:.2f}, y={y:.2f}, z={z:.2f}")

            # Display the frame with distance info
            cv2.imshow("Video Stream", frame)

            # Press 'q' to quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info('Exiting...')
                cv2.destroyAllWindows()
                return

        except Exception as e:
            self.get_logger().info('Error: %s' % str(e))


def calculate_distance_to_aruco(frame, aruco_dict, aruco_params, camera_matrix, dist_coeffs, marker_length, target_id, scaling_factor=0.9174):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    
    distances = []
    marker_positions = []  # To store the positions (x, y, z)
    if ids is not None:
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)
        
        for i, marker_id in enumerate(ids.flatten()):
            if marker_id == target_id:
                tvec = tvecs[i][0]
                # Swap the coordinates as requested
                x, y, z = tvec[2], tvec[0], tvec[1]  # Swap x, y, z
                distance = np.linalg.norm(tvec) * scaling_factor
                distances.append(distance)
                marker_positions.append((x, y, z))  # Store the swapped x, y, z coordinates
                
                # Draw the marker and its pose on the frame
                cv2.aruco.drawDetectedMarkers(frame, [corners[i]], ids[i])
                cv2.aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvecs[i], tvec, marker_length / 2)
                
                # Display the distance on the frame
                cv2.putText(frame, f"ID: {marker_id} Dist: {distance:.2f}m",
                            tuple(corners[i][0][0].astype(int)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    return marker_positions, distances, frame


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
