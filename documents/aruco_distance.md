### ArUco Marker Distance Calculator

A Python script that calculates real-time distance between a camera and ArUco markers using OpenCV.

```
import cv2
import numpy as np

def calculate_distance_to_aruco(frame, aruco_dict, aruco_params, camera_matrix, dist_coeffs, marker_length, target_id, scaling_factor=1.0):
    """
    Calculate the distance between the camera and a specific ArUco marker.
    
    Args:
        frame (ndarray): The current frame from the camera.
        aruco_dict (cv2.aruco.Dictionary): The ArUco dictionary used.
        aruco_params (cv2.aruco.DetectorParameters): Parameters for ArUco marker detection.
        camera_matrix (ndarray): The camera's intrinsic matrix.
        dist_coeffs (ndarray): The camera's distortion coefficients.
        marker_length (float): The real-world side length of the ArUco marker (meters).
        target_id (int): The ID of the target ArUco marker to detect.
        scaling_factor (float): Adjustment factor for the calculated distance.
    
    Returns:
        distances (list): List of distances to the target marker (meters).
    """
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detect ArUco markers in the frame
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    
    distances = []
    if ids is not None:
        # Estimate the pose of each detected marker
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)
        
        for i, marker_id in enumerate(ids.flatten()):
            if marker_id == target_id:  # Check if the marker ID matches the target ID
                # Extract the translation vector (tvec) for the marker
                tvec = tvecs[i][0]
                distance = np.linalg.norm(tvec) * scaling_factor  # Apply scaling factor
                distances.append(distance)
                
                # Draw the marker and its pose on the frame
                cv2.aruco.drawDetectedMarkers(frame, [corners[i]], ids[i])
                cv2.aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvecs[i], tvec, marker_length / 2)
                
                # Display the distance on the frame
                cv2.putText(frame, f"ID: {marker_id} Dist: {distance:.2f}m",
                            tuple(corners[i][0][0].astype(int)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    return distances, frame


# Main function to test the distance calculation
def main():
    # Camera parameters (provided)
    camera_matrix = np.array([
        [2495.97951, 0.0, 819.918478],
        [0.0, 2487.94672, 471.820610],
        [0.0, 0.0, 1.0]
    ])
    dist_coeffs = np.array([0.60545597, -12.0187385, 0.0142193654, 0.0104931733, 116.765507])

    # Define the ArUco marker dictionary and detector parameters
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
    aruco_params = cv2.aruco.DetectorParameters_create()

    # Define the real-world marker side length (meters)
    marker_length = 0.096  # 9.6 cm

    # Target marker ID
    target_id = 10

    # Scaling factor for distance correction
    scaling_factor = 0.5405  # Adjust based on observed results (use 1.0 for no scaling)

    # Open the camera
    cap = cv2.VideoCapture(2)
    if not cap.isOpened():
        print("Error: Could not open the camera.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read the frame.")
            break

        distances, frame = calculate_distance_to_aruco(frame, aruco_dict, aruco_params, camera_matrix, dist_coeffs, marker_length, target_id, scaling_factor)
        
        # Display the frame
        cv2.imshow("Frame", frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

```

#### Setup
```bash
pip install opencv-python numpy
```

#### Important Parameters to Adjust

Before running the code, adjust these parameters according to your setup:

1. `camera_matrix` and `dist_coeffs`: Replace with your camera's calibration data
2. `camera_index`: Change VideoCapture index (0, 1, or 2) to match your camera
3. `marker_length`: Set to your ArUco marker's real-world size in meters (default: 0.096m)
4. `target_id`: Set to your ArUco marker's ID (default: 10)
5. `scaling_factor`: Crucial for accurate measurements. Adjust based on test measurements (default: 0.5405)

#### Usage
Run the script:
```bash
python aruco_distance.py
```

Hold the ArUco marker in front of the camera. Press 'q' to quit.

The program will display the detected marker with its ID and real-time distance measurements in meters.

#### Example Preview

![preview](https://raw.githubusercontent.com/nandan645/FlightControl-Simulation-And-Algorithms/refs/heads/main/assets/Screenshot%20from%202025-01-07%2001-34-47.png)
