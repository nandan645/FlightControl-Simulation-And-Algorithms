### Monocular ORB-SLAM3 Implementation

#### Overview
A C++ implementation of monocular SLAM using ORB-SLAM3 and webcam input. The system performs real-time camera tracking and distance estimation using calibrated camera parameters.

#### Requirements
- OpenCV
- ORB-SLAM3
- Calibrated webcam with YAML calibration file

#### Code

```
#include <iostream>
#include <opencv2/opencv.hpp>
#include <System.h>
#include <opencv2/core/core.hpp>

using namespace std;
using namespace ORB_SLAM3;

void loadCameraCalibration(const string &file_name, cv::Mat &camera_matrix, cv::Mat &dist_coeffs) {
    // Open the YAML calibration file
    cv::FileStorage fs(file_name, cv::FileStorage::READ);
    
    // Check if the file was opened successfully
    if (!fs.isOpened()) {
        cerr << "Error: Could not open calibration file " << file_name << endl;
        exit(-1);  // Exit the program if calibration file cannot be loaded
    }

    // Read the camera matrix (intrinsics)
    camera_matrix = cv::Mat::eye(3, 3, CV_64F);  // Initialize to identity matrix
    fs["Camera1.fx"] >> camera_matrix.at<double>(0, 0);
    fs["Camera1.fy"] >> camera_matrix.at<double>(1, 1);
    fs["Camera1.cx"] >> camera_matrix.at<double>(0, 2);
    fs["Camera1.cy"] >> camera_matrix.at<double>(1, 2);

    // Read distortion coefficients
    dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);  // Initialize to zero
    fs["Camera1.k1"] >> dist_coeffs.at<double>(0);
    fs["Camera1.k2"] >> dist_coeffs.at<double>(1);
    fs["Camera1.p1"] >> dist_coeffs.at<double>(2);
    fs["Camera1.p2"] >> dist_coeffs.at<double>(3);
    fs["Camera1.k3"] >> dist_coeffs.at<double>(4);
    
    cout << "Successfully loaded calibration file!" << endl;
}

int main(int argc, char** argv) {
    if (argc != 3) {
        cerr << "Usage: " << argv[0] << " <path_to_vocabulary> <path_to_config>" << endl;
        return -1;
    }

    // Get the vocabulary and config paths from arguments
    string vocabulary_file = argv[1];
    string config_file = argv[2];

    // Initialize the ORB-SLAM3 system
    System slam_system(vocabulary_file, config_file, System::MONOCULAR);

    // Open the webcam (or provide the path to a video file)
    cv::VideoCapture cap(2, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        cerr << "Error: Unable to open the camera." << endl;
        return -1;
    }

    // Camera calibration
    string calibration_file = "/home/abhinandan/Documents/ORB_SLAM3/Examples/Monocular/webcam.yaml";  // Replace with actual absolute path
    cv::Mat camera_matrix, dist_coeffs;
    loadCameraCalibration(calibration_file, camera_matrix, dist_coeffs);

    cout << "Press 'q' to exit." << endl;

    while (true) {
        cv::Mat frame;
        cap >> frame;  // Capture frame from webcam
        if (frame.empty()) {
            cerr << "Error: Unable to capture frame." << endl;
            break;
        }

        // Convert the frame to grayscale as required by ORB-SLAM3
        cv::Mat gray_frame;
        cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);

        // Pass the frame to the SLAM system
        double timestamp = static_cast<double>(cv::getTickCount()) / cv::getTickFrequency();
        Sophus::SE3f pose = slam_system.TrackMonocular(gray_frame, timestamp);

        // If pose is available, extract the camera position
        // if (pose.translation().norm() > 1e-6) {  // Check if the pose is not identity
        //     Eigen::Vector3f camera_position = pose.translation();
        //     cout << "Camera position: " << camera_position.transpose() << endl;

        //     // Calculate distance to the object (assume the object is at a specific depth)
        //     float depth = camera_position.z();  // Assuming z is the depth of the object in the camera's coordinate system
        //     cout << "Distance to object: " << depth << " meters" << endl;
        // }

        // If pose is available, extract the camera position
        if (pose.translation().norm() > 1e-6) {  // Check if the pose is not identity
            Eigen::Vector3f camera_position = pose.translation();
            
            // Rearrange the order of components from y, z, x to x, y, z
            Eigen::Vector3f reordered_position(camera_position.z(), camera_position.x(), camera_position.y());
            
            cout << "Camera position: " << reordered_position.transpose() << endl;

            // Calculate distance to the object (assume the object is at a specific depth)
            float depth = reordered_position.x();  // Using reordered x as the depth of the object
            cout << "Distance to object: " << depth << " meters" << endl;
        }

        // Display the frame
        cv::imshow("ORB-SLAM3 Monocular", frame);

        // Exit on 'q' key press
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    // Shutdown SLAM system


    slam_system.Shutdown();

    // Release the video capture and close all OpenCV windows
    cap.release();
    cv::destroyAllWindows();

    return 0;
}
```

#### Usage
```bash
./program <path_to_vocabulary> <path_to_config>
```

#### Demo Video

https://github.com/user-attachments/assets/e41618ac-e63e-49c0-8473-e5a38d7b6c53
