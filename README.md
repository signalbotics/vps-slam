# VPS SLAM (Visual Positioning System with SLAM)

A ROS2 package that implements visual positioning using Google Street View images combined with real-time camera feed and GPS data. This system performs feature matching between live camera images and Street View references to enhance localization accuracy.

## Features

- Real-time image processing from ZED stereo camera
- GPS-based Google Street View image retrieval
- Feature detection and matching using ORB features
- Robust matching with RANSAC algorithm
- Homography computation between matched images
- ROS2 integration with standard message types

## Prerequisites

### System Requirements
- Ubuntu 22.04 (or compatible Linux distribution)
- ROS2 Humble
- C++17 or later
- Python 3.8+

### Dependencies
- OpenCV 4.x
- libcurl
- Eigen3
- nlohmann_json
- ROS2 packages:
  - rclcpp
  - sensor_msgs
  - cv_bridge
  - geometry_msgs
  - nav_msgs
- ZED SDK (for ZED camera support)

## Installation

1. Create a ROS2 workspace (if you haven't already):
    ```
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```

2. Clone the repository:
    ```
    git clone https://github.com/yourusername/vps_slam.git
    ```

3. Install dependencies:
    ```
    sudo apt-get update
    sudo apt-get install libcurl4-openssl-dev
    sudo apt-get install python3-opencv
    sudo apt-get install ros-humble-cv-bridge
    ```

4. Build the package:
    ```
    cd ~/ros2_ws
    colcon build --packages-select vps_slam
    source install/setup.bash
    ```

## Usage

1. Set up your Google Street View API key:
   - Obtain an API key from Google Cloud Console
   - Replace the placeholder API key in `src/match_streetview.cpp`

2. Launch the VPS SLAM node:
    ```
    ros2 run vps_slam vps_slam_node
    ```

3. Required ROS2 topics:
   - `/zedm/zed_node/left/image_rect_color` (sensor_msgs/Image)
   - `depth` (sensor_msgs/Image)
   - `gps/fix` (sensor_msgs/NavSatFix)
   - `camera_info` (sensor_msgs/CameraInfo)


## How It Works

1. System receives real-time camera images and GPS coordinates
2. Based on GPS location, queries Google Street View metadata for nearest panorama
3. Retrieves Street View image and performs ORB feature matching
4. Computes homography matrix between matched images
5. Updates pose estimation using both GPS and visual data

### Implemented
- GPS-based Google Street View image retrieval with metadata
- Feature detection and matching using ORB features
- Homography computation between matched images
- Basic pose estimation from homography decomposition
- ROS2 integration with standard message types

### Pending
- Real-time image processing from ZED stereo camera
- Depth image integration
- Full visual odometry implementation
- Camera calibration integration
- Full SLAM implementation

## License

This project is licensed under the GNU GPLv3 License - see the [LICENSE](LICENSE) file for details.