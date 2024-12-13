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
    git clone https://github.com/brukg/vps-slam.git
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

## Technical Details

### Pose from Homography

The homography matrix H relates points between the Street View image and current camera image:
x₂ = Hx₁

For calibrated cameras, H can be decomposed into rotation (R) and translation (t):
H = K(R + t·nᵀ/d)K⁻¹

where:
- K is the camera intrinsic matrix
- R is the rotation matrix
- t is the translation vector
- n is the normal vector of the plane
- d is the distance to the plane

### Pose Update Process

1. GPS Update:
   - Position: p_gps = [lat, lon, alt]
   - Covariance: Σ_gps

2. Visual Update from Homography:
   - Relative rotation: R_rel
   - Relative translation: t_rel (up to scale)
   - Street View position: p_sv = [lat_sv, lon_sv, alt_sv]

3. Pose Fusion:
   - Absolute rotation: R = R_rel * R_sv
   - Scale-corrected translation: t = s * t_rel 
     where s is estimated from GPS distance
   - Final position: p = p_sv + R * t

[PENDING IMPLEMENTATION]

4. Uncertainty Estimation:
   - Visual uncertainty: Σ_vis from feature matching residuals
   - Fused covariance: Σ = (Σ_gps⁻¹ + Σ_vis⁻¹)⁻¹

5. State Update:
   - Extended Kalman Filter prediction/update cycle
   - State vector: x = [position, orientation, velocity]

### Street View Metadata Usage

1. Location Refinement:
   - Input: Approximate GPS position (lat, lon)
   - Metadata provides:
     - Exact panorama location (lat_sv, lon_sv)
     - Panorama ID (pano_id) for consistent image retrieval
     - Heading information

2. Image Retrieval Optimization:
   - Using pano_id ensures we get the exact same panorama view
   - Heading is used to align the Street View image with our camera view
   - FOV (field of view) is set to 90° to match typical camera settings

3. Pose Graph Integration:
   [PENDING IMPLEMENTATION]
   - Store panorama IDs as unique nodes
   - Use metadata locations as absolute position constraints
   - Track visited panoramas to build a connected graph

## License

This project is licensed under the GNU GPLv3 License - see the [LICENSE](LICENSE) file for details.