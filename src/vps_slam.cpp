#include <iostream>
#include <opencv2/opencv.hpp>
#include <curl/curl.h>
#include "vps_slam/vps_slam.hpp"
#include "vps_slam/pose_estimator.hpp"


namespace vps_slam
{
  VPSSLAM::VPSSLAM(const rclcpp::NodeOptions& options): Node("vps_slam", options)
  {
    image_sub = this->create_subscription<sensor_msgs::msg::Image>("/zedm/zed_node/left/image_rect_color", 10, std::bind(&VPSSLAM::imageCallback, this, std::placeholders::_1));
    depth_sub = this->create_subscription<sensor_msgs::msg::Image>("depth", 10, std::bind(&VPSSLAM::depthCallback, this, std::placeholders::_1));
    navsafix_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>("gps/fix", 10, std::bind(&VPSSLAM::navsafixCallback, this, std::placeholders::_1));
    camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>("camera_info", 10, std::bind(&VPSSLAM::cameraInfoCallback, this, std::placeholders::_1));
    match_google_streetview = MatchGoogleStreetView();
    pose_estimator_ = std::make_unique<PoseEstimator>();
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovariance>(
        "estimated_pose", 10);
    matches_pub_ = this->create_publisher<sensor_msgs::msg::Image>("debug/matches", 10);
    streetview_pub_ = this->create_publisher<sensor_msgs::msg::Image>("debug/streetview", 10);

    // Initialize K_ matrix (you should get these values from your camera calibration)
    K_ = (cv::Mat_<double>(3,3) << 
        525.0, 0.0, 319.5,
        0.0, 525.0, 239.5,
        0.0, 0.0, 1.0);
  }

  void VPSSLAM::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  { 
    RCLCPP_INFO(this->get_logger(), "Received image");
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    image_cam = cv_ptr->image;
    is_image_received = true;
  }

  void VPSSLAM::depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    (void)msg;
    // Do something with the depth image
  }

  void VPSSLAM::navsafixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    match_google_streetview.SetGPSCoordinates(msg->latitude, msg->longitude);
    has_new_gps_ = true;
    pose_estimator_->updateFromGPS(*msg);
    auto current_pose = pose_estimator_->getCurrentPose();
    pose_pub_->publish(current_pose);
  }

  void VPSSLAM::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    (void)msg;
    // Do something with the camera info
  }

  void VPSSLAM::processVisualData(const cv::Mat& H)
  {
    std::vector<cv::Mat> rotations, translations, normals;
    cv::decomposeHomographyMat(H, K_, rotations, translations, normals);
    
    // Use the first decomposition (in a real implementation, you'd want to select the best one)
    pose_estimator_->updateFromVisual(rotations[0], translations[0]);
    auto current_pose = pose_estimator_->getCurrentPose();
    pose_pub_->publish(current_pose);
  }

    
}

int main() {
   rclcpp::init(0, nullptr);
    rclcpp::spin(std::make_shared<vps_slam::VPSSLAM>());
    rclcpp::shutdown();


    return 0;
}
