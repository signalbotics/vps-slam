#include <iostream>
#include <opencv2/opencv.hpp>
#include <curl/curl.h>
#include "vps_slam/vps_slam.hpp"


namespace vps_slam
{
  VPSSLAM::VPSSLAM(const rclcpp::NodeOptions& options): Node("vps_slam", options)
  {

    image_sub = this->create_subscription<sensor_msgs::msg::Image>("/zedm/zed_node/left/image_rect_color", 10, std::bind(&VPSSLAM::imageCallback, this, std::placeholders::_1));
    depth_sub = this->create_subscription<sensor_msgs::msg::Image>("depth", 10, std::bind(&VPSSLAM::depthCallback, this, std::placeholders::_1));
    navsafix_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>("gps/fix", 10, std::bind(&VPSSLAM::navsafixCallback, this, std::placeholders::_1));
    camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>("camera_info", 10, std::bind(&VPSSLAM::cameraInfoCallback, this, std::placeholders::_1));
    match_google_streetview = MatchGoogleStreetView();
    double gps_lat = 41.3935598;
    double gps_long = 2.1920437;
    double roi_radius = 250.0;
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
    double gps_lat = msg->latitude;
    double gps_long = msg->longitude;
    double roi_radius = 250.0;
    RCLCPP_INFO(this->get_logger(), "Received GPS fix: %f, %f", gps_lat, gps_long);
    if (is_image_received && wait)
      wait =match_google_streetview.retrieve(gps_lat, gps_long, roi_radius, image_cam);

  }

  void VPSSLAM::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    (void)msg;
    // Do something with the camera info
  }

    
}

int main() {
   rclcpp::init(0, nullptr);
    rclcpp::spin(std::make_shared<vps_slam::VPSSLAM>());
    rclcpp::shutdown();


    return 0;
}
