#include <iostream>
#include <opencv2/opencv.hpp>
#include <curl/curl.h>
#include "vps_slam/vps_slam.hpp"
#include "vps_slam/pose_estimator.hpp"


namespace vps_slam
{
  VPSSLAM::VPSSLAM(const rclcpp::NodeOptions& options): Node("vps_slam_node", options),
    has_new_gps_(false),
    is_image_received(false),
    K_(cv::Mat::eye(3, 3, CV_64F)) // Initialize as identity matrix
  {
    // Use reliable QoS for critical data
    auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    
    // Add debug logging for initialization
    RCLCPP_INFO(this->get_logger(), "Initializing VPS SLAM node");
    
    // Create subscriptions with explicit QoS
    image_sub = this->create_subscription<sensor_msgs::msg::Image>(
        "image", reliable_qos,
        std::bind(&VPSSLAM::imageCallback, this, std::placeholders::_1));
        
    depth_sub = this->create_subscription<sensor_msgs::msg::Image>(
        "depth", reliable_qos,
        std::bind(&VPSSLAM::depthCallback, this, std::placeholders::_1));
        
    navsafix_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "gps", reliable_qos,
        std::bind(&VPSSLAM::navsafixCallback, this, std::placeholders::_1));
        
    camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info", reliable_qos,
        std::bind(&VPSSLAM::cameraInfoCallback, this, std::placeholders::_1));
        
    RCLCPP_DEBUG(this->get_logger(), "Subscriptions created successfully");

    pose_estimator_ = std::make_unique<PoseEstimator>();
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovariance>(
        "estimated_pose", 10);
    matches_pub_ = this->create_publisher<sensor_msgs::msg::Image>("debug/matches", 10);
    streetview_pub_ = this->create_publisher<sensor_msgs::msg::Image>("debug/streetview", 10);
  }

  void VPSSLAM::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  { 
    RCLCPP_DEBUG(this->get_logger(), "Received image");
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    
    image_cam = cv_ptr->image;
    is_image_received = true;

    // Only process if we have GPS data
    if (has_new_gps_ && is_image_received) {
      try {
        // Get Street View image using accessor methods
        auto [streetview_img, metadata] = match_google_streetview.GetStreetView(
            match_google_streetview.GetLatitude(), 
            match_google_streetview.GetLongitude(),
            100.0); // 100m radius

        if (!streetview_img.empty()) {
          auto [matches_img, H]  = match_google_streetview.GetMatchingPoints(streetview_img, image_cam);

          if (!H.empty()) {
            // Process visual data and update pose with metadata
            processVisualData(H, metadata);

            // Publish debug images
            if (!matches_img.empty()) {
              auto matches_msg = cv_bridge::CvImage(msg->header, "bgr8", matches_img).toImageMsg();
              matches_pub_->publish(*matches_msg);
            }
          }
        }
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error processing Street View matching: %s", e.what());
      }
      
      has_new_gps_ = false;
    }
  }

  void VPSSLAM::processVisualData(const cv::Mat& H, const MatchGoogleStreetView::StreetViewMetadata& metadata)
  {
    std::vector<cv::Mat> rotations, translations, normals;
    cv::decomposeHomographyMat(H, K_, rotations, translations, normals);
    
    if (!rotations.empty() && !translations.empty()) {
      // Convert GPS offsets to Cartesian coordinates
      const double earth_radius = 6378137.0; // WGS84 equatorial radius in meters
      double lat1 = match_google_streetview.GetLatitude() * M_PI/180.0;
      double lon1 = match_google_streetview.GetLongitude() * M_PI/180.0;
      double lat2 = metadata.latitude * M_PI/180.0;
      double lon2 = metadata.longitude * M_PI/180.0;

      // Haversine formula for Cartesian offsets
      double dlat = lat2 - lat1;
      double dlon = lon2 - lon1;
      double a = sin(dlat/2) * sin(dlat/2) +
                cos(lat1) * cos(lat2) *
                sin(dlon/2) * sin(dlon/2);
      double c = 2 * atan2(sqrt(a), sqrt(1-a));
      double distance = earth_radius * c;

      // Calculate bearing
      double y = sin(dlon) * cos(lat2);
      double x = cos(lat1) * sin(lat2) -
                sin(lat1) * cos(lat2) * cos(dlon);
      double bearing = atan2(y, x);

      // Convert to Cartesian offsets
      double x_offset = distance * cos(bearing);
      double y_offset = distance * sin(bearing);

      // Adjust translation based on Cartesian offsets
      translations[0].at<double>(0) += x_offset;
      translations[0].at<double>(1) += y_offset;
      RCLCPP_INFO(this->get_logger(), "Adjusted translation: %f, %f", translations[0].at<double>(0), translations[0].at<double>(1));
      // Use the first decomposition (in a real implementation, you'd want to select the best one)
      pose_estimator_->updateFromVisual(rotations[0], translations[0]);
      auto current_pose = pose_estimator_->getCurrentPose();
      pose_pub_->publish(current_pose);
    }
  }

  void VPSSLAM::depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    (void)msg;
    // Do something with the depth image
  }

  void VPSSLAM::navsafixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    RCLCPP_DEBUG(this->get_logger(), "Received GPS fix: %f, %f", msg->latitude, msg->longitude);
    match_google_streetview.SetGPSCoordinates(msg->latitude, msg->longitude);
    has_new_gps_ = true;
    pose_estimator_->updateFromGPS(*msg);
    auto current_pose = pose_estimator_->getCurrentPose();
    pose_pub_->publish(current_pose);
  }

  void VPSSLAM::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    // Convert camera matrix to OpenCV Mat
    K_ = (cv::Mat_<double>(3,3) << 
        msg->k[0], msg->k[1], msg->k[2],
        msg->k[3], msg->k[4], msg->k[5],
        msg->k[6], msg->k[7], msg->k[8]);
    
    RCLCPP_INFO(this->get_logger(), "Updated camera matrix from camera info");
  
    // close subscriber
    camera_info_sub.reset();

  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions();
  rclcpp::spin(std::make_shared<vps_slam::VPSSLAM>(options));
  rclcpp::shutdown();

  return 0;
}
