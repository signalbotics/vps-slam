#include <rclcpp/rclcpp.hpp>
#include "vps_slam/match_streetview.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.hpp>
namespace vps_slam
{
  class VPSSLAM : public rclcpp::Node
  {
    public:
      VPSSLAM(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    
    private:
      void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
      void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);
      void navsafixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
      void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
      rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub, depth_sub; 
      // navsafix sub
      rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsafix_sub;
      // camera info sub
      rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub;
      cv::Mat image_cam;
      bool is_image_received = false, wait = true;

      MatchGoogleStreetView match_google_streetview;

  };
}  // namespace vps_slam