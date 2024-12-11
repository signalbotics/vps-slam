#include <rclcpp/rclcpp.hpp>
#include "vps_slam/match_streetview.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include "vps_slam/pose_estimator.hpp"
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
      std::unique_ptr<PoseEstimator> pose_estimator_;
      rclcpp::Publisher<geometry_msgs::msg::PoseWithCovariance>::SharedPtr pose_pub_;
      cv::Mat K_;

      void processVisualData(const cv::Mat& H);

      // Add new publishers for visualization
      rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr matches_pub_;
      rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr streetview_pub_;
      
      // Add flag for tracking GPS updates
      bool has_new_gps_ = false;
  };
}  // namespace vps_slam