#ifndef POSE_ESTIMATOR_HPP
#define POSE_ESTIMATOR_HPP

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <opencv2/core/mat.hpp>
#include <memory>

namespace vps_slam {

class PoseEstimator {
public:
    PoseEstimator();
    
    // Update state with GPS measurement
    void updateFromGPS(const sensor_msgs::msg::NavSatFix& gps_msg);
    
    // Update state with visual measurement (from homography)
    void updateFromVisual(const cv::Mat& R, const cv::Mat& t);
    
    // Get current pose estimate
    geometry_msgs::msg::PoseWithCovariance getCurrentPose() const;
    
    // Get current state covariance
    Eigen::MatrixXd getCovariance() const;

private:
    // State vector: [x, y, z, vx, vy, vz, roll, pitch, yaw]
    Eigen::VectorXd state_;
    Eigen::MatrixXd state_covariance_;
    Eigen::MatrixXd process_noise_;
    
    // Last update timestamp
    double last_update_time_;
    
    // System matrices
    Eigen::MatrixXd F_; // State transition matrix
    Eigen::MatrixXd H_gps_; // GPS measurement matrix
    Eigen::MatrixXd H_visual_; // Visual measurement matrix
    Eigen::MatrixXd R_gps_; // GPS measurement noise
    Eigen::MatrixXd R_visual_; // Visual measurement noise
    
    void initializeMatrices();
    void predictStep(double dt);
    void updateStep(const Eigen::VectorXd& measurement, 
                   const Eigen::MatrixXd& H,
                   const Eigen::MatrixXd& R);
    
    // Convert GPS to local coordinates
    Eigen::Vector3d gpsToLocal(const sensor_msgs::msg::NavSatFix& gps_msg) const;
    
    // Convert rotation matrix and translation vector to state vector format
    Eigen::VectorXd visualMeasurementToState(const cv::Mat& R, const cv::Mat& t) const;
};

} // namespace vps_slam

#endif // POSE_ESTIMATOR_HPP 