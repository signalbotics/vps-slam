#include "vps_slam/pose_estimator.hpp"
#include <cmath>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/core/eigen.hpp>

namespace vps_slam {

PoseEstimator::PoseEstimator() 
    : state_(Eigen::VectorXd::Zero(9)),
      state_covariance_(Eigen::MatrixXd::Identity(9, 9)),
      last_update_time_(0.0)
{
    initializeMatrices();
}

void PoseEstimator::initializeMatrices() {
    // Initialize state transition matrix (F)
    F_ = Eigen::MatrixXd::Identity(9, 9);
    
    // Initialize process noise (Q)
    process_noise_ = Eigen::MatrixXd::Identity(9, 9) * 0.1;
    
    // Initialize GPS measurement matrix (H_gps)
    H_gps_ = Eigen::MatrixXd::Zero(3, 9);
    H_gps_.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
    
    // Initialize visual measurement matrix (H_visual)
    H_visual_ = Eigen::MatrixXd::Identity(9, 9);
    
    // Initialize measurement noise matrices
    R_gps_ = Eigen::MatrixXd::Identity(3, 3) * 5.0; // 5m GPS noise
    R_visual_ = Eigen::MatrixXd::Identity(9, 9) * 0.1; // 0.1m/rad visual noise
}

void PoseEstimator::predictStep(double dt) {
    // Update state transition matrix with dt
    F_.block<3,3>(0,3) = Eigen::Matrix3d::Identity() * dt;
    
    // Predict state
    state_ = F_ * state_;
    
    // Predict covariance
    state_covariance_ = F_ * state_covariance_ * F_.transpose() + process_noise_ * dt;
}

void PoseEstimator::updateStep(const Eigen::VectorXd& measurement,
                             const Eigen::MatrixXd& H,
                             const Eigen::MatrixXd& R) {
    // Compute Kalman gain
    Eigen::MatrixXd K = state_covariance_ * H.transpose() * 
                        (H * state_covariance_ * H.transpose() + R).inverse();
    
    // Update state
    state_ = state_ + K * (measurement - H * state_);
    
    // Update covariance
    state_covariance_ = (Eigen::MatrixXd::Identity(9, 9) - K * H) * state_covariance_;
}

void PoseEstimator::updateFromGPS(const sensor_msgs::msg::NavSatFix& gps_msg) {
    double current_time = gps_msg.header.stamp.sec + gps_msg.header.stamp.nanosec * 1e-9;
    
    if (last_update_time_ > 0) {
        double dt = current_time - last_update_time_;
        predictStep(dt);
    }
    
    Eigen::Vector3d gps_measurement = gpsToLocal(gps_msg);
    Eigen::VectorXd measurement(3);
    measurement << gps_measurement;
    
    updateStep(measurement, H_gps_, R_gps_);
    last_update_time_ = current_time;
}

void PoseEstimator::updateFromVisual(const cv::Mat& R, const cv::Mat& t) {
    Eigen::VectorXd measurement = visualMeasurementToState(R, t);
    updateStep(measurement, H_visual_, R_visual_);
}

geometry_msgs::msg::PoseWithCovariance PoseEstimator::getCurrentPose() const {
    geometry_msgs::msg::PoseWithCovariance pose;
    
    // Position
    pose.pose.position.x = state_(0);
    pose.pose.position.y = state_(1);
    pose.pose.position.z = state_(2);
    
    // Orientation (convert euler angles to quaternion)
    Eigen::Quaterniond q = Eigen::AngleAxisd(state_(8), Eigen::Vector3d::UnitZ()) *
                          Eigen::AngleAxisd(state_(7), Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(state_(6), Eigen::Vector3d::UnitX());
                          
    pose.pose.orientation.w = q.w();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    
    // Covariance
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            pose.covariance[i*6 + j] = state_covariance_(i,j);
        }
    }
    
    return pose;
}

Eigen::MatrixXd PoseEstimator::getCovariance() const {
    return state_covariance_;
}

Eigen::Vector3d PoseEstimator::gpsToLocal(const sensor_msgs::msg::NavSatFix& gps_msg) const {
    // Convert GPS coordinates to local frame (simplified version)
    // In a real implementation, you would use proper GPS to local coordinate conversion
    static const double EARTH_RADIUS = 6371000.0; // meters
    static const double REF_LAT = gps_msg.latitude;
    static const double REF_LON = gps_msg.longitude;
    
    double delta_lat = gps_msg.latitude - REF_LAT;
    double delta_lon = gps_msg.longitude - REF_LON;
    
    Eigen::Vector3d local_pos;
    local_pos.x() = EARTH_RADIUS * cos(REF_LAT) * delta_lon;
    local_pos.y() = EARTH_RADIUS * delta_lat;
    local_pos.z() = gps_msg.altitude;
    
    return local_pos;
}

Eigen::VectorXd PoseEstimator::visualMeasurementToState(const cv::Mat& R, const cv::Mat& t) const {
    Eigen::VectorXd measurement(9);
    
    // Extract translation
    measurement.segment<3>(0) << t.at<double>(0), t.at<double>(1), t.at<double>(2);
    
    // Extract velocities (set to 0 as they're not directly measured)
    measurement.segment<3>(3).setZero();
    
    // Extract euler angles from rotation matrix
    Eigen::Matrix3d R_eigen;
    cv::cv2eigen(R, R_eigen);
    Eigen::Vector3d euler = R_eigen.eulerAngles(0, 1, 2);
    measurement.segment<3>(6) = euler;
    
    return measurement;
}

} // namespace vps_slam 