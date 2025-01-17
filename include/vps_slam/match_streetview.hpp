#ifndef MATCH_STREETVIEW_HPP
#define MATCH_STREETVIEW_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <rclcpp/rclcpp.hpp>

class MatchGoogleStreetView {
public:
    struct StreetViewMetadata {
        double latitude;
        double longitude;
        double heading;
        bool available;
        std::string pano_id;
    };

    MatchGoogleStreetView();
    
    void SetGPSCoordinates(double lat, double lon);
    double GetLatitude() const { return gps_lat; }
    double GetLongitude() const { return gps_long; }
    StreetViewMetadata QueryMetadata();
    cv::Mat QueryStreetViewImage(const StreetViewMetadata& metadata);
    cv::Mat GetHomography(const cv::Mat& current_image);
    std::pair<cv::Mat, cv::Mat>  GetMatchingPoints(const cv::Mat& img1, const cv::Mat& img2);
    std::pair<cv::Mat, StreetViewMetadata> GetStreetView(double lat, double lon, double radius);
    
    // Make this public for visualization
    cv::Mat last_streetview_image_;

private:
    double gps_lat;
    double gps_long;
    StreetViewMetadata last_metadata_;
    bool has_streetview_image_;
    std::string apiKey; 

    static size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp);
    static size_t MetadataCallback(void* contents, size_t size, size_t nmemb, std::string* userp);
    bool ParseMetadataJson(const std::string& json, StreetViewMetadata& metadata);
    int retrieve(double gps_lat, double gps_long, double roi_radius, cv::Mat& image_cam);
};

#endif // MATCH_STREETVIEW_HPP