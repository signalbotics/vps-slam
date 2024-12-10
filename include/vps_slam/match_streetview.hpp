#ifndef MATCH_STREETVIEW_HPP
#define MATCH_STREETVIEW_HPP

#include <iostream>
#include <opencv2/opencv.hpp>
#include <curl/curl.h>

class MatchGoogleStreetView {
private:
    std::string SERVER_URL;
    double gps_lat;
    double gps_long;
    double roi_radius;
    std::map<std::string, std::string> reqdict;
public:
    MatchGoogleStreetView();
    int retrieve(double gps_lat, double gps_long, double roi_radius, cv::Mat& image_cam);

    static size_t WriteMemoryCallback(void *contents, size_t size, size_t nmemb, void *userp);
    cv::Mat QueryToServer();
    void SetParams(double gps_lat, double gps_long, double roi_radius);
    cv::Mat GetStreetView(double gps_lat, double gps_long, double roi_radius);
    cv::Mat GetMatchingPoints(const cv::Mat& img1, const cv::Mat& img2);
};
#endif // MATCH_STREETVIEW_HPP