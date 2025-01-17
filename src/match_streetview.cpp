#include <Eigen/Dense>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <curl/curl.h>
#include <nlohmann/json.hpp>
#include <cmath>
#include <cstdlib> // For setenv
#include <fstream> 
using json = nlohmann::json;

#include "vps_slam/match_streetview.hpp"

// Trim function to remove leading/trailing whitespace
static inline std::string trim(const std::string &s) {
    auto start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return "";
    auto end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end - start + 1);
}

// Function to load .env file and set environment variables
void loadEnvFile(const std::string &filename) {
    std::ifstream envFile(filename);
    if (!envFile.is_open()) {
        std::cerr << "Could not open file: " << filename << std::endl;
        return;
    }

    std::string line;
    while (std::getline(envFile, line)) {
        line = trim(line);

        // Skip empty lines or lines starting with '#'
        if (line.empty() || line[0] == '#') {
            continue;
        }

        // Find '='
        size_t pos = line.find('=');
        if (pos == std::string::npos) {
            // Invalid format; skip
            continue;
        }

        // Extract key and value
        std::string key = trim(line.substr(0, pos));
        std::string value = trim(line.substr(pos + 1));

        // Set environment variable (overwrite = 1)
        setenv(key.c_str(), value.c_str(), 1);
    }
}

double tot_start_time;

MatchGoogleStreetView::MatchGoogleStreetView() 
    : gps_lat(0.0)
    , gps_long(0.0)
    , has_streetview_image_(false) 
{
    // Load variables from .env
    loadEnvFile(".env");

    // Test retrieving one of the variables
    const char* val = std::getenv("API_KEY");
    if (val) {
        std::cout << "GOOGLE API KEY LOADED"<< std::endl;
        apiKey = val;
    } else {
        std::cout << "API KEY not found. Please make sure yu have .env file with API_KEY for google streat view in the current directory" << std::endl;
    }
}

void MatchGoogleStreetView::SetGPSCoordinates(double lat, double lon) {
    gps_lat = lat;
    gps_long = lon;
    has_streetview_image_ = false;  // Reset flag when GPS changes
}

size_t MatchGoogleStreetView::WriteCallback(void* contents, size_t size, size_t nmemb, void* userp) {
    size_t realsize = size * nmemb;
    std::vector<unsigned char>* mem = (std::vector<unsigned char>*)userp;
    mem->insert(mem->end(), (unsigned char*)contents, (unsigned char*)contents + realsize);
    return realsize;
}

size_t MatchGoogleStreetView::MetadataCallback(void* contents, size_t size, size_t nmemb, std::string* userp) {
    userp->append((char*)contents, size * nmemb);
    return size * nmemb;
}

MatchGoogleStreetView::StreetViewMetadata MatchGoogleStreetView::QueryMetadata() {
    StreetViewMetadata metadata;
    metadata.available = false;

    std::string serverUrl = "https://maps.googleapis.com/maps/api/streetview/metadata";
    
    std::string fullUrl = serverUrl + "?location=" + 
                         std::to_string(gps_lat) + "," + 
                         std::to_string(gps_long) + 
                         "&key=" + apiKey;

    CURL *curl;
    CURLcode res;
    std::string readBuffer;
    
    curl = curl_easy_init();
    if(curl) {
        curl_easy_setopt(curl, CURLOPT_URL, fullUrl.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, MetadataCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
        res = curl_easy_perform(curl);
        curl_easy_cleanup(curl);

        if(res == CURLE_OK) {
            ParseMetadataJson(readBuffer, metadata);
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("vps_slam"), 
                        "Failed to get metadata: %s", curl_easy_strerror(res));
        }
    }

    return metadata;
}

bool MatchGoogleStreetView::ParseMetadataJson(const std::string& json_str, 
                                            StreetViewMetadata& metadata) {
    try {
        json j = json::parse(json_str);
        
        if (j["status"] == "OK") {
            metadata.available = true;
            metadata.latitude = j["location"]["lat"].get<double>();
            metadata.longitude = j["location"]["lng"].get<double>();
            metadata.pano_id = j["pano_id"].get<std::string>();
            
            if (j.contains("heading")) {
                metadata.heading = j["heading"].get<double>();
            } else {
                double dx = metadata.longitude - gps_long;
                double dy = metadata.latitude - gps_lat;
                metadata.heading = std::atan2(dx, dy) * 180.0 / M_PI;
            }
            
            return true;
        } else {
            RCLCPP_WARN(rclcpp::get_logger("vps_slam"), 
                       "Parsing metadata JSON failed: %s", j["status"].get<std::string>());
        }

    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("vps_slam"), 
                    "Error parsing metadata JSON: %s", e.what());
    }
    return false;
}

cv::Mat MatchGoogleStreetView::QueryStreetViewImage(const StreetViewMetadata& metadata) {
    if (!metadata.available) {
        return cv::Mat();
    }

    std::string serverUrl = "https://maps.googleapis.com/maps/api/streetview";
    
    std::string fullUrl = serverUrl + "?size=640x480" +
                         "&location=" + std::to_string(metadata.latitude) + 
                         "," + std::to_string(metadata.longitude) +
                         "&heading=" + std::to_string(metadata.heading) +
                         "&fov=90" +
                         "&pitch=0" +
                         "&key=" + apiKey;

    if (!metadata.pano_id.empty()) {
        fullUrl += "&pano=" + metadata.pano_id;
    }

    CURL *curl;
    CURLcode res;
    std::vector<unsigned char> buffer;
    
    curl = curl_easy_init();
    if(curl) {
        curl_easy_setopt(curl, CURLOPT_URL, fullUrl.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &buffer);
        res = curl_easy_perform(curl);
        curl_easy_cleanup(curl);

        if(res == CURLE_OK && !buffer.empty()) {
            return cv::imdecode(cv::Mat(buffer), cv::IMREAD_COLOR);
        }
    }
    
    return cv::Mat();
}

std::pair<cv::Mat, cv::Mat> MatchGoogleStreetView::GetMatchingPoints(const cv::Mat& img1, const cv::Mat& img2) {
    // Ensure images are not empty
    if (img1.empty() || img2.empty()) {
        std::cerr << "One of the images is empty." << std::endl;
        throw std::runtime_error("Image is empty.");
    }
    // resize img2 to img1 size
    cv::Mat resized_img2;
    cv::resize(img2, resized_img2, cv::Size(img1.cols, img1.rows));
    // Convert images to grayscale if they are not already
    cv::Mat img1_gray, img2_gray;
    if (img1.channels() == 3) {
        cv::cvtColor(img1, img1_gray, cv::COLOR_BGR2GRAY);
    } else {
        img1_gray = img1.clone();
    }

    if (resized_img2.channels() == 3) {
        cv::cvtColor(resized_img2, img2_gray, cv::COLOR_BGR2GRAY);
    } else {
        img2_gray = resized_img2.clone();
    }
    cv::Ptr<cv::Feature2D> detector = cv::ORB::create();

    std::vector<cv::KeyPoint> keypoints1;
    cv::Mat descriptors1;
    detector->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
    cv::Mat frame_with_keypoints = img1.clone();
    cv::drawKeypoints(img1, keypoints1, frame_with_keypoints, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);


    std::vector<cv::KeyPoint> keypoints2;
    cv::Mat descriptors2;
    detector->detectAndCompute(resized_img2, cv::noArray(), keypoints2, descriptors2);

    frame_with_keypoints = resized_img2.clone();
    cv::drawKeypoints(resized_img2, keypoints2, frame_with_keypoints, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    
    cv::BFMatcher matcher;
    std::vector<std::vector<cv::DMatch>> matches;
    matcher.knnMatch(descriptors1, descriptors2, matches, 2);

    std::vector<cv::DMatch> good;
    for (size_t i = 0; i < matches.size(); i++) {
        if (matches[i][0].distance < 0.75 * matches[i][1].distance) {
            good.push_back(matches[i][0]);
        }
    }

    cv::Mat img_matches;
    cv::drawMatches(img1, keypoints1, resized_img2, keypoints2, good, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    cv::Mat src_pts(good.size(), 1, CV_32FC2);
    cv::Mat dst_pts(good.size(), 1, CV_32FC2);
    for (size_t i = 0; i < good.size(); i++) {
        src_pts.at<cv::Point2f>(i, 0) = keypoints1[good[i].queryIdx].pt;
        dst_pts.at<cv::Point2f>(i, 0) = keypoints2[good[i].trainIdx].pt;
    }

    cv::Mat H;
    // if correspondences are atleast 4, find homography matrix
    if (good.size() >= 4) {
        H = cv::findHomography(src_pts, dst_pts, cv::RANSAC, 5.0);
        std::cout << "Homography matrix: \n" << H << std::endl;
    }
    
    return {img_matches, H};
}

// for testing directly 
int MatchGoogleStreetView::retrieve(double gps_lat, double gps_long, double roi_radius, cv::Mat& image_cam) {
    tot_start_time = cv::getTickCount();

    double start_time = cv::getTickCount();
    auto [streetview_img, metadata] = GetStreetView(gps_lat, gps_long, roi_radius);
    double end_time = cv::getTickCount();
    double elapsed_time = (end_time - start_time) / cv::getTickFrequency();
    std::cout << "Time to get image: " << elapsed_time << " seconds" << std::endl;

    cv::resize(image_cam, image_cam, cv::Size(640, 480));
    auto [img_matches, H] = GetMatchingPoints(streetview_img, image_cam);
    double tot_end_time = cv::getTickCount();
    double tot_elapsed_time = (tot_end_time - tot_start_time) / cv::getTickFrequency();
    std::cout << "Total time: " << tot_elapsed_time << " seconds" << std::endl;
    
    cv::imshow("Matches", img_matches);
    cv::waitKey(0);

    return 0;
}

cv::Mat MatchGoogleStreetView::GetHomography(const cv::Mat& current_image) {
    // First, query metadata to get exact location
    StreetViewMetadata metadata = QueryMetadata();
    
    if (!metadata.available) {
        RCLCPP_WARN(rclcpp::get_logger("vps_slam"), 
                   "No Street View image available at current location");
        return cv::Mat();
    }

    // Get Street View image using metadata
    last_streetview_image_ = QueryStreetViewImage(metadata);
    if (last_streetview_image_.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("vps_slam"), 
                    "Failed to get Street View image");
        return cv::Mat();
    }

    // Store metadata for pose estimation
    last_metadata_ = metadata;

    // resize current image to 640x480
    cv::resize(current_image, current_image, cv::Size(640, 480));
    
    // Get matching points and compute homography
    auto [img_matches, H] = GetMatchingPoints(current_image, last_streetview_image_);
    return H;
}

std::pair<cv::Mat, MatchGoogleStreetView::StreetViewMetadata> MatchGoogleStreetView::GetStreetView(double lat, double lon, double radius) {
    // Store coordinates for later use
    gps_lat = lat;
    gps_long = lon;

    // First get metadata to find exact location
    StreetViewMetadata metadata = QueryMetadata();
    
    if (!metadata.available) {
        RCLCPP_WARN(rclcpp::get_logger("vps_slam"),
                   "No Street View image available at location: %f, %f", lat, lon);
    }

    double start_time = cv::getTickCount();
    // Get image using metadata
    cv::Mat streetview_img = QueryStreetViewImage(metadata);
    double end_time = cv::getTickCount();
    double elapsed_time = (end_time - start_time) / cv::getTickFrequency();
    std::cout << "Time to get image: " << elapsed_time << " seconds" << std::endl;
    if (streetview_img.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("vps_slam"),
                    "Failed to get Street View image");
        return {cv::Mat(), metadata};
    }
    RCLCPP_INFO(rclcpp::get_logger("vps_slam"),
               "Got Street View image at location: %f, %f", lat, lon);
    last_streetview_image_ = streetview_img;
    last_metadata_ = metadata;
    has_streetview_image_ = true;

    return {streetview_img, metadata};
}

