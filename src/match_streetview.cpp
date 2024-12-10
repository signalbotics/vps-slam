#include <iostream>
#include <opencv2/opencv.hpp>
#include <curl/curl.h>

#include "vps_slam/match_streetview.hpp"

double tot_start_time;
/**
 * @brief The MatchGoogleStreetView class represents a client for querying Google Street View images.
 * 
 * This class provides methods to query the Google Street View API and retrieve images based on GPS coordinates.
 * It also allows setting parameters such as latitude, longitude, and radius for the query.
 */
MatchGoogleStreetView::MatchGoogleStreetView() {
        // SERVER_URL = "https://" + ipaddr;
        gps_lat = 37.513366;
        gps_long = 127.056132;
        roi_radius = 100;
    }
    
    // Callback function writes data to a std::vector<unsigned char>
    size_t MatchGoogleStreetView::WriteMemoryCallback(void *contents, size_t size, size_t nmemb, void *userp) {
        size_t real_size = size * nmemb;
        auto *mem = static_cast<std::vector<unsigned char> *>(userp);
        const auto *start = static_cast<unsigned char *>(contents);
        mem->insert(mem->end(), start, start + real_size);
        return real_size;
    }


    cv::Mat MatchGoogleStreetView::QueryToServer() {

        std::string serverUrl = "https://maps.googleapis.com/maps/api/"; // Make sure this is correct
        std::string apiKey = ""; // Your actual API key
        // Ensure SERVER_URL ends with / or start the path with / in the next line if it doesn't
        //center=40.714728,-73.998672&zoom=10&size=400x400&maptype=satellite&key=

        // maps/api/streetview/metadata?size=640x480&location=41.393242,2.191709&fov=73&key=
        
        std::string viewtype = "streetview"; //staticmap, 
        std::string fullUrl = serverUrl + "streetview?size=640x480&location=" +
                            std::to_string(gps_lat) + "," + std::to_string(gps_long) + "&key=" + apiKey + "&&heading=200&fov=73";

        CURL *curl;
        CURLcode res;
        curl = curl_easy_init();
        std::vector<unsigned char> buffer;

        if(curl) {
            curl_easy_setopt(curl, CURLOPT_URL, fullUrl.c_str());
            std::cout << "URL: " << fullUrl << std::endl;
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteMemoryCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, (void *)&buffer);
            res = curl_easy_perform(curl);
            curl_easy_cleanup(curl);

            if(res != CURLE_OK) {
                std::cerr << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
                return cv::Mat();
            }
        }

        if(buffer.empty()) {
            std::cerr << "No data received from server." << std::endl;
            return cv::Mat();
        }
        return cv::imdecode(cv::Mat(buffer, true), cv::IMREAD_COLOR);

    }

    void MatchGoogleStreetView::SetParams(double gps_lat, double gps_long, double roi_radius) {
        this->gps_lat = gps_lat;
        this->gps_long = gps_long;
        this->roi_radius = roi_radius;
    }



    // std::tuple<std::string, double, double, std::string, double, int> GetStreetViewInfo(int imgidx, const json& res) {
    //     int numImgs = res["features"].size();
    //     double imgLong = res["features"][imgidx]["properties"]["longitude"];
    //     double imgLat = res["features"][imgidx]["properties"]["latitude"];
    //     std::string imgDate = res["features"][imgidx]["properties"]["date"];
    //     double imgHeading = res["features"][imgidx]["properties"]["heading"];
    //     std::string imgID = res["features"][imgidx]["properties"]["id"];
    //     return std::make_tuple(imgID, imgLat, imgLong, imgDate, imgHeading, numImgs);
    // }
// };

cv::Mat MatchGoogleStreetView::GetStreetView(double gps_lat, double gps_long, double roi_radius) {
    MatchGoogleStreetView isv;
    SetParams(gps_lat, gps_long, roi_radius);
    // isv.SetReqDict();
    return QueryToServer();
}

cv::Mat MatchGoogleStreetView::GetMatchingPoints(const cv::Mat& img1, const cv::Mat& img2) {
    // Ensure images are not empty
    if (img1.empty() || img2.empty()) {
        std::cerr << "One of the images is empty." << std::endl;
        throw std::runtime_error("Image is empty.");
    }

    // Convert images to grayscale if they are not already
    cv::Mat img1_gray, img2_gray;
    if (img1.channels() == 3) {
        cv::cvtColor(img1, img1_gray, cv::COLOR_BGR2GRAY);
    } else {
        img1_gray = img1.clone();
    }

    if (img2.channels() == 3) {
        cv::cvtColor(img2, img2_gray, cv::COLOR_BGR2GRAY);
    } else {
        img2_gray = img2.clone();
    }
    cv::Ptr<cv::Feature2D> detector = cv::ORB::create();

    double start_time = cv::getTickCount();

    std::vector<cv::KeyPoint> keypoints1;
    cv::Mat descriptors1;
    detector->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);

    double end_time = cv::getTickCount();
    double elapsed_time = (end_time - start_time) / cv::getTickFrequency();
    std::cout << "Time to create key points: " << elapsed_time << " seconds" << std::endl;

    cv::Mat frame_with_keypoints = img1.clone();
    cv::drawKeypoints(img1, keypoints1, frame_with_keypoints, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    // cv::imshow("Feature Method - SIFT 1", frame_with_keypoints);

    start_time = cv::getTickCount();

    std::vector<cv::KeyPoint> keypoints2;
    cv::Mat descriptors2;
    detector->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);

    end_time = cv::getTickCount();
    elapsed_time = (end_time - start_time) / cv::getTickFrequency();
    std::cout << "Time to create key points: " << elapsed_time << " seconds" << std::endl;

    frame_with_keypoints = img2.clone();
    cv::drawKeypoints(img2, keypoints2, frame_with_keypoints, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    // cv::imshow("Feature Method - SIFT 2", frame_with_keypoints);
    
    start_time = cv::getTickCount();
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
    cv::drawMatches(img1, keypoints1, img2, keypoints2, good, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    cv::Mat src_pts(good.size(), 1, CV_32FC2);
    cv::Mat dst_pts(good.size(), 1, CV_32FC2);
    for (size_t i = 0; i < good.size(); i++) {
        src_pts.at<cv::Point2f>(i, 0) = keypoints1[good[i].queryIdx].pt;
        dst_pts.at<cv::Point2f>(i, 0) = keypoints2[good[i].trainIdx].pt;
    }

    // if correspondences are atleast 4, find homography matrix
    if (good.size() >= 4) {
        cv::Mat H = cv::findHomography(src_pts, dst_pts, cv::RANSAC, 5.0);
        std::cout << "Homography matrix: \n" << H << std::endl;
    }
    
    // extract the rotation and translation in the camera frame in meter and radian units
    // std::vector<cv::Mat> rotations, translations, normals;
    // int solutions = cv::decomposeHomographyMat(H, K, rotations, translations, normals);

    // for(int i = 0; i < solutions; ++i) {
    //     std::cout << "Solution " << i << ":\n";
    //     std::cout << "Rotation:\n" << rotations[i] << "\n";
    //     std::cout << "Translation:\n" << translations[i] << "\n";
    //     std::cout << "Normal:\n" << normals[i] << "\n\n";
    // }
    end_time = cv::getTickCount();
    elapsed_time = (end_time - start_time) / cv::getTickFrequency();
    std::cout << "Time to match key points: " << elapsed_time << " seconds" << std::endl;
    return img_matches;
}

int MatchGoogleStreetView::retrieve(double gps_lat, double gps_long, double roi_radius, cv::Mat& image_cam) {
    tot_start_time = cv::getTickCount();

    double start_time = cv::getTickCount();
    cv::Mat img1 = GetStreetView(gps_lat, gps_long, roi_radius);
    double end_time = cv::getTickCount();
    double elapsed_time = (end_time - start_time) / cv::getTickFrequency();
    std::cout << "Time to get image: " << elapsed_time << " seconds" << std::endl;

    // gps_lat = 41.3935598;
    // gps_long = 2.19204;

    // std::cout << "Request 2" << std::endl;
    // start_time = cv::getTickCount();
    // cv::Mat img2 = GetStreetView(gps_lat, gps_long, roi_radius);
    // end_time = cv::getTickCount();
    // elapsed_time = (end_time - start_time) / cv::getTickFrequency();
    // std::cout << "Time to get image: " << elapsed_time << " seconds" << std::endl;
    cv::resize(image_cam, image_cam, cv::Size(640, 480));
    cv::Mat img_matches = GetMatchingPoints(img1, image_cam);
    double tot_end_time = cv::getTickCount();
    double tot_elapsed_time = (tot_end_time - tot_start_time) / cv::getTickFrequency();
    std::cout << "Total time: " << tot_elapsed_time << " seconds" << std::endl;
    
    cv::imshow("Matches", img_matches);
    cv::waitKey(0);

    return 0;
}
