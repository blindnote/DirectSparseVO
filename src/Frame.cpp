//
// Created by rochelle on 17-5-17.
//
#include "Frame.h"

void Frame::InitFrame(const cv::Mat& color, bool detect)
{
    _color = color.clone();
    cv::cvtColor(_color, _gray, cv::COLOR_BGR2GRAY);

    if (!detect) return;

    std::vector<cv::KeyPoint> keypoints_vec;
    cv::Ptr<cv::FastFeatureDetector> detectorPtr = cv::FastFeatureDetector::create();
    detectorPtr->detect(_gray, keypoints_vec);

//    cv::Mat keypoints_img;
//    cv::drawKeypoints(_gray, keypoints_vec, keypoints_img);
//    cv::imshow("1st frame", keypoints_img);
//    cv::waitKey(0);
//    cv::destroyWindow("1st frame");

    for(auto kp : keypoints_vec)
    {
        float kp_x = kp.pt.x, kp_y = kp.pt.y;
        // remove points near edges
        const int bound = 20;
        if (kp_x < bound || kp_y < bound || (kp_x + bound) > _gray.cols || (kp_y + bound) > _gray.rows)
        {
            continue;
        }

        Feature *feature = new Feature(Eigen::Vector2d(kp_x, kp_y), this);
        _features.push_back(feature);
    }
}