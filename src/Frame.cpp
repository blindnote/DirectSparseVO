//
// Created by rochelle on 17-5-17.
//
#include <fast/fast.h>
#include "Frame.h"
#include "Config.h"

Frame::Setting Frame::_setting;

void Frame::InitFrame(const cv::Mat& color, bool detect)
{
    _setting._pyramid_level = Config::Get<double> ( "frame.pyramid" );
    //_setting._detection_threshold = Config::Get<double> ( "feature.detection_threshold" );

    _pyramid.resize( _setting._pyramid_level );

    _color = color.clone();
    cv::cvtColor(_color, _pyramid[0], cv::COLOR_BGR2GRAY);
    CreateImagePyramid();

    if (!detect) return;

    std::vector<cv::KeyPoint> keypoints_vec;
    cv::Ptr<cv::FastFeatureDetector> detectorPtr = cv::FastFeatureDetector::create();

    for(int L = 0; L < _setting._pyramid_level; L++)
    {
        keypoints_vec.clear();

        detectorPtr->detect(_pyramid[L], keypoints_vec);

        const int scale = ( 1<<L );
//        std::vector<fast::fast_xy> fast_corners;
//
//        fast::fast_corner_detect_10_sse2 (
//                ( fast::fast_byte* ) _pyramid[L].data, _pyramid[L].cols,
//                _pyramid[L].rows, _pyramid[L].cols, _setting._detection_threshold, fast_corners );

        for(auto kp : keypoints_vec)
        {
            float kp_x = kp.pt.x, kp_y = kp.pt.y;
            // remove points near edges
            const int bound = 20;
            if (kp_x < bound || kp_y < bound || (kp_x + bound) > _pyramid[0].cols || (kp_y + bound) > _pyramid[0].rows)
            {
                continue;
            }

            Feature *feature = new Feature(this, L, Eigen::Vector2d( kp_x*scale, kp_y*scale));
            _features.push_back(feature);
        }
    }

//    cv::Mat keypoints_img;
//    cv::drawKeypoints(_pyramid[0], keypoints_vec, keypoints_img);
//    cv::imshow("1st frame", keypoints_img);
//    cv::waitKey(0);
//    cv::destroyWindow("1st frame");
}

void Frame::CreateImagePyramid()
{
    for ( size_t i = 1; i<_pyramid.size(); i++ )
    {
        // 请记得第0层是原始分辨率
        cv::pyrDown(_pyramid[i-1], _pyramid[i]);
    }
}