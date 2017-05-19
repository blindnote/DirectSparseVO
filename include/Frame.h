//
// Created by rochelle on 17-5-17.
//

#ifndef DIRECTSPARSEVO_FRAME_H
#define DIRECTSPARSEVO_FRAME_H

#include "Common.h"
#include "Feature.h"

struct Frame
{
    struct Setting
    {
        int _pyramid_level = 3;
//      int _detection_threshold = 50;
    } static _setting;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Frame() {}
    Frame(const Frame& frame) = delete;
    Frame operator = (const Frame& f2) = delete;
    ~Frame() {}

    void InitFrame(const cv::Mat& raw, bool detect = false);
    void CreateImagePyramid();

    Eigen::Vector3d GetCamCenter() const
    {
        return _TCW.inverse().translation();
    }

    unsigned long _id;

    cv::Mat _color;
    //cv::Mat _gray;
    std::vector<cv::Mat> _pyramid;

    std::vector<Feature*> _features;

    Sophus::SE3 _TCW = Sophus::SE3();
};

#endif //DIRECTSPARSEVO_FRAME_H
