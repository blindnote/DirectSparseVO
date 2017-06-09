//
// Created by rochelle on 17-5-17.
//

#ifndef DIRECTSPARSEVO_FRAME_H
#define DIRECTSPARSEVO_FRAME_H

#include "Common.h"
#include "Camera.h"
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

    void InitFrame(const cv::Mat& raw, PinholeCamera* pCamera, bool detect = false);
    void CreateImagePyramid();

    Eigen::Vector3d GetCamCenter() const
    {
        return _TCW.inverse().translation();
    }

    inline bool InFrame(const Eigen::Vector2d& pixel, int level = 0, int border = 0) const
    {
//        int scale = (1 << level);
//        return (pixel[0]/scale >= border && pixel[0]/scale < _pyramid[level].cols - border
//            &&  pixel[1]/scale >= border && pixel[1]/scale < _pyramid[level].rows - border);
        return pixel[0]/(1<<level) >= border && pixel[0]/(1<<level) < _color.cols - border
               && pixel[1]/(1<<level) >= border && pixel[1]/(1<<level) < _color.rows - border;
    }

    unsigned long _id;

    cv::Mat _color;
    std::vector<cv::Mat> _pyramid;

    PinholeCamera* _pCamera = nullptr;

    std::vector<Feature*> _features;

    Sophus::SE3 _TCW = Sophus::SE3();

    void CleanAllFeatures();
};

#endif //DIRECTSPARSEVO_FRAME_H
