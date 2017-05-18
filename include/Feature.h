//
// Created by rochelle on 17-5-17.
//

#ifndef DIRECTSPARSEVO_FEATURE_H
#define DIRECTSPARSEVO_FEATURE_H

#include "Common.h"

struct Frame;
struct MapPoint;

struct Feature
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Feature(const Eigen::Vector2d& pixel,
            Frame* frame
    ) : _pixel(pixel), _frame(frame) {}

    Eigen::Vector2d _pixel = Eigen::Vector2d(0, 0);
    double _depth = -1;
    Frame* _frame = nullptr;
    MapPoint* _mappoint = nullptr;
};

#endif //DIRECTSPARSEVO_FEATURE_H
