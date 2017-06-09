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
    Feature(Frame* frame, int level,
            const Eigen::Vector2d& pixel,
            double score
    ) : _frame(frame), _level(level), _pixel(pixel), _score(score) {}

    Frame* _frame = nullptr;
    int _level = -1;
    double _score = 0.0;
    Eigen::Vector2d _pixel = Eigen::Vector2d(0, 0);

    double _depth = -1.0;
    MapPoint* _mappoint = nullptr;
};

#endif //DIRECTSPARSEVO_FEATURE_H
