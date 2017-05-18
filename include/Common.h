//
// Created by rochelle on 17-5-16.
//

#ifndef DIRECTSPARSEVO_COMMON_H
#define DIRECTSPARSEVO_COMMON_H

// std
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <set>
#include <unordered_map>
#include <algorithm>
#include <mutex>
#include <thread>

//using namespace std;

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector> // for vector of Eigen objects

// for Sophus
#include <sophus/se3.h>
#include <sophus/so3.h>

// opencv
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// for glog
#include <glog/logging.h>

typedef Eigen::Matrix<double, 6, 1> Vector6d;

#endif //DIRECTSPARSEVO_COMMON_H
