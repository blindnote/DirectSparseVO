//
// Created by rochelle on 17-5-16.
//

#ifndef DIRECTSPARSEVO_CAMERA_H
#define DIRECTSPARSEVO_CAMERA_H

#include "Common.h"
#include "Config.h"

class PinholeCamera
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PinholeCamera()
    {
        _width = Config::Get<int>("image.width");
        _height = Config::Get<int>("image.height");

        _fx = Config::Get<float>("camera.fx");
        _fy = Config::Get<float>("camera.fy");
        _cx = Config::Get<float>("camera.cx");
        _cy = Config::Get<float>("camera.cy");

        _scale = Config::Get<float>("depth.scale");

        _d[0] = Config::Get<float>("camera.d0");
        _d[1] = Config::Get<float>("camera.d1");
        _d[2] = Config::Get<float>("camera.d2");
        _d[3] = Config::Get<float>("camera.d3");
        _d[4] = Config::Get<float>("camera.d4");
        _distortion = fabs(_d[0]) > 1e-8;

        _cvK = (cv::Mat_<float>(3, 3) << _fx, 0.0, _cx, 0.0, _fy, _cy, 0.0, 0.0, 1.0);
        _cvD = (cv::Mat_<float>(1, 5) << _d[0], _d[1], _d[2],_d[3], _d[4]);
        // http://docs.opencv.org/2.4/modules/imgproc/doc/geometric_transformations.html
        cv::initUndistortRectifyMap(_cvK, _cvD, cv::Mat_<float>::eye(3, 3), _cvK,
                                   cv::Size(_width, _height), CV_16SC2, _undist_map1, _undist_map2);

        _K << _fx, 0.0, _cx, 0.0, _fy, _cy, 0.0, 0.0, 1.0;
        _K_inv = _K.inverse();
    }

    ~PinholeCamera() {}


    inline Eigen::Vector3d World2Camera(const Eigen::Vector3d& p_w, const Sophus::SE3& T_c_w)
    {
        return T_c_w * p_w;
    }
    inline Eigen::Vector3d Camera2World(const Eigen::Vector3d& p_c, const Sophus::SE3& T_c_w)
    {
        return T_c_w.inverse() * p_c;
    }
    inline Eigen::Vector2d Camera2Pixel(const Eigen::Vector3d& p_c)
    {
        return Eigen::Vector2d(
            _fx * p_c(0, 0) / p_c(2, 0) + _cx,
            _fy * p_c(1, 0) / p_c(2, 0) + _cy
        );
    }
    inline Eigen::Vector3d Pixel2Camera(const Eigen::Vector2d& p_p, double depth = 1)
    {
        return Eigen::Vector3d(
                (p_p(0, 0) - _cx) * depth / _fx,
                (p_p(1, 0) - _cy) * depth / _fy,
                depth
        );
    }
    inline Eigen::Vector3d Pixel2World(const Eigen::Vector2d& p_p, const Sophus::SE3& T_c_w, double depth = 1)
    {
        return Camera2World( Pixel2Camera(p_p, depth),  T_c_w );
    }
    inline Eigen::Vector2d World2Pixel(const Eigen::Vector3d& p_w, const Sophus::SE3& T_c_w)
    {
        return Camera2Pixel( World2Camera(p_w, T_c_w) );
    }

    inline const Eigen::Vector2d focal_length() const
    {
        return Eigen::Vector2d(_fx, _fy);
    }
    inline float focal() const
    {
        return (_fx + _fy) / 2;
    }

    inline const cv::Mat& CameraMatrixCV() const { return _cvK; }
    inline const Eigen::Matrix3d& K() const { return _K; }
    inline const Eigen::Matrix3d& K_inv() const { return _K_inv; }
    inline int width() const { return _width; }
    inline int height() const { return _height; }
    inline float scale() const { return _scale; }
    inline float fx() const { return _fx; }
    inline float fy() const { return _fy; }
    inline float cx() const { return _cx; }
    inline float cy() const { return _cy; }
    inline float d0() const { return _d[0]; }
    inline float d1() const { return _d[1]; }
    inline float d2() const { return _d[2]; }
    inline float d3() const { return _d[3]; }
    inline float d4() const { return _d[4]; }

    inline bool InFrame(const Eigen::Vector2i& obs, int level = 0, int boundary = 0) const
    {
        return (obs[0] >= boundary && obs[0] < _width/(1 << level) - boundary &&
                obs[1] >= boundary && obs[1] < _height/(1 << level) - boundary);
    }

    // normalized pt in camera coordinates
    inline Eigen::Vector2d UndistortPoint ( const Eigen::Vector2d& pt )
    {
        double k1 = _d[0], k2 = _d[1], p1 = _d[2], p2 = _d[3], k3 = _d[4];
        // http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
        double r2 = pt[0] * pt[0] + pt[1] * pt[1];
        Eigen::Vector2d v;
        v[0] = pt[0] * (1 + k1*r2 + k2*r2*r2 + k3*r2*r2*r2) + 2*p1*pt[0]*pt[1] + p2*(r2 + 2*pt[0]*pt[0]);
        v[1] = pt[1] * (1 + k1*r2 + k2*r2*r2 + k3*r2*r2*r2) + 2*p2*pt[0]*pt[1] + p1*(r2 + 2*pt[1]*pt[1]);
        return v;
    }

    inline void UndistortImage(const cv::Mat& raw, cv::Mat& rectified)
    {
        if (_distortion)
        {
            cv::remap(raw, rectified, _undist_map1, _undist_map2, CV_INTER_LINEAR);
            // cv::undistort(raw, rectified, _cvK, _cvD);
            return;
        }

        rectified = raw.clone();
    }

    inline void PrintCameraParameters() const
    {
        std::cout << "K = " << std::endl << _K << std::endl;
    }


protected:
    int _width, _height;
    float _scale;           // depth scale
    float _fx, _fy, _cx, _cy;
    // distortion
    bool _distortion;       // is it pure pinhole model or has it radial distorion
    float _d[5];            // http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html

    cv::Mat _cvK, _cvD;
    cv::Mat _undist_map1, _undist_map2;

    Eigen::Matrix3d _K;
    Eigen::Matrix3d _K_inv;
};

#endif //DIRECTSPARSEVO_CAMERA_H
