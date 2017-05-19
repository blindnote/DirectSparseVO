//
// Created by rochelle on 17-5-19.
//

#ifndef DIRECTSPARSEVO_BASICALIGNMENT_H
#define DIRECTSPARSEVO_BASICALIGNMENT_H

#include "Common.h"
#include "NLLSSolver.h"
#include "Frame.h"
#include "Camera.h"
#include "MapPoint.h"

class BasicAlignment : public NLLSSolver<6, Sophus::SE3>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    BasicAlignment(
            int min_level, int max_level,
            int n_iter,
            Method method,
            PinholeCamera* camera,
            bool verbose = true);

    size_t run(Frame* ref_frame,
               Frame* cur_frame);

protected:
    Frame* ref_frame_;
    Frame* cur_frame_;

    PinholeCamera* _pCamera = nullptr;
    std::vector<Feature*> _features;

    int level_;
    int max_level_;
    int min_level_;

    virtual double computeResiduals(const Sophus::SE3& model,
                                    bool linearize_system,
                                    bool compute_weight_scale = false);
    virtual bool solve();
    virtual void update(const ModelType& old_model, ModelType& new_model);

private:
    Eigen::Matrix<double,2,6> JacobXYZ2Cam(const Eigen::Vector3d& xyz);
    Eigen::Matrix<double,2,6> JacobXYZ2CamSVO(const Eigen::Vector3d& xyz);

    inline float GetPixelValue(const cv::Mat* image, float px, float py)
    {
        uchar* data = & image->data[ int(py)*image->step + int(px) ];
        float xx = px - floor(px);
        float yy = py - floor(py);
        return float(
                (1 - xx) * (1 - yy) * data[0] +
                xx * (1 - yy) * data[1] +
                (1 - xx) * yy * data[image->step] +
                xx * yy * data[image->step + 1]);
    }

    void PlotAlignment(const Sophus::SE3& Tcw);
};

#endif //DIRECTSPARSEVO_BASICALIGNMENT_H
