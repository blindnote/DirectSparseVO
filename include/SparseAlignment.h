//
// Created by rochelle on 17-5-17.
//

#ifndef DIRECTSPARSEVO_SPARSEALIGNMENT_H
#define DIRECTSPARSEVO_SPARSEALIGNMENT_H

#include "Common.h"
#include "NLLSSolver.h"
#include "Frame.h"
#include "Camera.h"

class SparseAlignment : public NLLSSolver<6, Sophus::SE3>
{
    static const int patch_halfsize_ = 2;
    static const int patch_size_ = 2 * patch_halfsize_;
    static const int patch_area_ = patch_size_ * patch_size_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    cv::Mat resimg_;

    SparseAlignment(
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
    int level_;
    int max_level_;
    int min_level_;

    // cache:
    Eigen::Matrix<double, 6, Eigen::Dynamic, Eigen::ColMajor> jacobian_cache_;
    bool have_ref_patch_cache_;
    cv::Mat ref_patch_cache_;
    std::vector<bool> visible_fts_;

    PinholeCamera* _pCamera = nullptr;

    void preComputeReferencePatches();
    virtual double computeResiduals(const Sophus::SE3& model,
                                    bool linearize_system,
                                    bool compute_weight_scale = false);
    virtual bool solve();
    virtual void update(const ModelType& old_model, ModelType& new_model);
    virtual void finishIteration();

private:
    Eigen::Matrix<double,2,6> JacobXYZ2Cam(const Eigen::Vector3d& xyz);
};

#endif //DIRECTSPARSEVO_SPARSEALIGNMENT_H
