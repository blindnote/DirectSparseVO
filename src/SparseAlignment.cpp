#include <MapPoint.h>
#include "SparseAlignment.h"

SparseAlignment::SparseAlignment(int min_level, int max_level, int n_iter,
                                 Method method, PinholeCamera* camera, bool verbose)
: min_level_(min_level), max_level_(max_level)
{
    n_iter_init_ = n_iter;
    n_iter_ = n_iter_init_;
    method_ = method;
    verbose_ = verbose;
    eps_ = 0.000001;
    _pCamera = camera;
}

size_t SparseAlignment::run(Frame* ref_frame, Frame* cur_frame)
{
    reset();
   // LOG(INFO) << "ref_frame._features.size():" << ref_frame->_features.size() << std::endl;
    if (ref_frame->_features.empty()) return 0;

    ref_frame_ = ref_frame;
    cur_frame_ = cur_frame;

    ref_patch_cache_ = cv::Mat(ref_frame->_features.size(), patch_area_, CV_32F);
//    std::cout << "ref_patch_cache_.rows:" << ref_patch_cache_.rows
//              << ", ref_patch_cache_.cols:" << ref_patch_cache_.cols << std::endl;
    jacobian_cache_.resize(Eigen::NoChange, ref_patch_cache_.rows * patch_area_);
    visible_fts_.resize(ref_patch_cache_.rows, false);

    Sophus::SE3 T_curr_from_ref(cur_frame_->_TCW * ref_frame_->_TCW.inverse());

    for (level_ = max_level_; level_ >= min_level_; level_--)
    {
        mu_ = 0.1;
        jacobian_cache_.setZero();
        have_ref_patch_cache_ = false;
        if (verbose_)
            printf ( "\nPYRAMID LEVEL %i\n---------------\n", level_ );
        optimize(T_curr_from_ref);
    }
    cur_frame->_TCW = T_curr_from_ref * ref_frame->_TCW;
    return n_meas_ / patch_area_;
}

void SparseAlignment::preComputeReferencePatches()
{
    const int border = patch_halfsize_ + 1;
    //const cv::Mat& ref_img = ref_frame_->_pyramid[level_];
    const cv::Mat& ref_img = ref_frame_->_pyramid.at(level_);
    const int stride = ref_img.cols;
    const float scale = 1.0f / ( 1<<level_ );          // 1.0f/(1<<level-)
    const Eigen::Vector3d ref_pos = ref_frame_->GetCamCenter();

    const double focal_length = _pCamera->focal();
    size_t feature_counter = 0;
    std::vector<bool>::iterator visibility_it = visible_fts_.begin();

    for (auto it = ref_frame_->_features.begin();
         it != ref_frame_->_features.end();
         ++it, ++feature_counter, ++visibility_it)
    {
        // check if reference with patch size is within image
        Feature* fea = (*it);
        const float u_ref = fea->_pixel[0] * scale;
        const float v_ref = fea->_pixel[1] * scale;
        const int u_ref_i = floorf(u_ref);
        const int v_ref_i = floorf(v_ref);
        if (fea->_mappoint == nullptr ||
            u_ref_i - border < 0 || v_ref_i - border < 0 ||
            u_ref_i + border >= ref_img.cols ||
            v_ref_i + border >= ref_img.rows)
            continue;

        *visibility_it = true;

        // cannot just take the 3d points coordinate because of the reprojection errors in the reference image!!!
        // const double depth( (fea->_mappoint->_pos_world - ref_pos).norm() );
       // LOG(INFO) << "depth = " << depth << ", features depth = " << fea->_depth << std::endl;
        /*
        const double depth = (fea->_mappoint->_pos_world - ref_frame_->GetCamCenter()).norm();
        cv::Point2f tmp_uv(fea->_pixel[0], fea->_pixel[1]), px;
        const cv::Mat src_pt(1, 1, CV_32FC2, &tmp_uv.x);
        cv::Mat dst_pt(1, 1, CV_32FC2, &px.x);
        cv::undistortPoints(src_pt, dst_pt, _pCamera->CameraMatrixCV().clone(), _pCamera->CameraMatrixCVD().clone());
        Eigen::Vector3d tmp_xyz;
        tmp_xyz[0] = px.x;
        tmp_xyz[1] = px.y;
        tmp_xyz[2] = 1.0;

        const Eigen::Vector3d xyz_ref( tmp_xyz *  depth);
         */
        //const Vector3d xyz_ref(fea->f*depth);  //f(frame->cam_->cam2world(px))
        const Eigen::Vector3d xyz_ref( _pCamera->Pixel2Camera(fea->_pixel, fea->_depth) );

        // evaluate projection jacobian
        Eigen::Matrix<double, 2, 6> frame_jac = JacobXYZ2Cam(xyz_ref);

        // combine bilateral interpolation weights for reference image
        const float subpix_u_ref = u_ref - u_ref_i;
        const float subpix_v_ref = v_ref - v_ref_i;
        const float w_ref_tl = (1.0 - subpix_u_ref) * ( 1.0 - subpix_v_ref);
        const float w_ref_tr = subpix_u_ref * (1.0 - subpix_v_ref);
        const float w_ref_bl = (1.0 - subpix_u_ref) * subpix_v_ref;
        const float w_ref_br = subpix_u_ref * subpix_v_ref;

        size_t pixel_counter = 0;
        float* cache_ptr = reinterpret_cast<float*>(ref_patch_cache_.data) + patch_area_ * feature_counter;
        for (int y = 0; y < patch_size_; ++y)
        {
            uint8_t* ref_img_ptr = (uint8_t*) ref_img.data + (v_ref_i + y - patch_halfsize_)*stride + (u_ref_i - patch_halfsize_);
            for (int x = 0; x < patch_size_; ++x, ++ref_img_ptr, ++cache_ptr, ++pixel_counter)
            {
                // precompute interpolated reference path color
                *cache_ptr = w_ref_tl*ref_img_ptr[0] + w_ref_tr*ref_img_ptr[1] + w_ref_bl*ref_img_ptr[stride] + w_ref_br*ref_img_ptr[stride+1];

                // we use the inverse compositional: thereby we can take the gradient always at the same position
                // get gradient of warped image (~gradient at warped position)
                float dx = 0.5f * ( (w_ref_tl*ref_img_ptr[1] + w_ref_tr*ref_img_ptr[2] + w_ref_bl*ref_img_ptr[stride+1] + w_ref_br*ref_img_ptr[stride+2])
                                  - (w_ref_tl*ref_img_ptr[-1] + w_ref_tr*ref_img_ptr[0] + w_ref_bl*ref_img_ptr[stride-1] + w_ref_br*ref_img_ptr[stride]) );
                float dy = 0.5f * ( (w_ref_tl*ref_img_ptr[stride] + w_ref_tr*ref_img_ptr[1+stride] + w_ref_bl*ref_img_ptr[stride*2] + w_ref_br*ref_img_ptr[stride*2+1])
                                  - (w_ref_tl*ref_img_ptr[-stride] + w_ref_tr*ref_img_ptr[1-stride] + w_ref_bl*ref_img_ptr[0] + w_ref_br*ref_img_ptr[1]) );

                // cache the jacobian
                jacobian_cache_.col(feature_counter * patch_area_ + pixel_counter) =
                    ( dx*frame_jac.row(0) + dy*frame_jac.row(1) )*(focal_length / (1 << level_));
            }
        }
    }

    have_ref_patch_cache_ = true;
}

double SparseAlignment::computeResiduals(const Sophus::SE3& T_cur_from_ref,
                                         bool linearize_system,
                                         bool compute_weight_scale)
{
    // Warp the current image such that it aligns with the reference image
    const cv::Mat& cur_img = cur_frame_->_pyramid.at(level_);

    if (linearize_system)
        resimg_ = cv::Mat(cur_img.size(), CV_32F, cv::Scalar(0));

    if (!have_ref_patch_cache_)
        preComputeReferencePatches();

    // computer the weights on the first iteration
    std::vector<float> errors;
    if (compute_weight_scale)
        errors.reserve(visible_fts_.size());

    const int stride = cur_img.cols;
    const int border = patch_halfsize_ + 1;
    const float scale = 1.0f / (1 << level_);
    const Eigen::Vector3d ref_pos(ref_frame_->GetCamCenter());
    float chi2 = 0.0;
    size_t feature_counter = 0; // is used to compute the index of the cached jacobian
    std::vector<bool>::iterator visibility_it = visible_fts_.begin();
    for(auto it = ref_frame_->_features.begin();
        it != ref_frame_->_features.end();
        ++it, ++feature_counter, ++visibility_it)
    {
        // check if feature is within image
        if (!*visibility_it) continue;

        Feature* fea = (*it);
        // compute pixel location in cur img
        /*
        const double depth = (fea->_mappoint->_pos_world - ref_pos).norm();
        cv::Point2f tmp_uv(fea->_pixel[0], fea->_pixel[1]), px;
        const cv::Mat src_pt(1, 1, CV_32FC2, &tmp_uv.x);
        cv::Mat dst_pt(1, 1, CV_32FC2, &px.x);
        cv::undistortPoints(src_pt, dst_pt, _pCamera->CameraMatrixCV().clone(), _pCamera->CameraMatrixCVD().clone());
        Eigen::Vector3d tmp_xyz;
        tmp_xyz[0] = px.x;
        tmp_xyz[1] = px.y;
        tmp_xyz[2] = 1.0;
        const Eigen::Vector3d xyz_ref( tmp_xyz *  depth);
         */
        //const Eigen::Vector3d xyz_ref(fea->f*depth);
        const Eigen::Vector3d xyz_ref(_pCamera->Pixel2Camera(fea->_pixel, fea->_depth));
        const Eigen::Vector3d xyz_cur(T_cur_from_ref * xyz_ref);
        const Eigen::Vector2f uv_cur_pyr(_pCamera->Camera2Pixel(xyz_cur).cast<float>() * scale);
        const float u_cur = uv_cur_pyr[0];
        const float v_cur = uv_cur_pyr[1];
        int u_cur_i = floorf(u_cur);
        int v_cur_i = floorf(v_cur);

        // check if projection is within the image
        if (u_cur_i < 0 || v_cur_i< 0 ||
            u_cur_i - border < 0 || v_cur_i - border < 0 ||
            u_cur_i + border >= cur_img.cols || v_cur_i + border >= cur_img.rows)
            continue;

        // compute bilateral interpolation weights for current image
        const float subpix_u_cur = u_cur - u_cur_i;
        const float subpix_v_cur = v_cur - v_cur_i;
        const float w_cur_tl = (1.0 - subpix_u_cur) * ( 1.0 - subpix_v_cur);
        const float w_cur_tr = subpix_u_cur * (1.0 - subpix_v_cur);
        const float w_cur_bl = (1.0 - subpix_u_cur) * subpix_v_cur;
        const float w_cur_br = subpix_u_cur * subpix_v_cur;

        float* ref_patch_cache_ptr = reinterpret_cast<float*>(ref_patch_cache_.data) + patch_area_ * feature_counter;
        size_t pixel_counter = 0;
        for (int y = 0; y < patch_size_; ++y)
        {
            uint8_t* cur_img_ptr = (uint8_t*) cur_img.data + (v_cur_i + y - patch_halfsize_)*stride + (u_cur_i - patch_halfsize_);

            for (int x = 0; x < patch_size_; ++x, ++pixel_counter, ++cur_img_ptr, ++ref_patch_cache_ptr)
            {
                // compute residual
                const float intensity_cur = w_cur_tl*cur_img_ptr[0] + w_cur_tr*cur_img_ptr[1] + w_cur_bl*cur_img_ptr[stride] + w_cur_br*cur_img_ptr[stride+1];
                const float res = intensity_cur - (*ref_patch_cache_ptr);

                // used to compute scale for robust cost
                if (compute_weight_scale)
                    errors.push_back(fabsf(res));

                // robustification
                float weight = 1.0;
                if (use_weights_)
                {
                    weight = weight_function_->value(res / scale_);
                }

                chi2 += res*res*weight;
                n_meas_++;

                if (linearize_system)
                {
                    // compute Jacobian, weighted HEssian and weighted "steepest descend images" (times error)
                    const Vector6d J(jacobian_cache_.col(feature_counter * patch_area_ + pixel_counter));
                    H_.noalias() += J * J.transpose() * weight;
                    Jres_.noalias() -= J * res * weight;

                    resimg_.at<float> ( ( int ) v_cur+y-patch_halfsize_, ( int ) u_cur+x-patch_halfsize_ ) = res/255.0;
                }
            }
        }
    }

    // compute the weights on the first iteration
    if (compute_weight_scale && iter_ == 0)
        scale_ = scale_estimator_->compute(errors);

    return chi2 / n_meas_;
}

bool SparseAlignment::solve()
{
    x_ = H_.ldlt().solve( Jres_ );
    return !( std::isnan(double(x_[0])) );
}

void SparseAlignment::update(const ModelType& T_cur_from_ref_old,
                             ModelType& T_cur_from_ref_new)
{
    T_cur_from_ref_new = T_cur_from_ref_old * Sophus::SE3::exp(-x_);
}

void SparseAlignment::finishIteration()
{
//    cv::namedWindow ( "residuals", CV_WINDOW_AUTOSIZE );
//    cv::imshow ( "residuals", resimg_ * 10 );
//    //cv::waitKey ( 0 );
//    cv::waitKey ( 200 );
//    cv::destroyWindow("residuals");
}

// *************************************************************************************
// 一些固定的雅可比
// xyz 到 相机坐标 的雅可比，平移在前
// 这里已经取了负号，不要再取一遍！
/// unit plane coordinates uv (focal length = 1).
Eigen::Matrix<double,2,6> SparseAlignment::JacobXYZ2Cam(const Eigen::Vector3d& xyz)
{
    Eigen::Matrix<double,2,6> J;

    const double x = xyz[0];
    const double y = xyz[1];
    const double z_inv = 1. / xyz[2];
    const double z_inv_2 = z_inv * z_inv;

    J(0, 0) = -z_inv;               // -1/z
    J(0, 1) = 0.0;                  // 0
    J(0, 2) = x*z_inv_2;            // x/z^2
    J(0, 3) = y*J(0, 2);            // x*y/z^2
    J(0, 4) = -(1.0 + x*J(0, 2));   // -(1.0 + x^2/z^2)
    J(0, 5) = y*z_inv;              // y/z

    J(1, 0) = 0.0;                  // 0
    J(1, 1) = -z_inv;               // -1/z
    J(1, 2) = y*z_inv_2;            // y/z^2
    J(1, 3) = 1.0 + y*J(1, 2);      // 1.0 + y^2/z^2
    J(1, 4) = -J(0, 3);             // -x*y/z^2
    J(1, 5) = -x*z_inv;             // x/z

    return J;
}