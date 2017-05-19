#include "BasicAlignment.h"

BasicAlignment::BasicAlignment(int min_level, int max_level, int n_iter,
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

size_t BasicAlignment::run(Frame* ref_frame, Frame* cur_frame)
{
    reset();
    LOG(INFO) << "ref_frame._features.size():" << ref_frame->_features.size() << std::endl;
    if (ref_frame->_features.empty()) return 0;

    ref_frame_ = ref_frame;
    cur_frame_ = cur_frame;

    Sophus::SE3 T_curr(ref_frame->_TCW);

    level_ = 0;
    for (level_ = max_level_; level_ >= min_level_; level_--)
    {
        mu_ = 0.1;
        printf ( "\nPYRAMID LEVEL %i\n---------------\n", level_ );
        _features.clear();
        optimize(T_curr);

//        if (level_ == 0)
//        PlotAlignment(T_curr);
    }
    cur_frame->_TCW = T_curr;
    return n_meas_;

}

double BasicAlignment::computeResiduals(
        const Sophus::SE3& T_cur,
        bool linearize_system,
        bool compute_weight_scale)
{
    // Warp the current image such that it aligns with the reference image
    const cv::Mat& cur_img = cur_frame_->_pyramid[level_];

    // computer the weights on the first iteration
    std::vector<float> errors;
    if (compute_weight_scale)
        errors.reserve(ref_frame_->_features.size());

    const int stride = cur_img.step;
   // LOG(INFO) << "stride:" << cur_img.cols << ", step:" << cur_img.step << std::endl;
    const int border = 10;
    const float scale = 1 << level_;
    float chi2 = 0.0;

    for(Feature* fea : ref_frame_->_features)
    {
        float ref_intensity = GetPixelValue(&(ref_frame_->_pyramid[level_]), fea->_pixel[0], fea->_pixel[1]);
        if (fea->_mappoint == nullptr) continue;

        const Eigen::Vector3d xyz_ref = fea->_mappoint->_pos_world;
        const Eigen::Vector2d uv_cur2(_pCamera->World2Pixel(xyz_ref, T_cur));
        const Eigen::Vector2f uv_cur(uv_cur2.cast<float>());  // ?? *scale
        const float u_cur = uv_cur[0];
        const float v_cur = uv_cur[1];
        int u_cur_i = floorf(u_cur);
        int v_cur_i = floorf(v_cur);

        // check if projection is within the image
        if (u_cur_i < 0 || v_cur_i< 0 ||
            u_cur_i - border < 0 || v_cur_i - border < 0 ||
            u_cur_i + border >= cur_img.cols || v_cur_i + border >= cur_img.rows)
            continue;

        _features.push_back(fea);

        float cur_intensity = GetPixelValue(&cur_img, u_cur_i, v_cur_i);
      //  float res = ref_intensity - cur_intensity;
        float res = cur_intensity - ref_intensity;

        if (compute_weight_scale)
            errors.push_back(fabsf(res));

        // robustification
        float weight = 1.0;
        if (use_weights_)
        {
            weight = weight_function_->value(res/ scale_);
        }

        chi2 += res*res*weight;
        n_meas_++;

        Eigen::Matrix<double, 2, 6> jacobian_uv_ksai = JacobXYZ2CamSVO(xyz_ref); //JacobXYZ2Cam(xyz_ref);
        Eigen::Matrix<double, 1, 2> jacobian_pixel_uv;
        jacobian_pixel_uv(0, 0) = ( GetPixelValue(&cur_img, u_cur_i+1, v_cur_i) - GetPixelValue(&cur_img, u_cur_i-1, v_cur_i) ) / 2;
        jacobian_pixel_uv(0, 1) = ( GetPixelValue(&cur_img, u_cur_i, v_cur_i+1) - GetPixelValue(&cur_img, u_cur_i, v_cur_i-1) ) / 2;

        if (linearize_system)
        {
            //const Vector6d J(-jacobian_pixel_uv * jacobian_uv_ksai);
            const Vector6d J(jacobian_pixel_uv * jacobian_uv_ksai);
            H_.noalias() += J * J.transpose() * weight;
            Jres_.noalias() -= J * res * weight;
        }
    }

    if (compute_weight_scale && iter_ == 0)
        scale_ = scale_estimator_->compute(errors);

    return chi2 / n_meas_;
}


bool BasicAlignment::solve()
{
    x_ = H_.ldlt().solve( Jres_ );
    return !( std::isnan(double(x_[0])) );
}

void BasicAlignment::update(const ModelType& T_cur_old,
                            ModelType& T_cur_new)
{
 //   T_cur_new = Sophus::SE3::exp(x_) * T_cur_old;
    T_cur_new = T_cur_old * Sophus::SE3::exp(-x_);
}

Eigen::Matrix<double,2,6> BasicAlignment::JacobXYZ2Cam(const Eigen::Vector3d& xyz)
{
    Eigen::Matrix<double,2,6> J;

    const double fx = _pCamera->fx();
    const double fy = _pCamera->fy();

    const double x = xyz[0];
    const double y = xyz[1];
    const double z_inv = 1. / xyz[2];
    const double z_inv_2 = z_inv * z_inv;

    J(0, 0) = fx*z_inv;             // fx/z
    J(0, 1) = 0.0;                  // 0
    J(0, 2) = -fx*x*z_inv_2;        // -fx*x/z^2
    J(0, 3) = y*J(0, 2);            // -fx*x*y/z^2
    J(0, 4) = fx - x*J(0, 2);       // fx + fx*x^2/z^2)
    J(0, 5) = -fx*y*z_inv;          // -fx*y/z

    J(1, 0) = 0.0;                  // 0
    J(1, 1) = fy*z_inv;             // fy/z
    J(1, 2) = -fy*y*z_inv_2;        // -fy*y/z^2
    J(1, 3) = -fy + y*J(1, 2);      // -fy - fy*y^2/z^2
    J(1, 4) = fy*x*y*z_inv_2;       // fy*x*y/z^2
    J(1, 5) = fy*x*z_inv;           // fy*x/z

    return J;
}

// curr - ref, right multiply (-ksai)
Eigen::Matrix<double,2,6> BasicAlignment::JacobXYZ2CamSVO(const Eigen::Vector3d& xyz)
{
    Eigen::Matrix<double,2,6> J;

    const double fx = _pCamera->fx();
    const double fy = _pCamera->fy();

    const double x = xyz[0];
    const double y = xyz[1];
    const double z_inv = 1. / xyz[2];
    const double z_inv_2 = z_inv * z_inv;

    J(0, 0) = -fx*z_inv;          // -fx/z
    J(0, 1) = 0.0;                // 0
    J(0, 2) = fx*x*z_inv_2;       // fx*x/z^2
    J(0, 3) = y*J(0, 2);          // fx*x*y/z^2
    J(0, 4) = -fx - x*J(0, 2);    // -fx - fx*x^2/z^2
    J(0, 5) = fx*y*z_inv;         // fx*y/z

    J(1, 0) = 0.0;                // 0
    J(1, 1) = -fy*z_inv;          // -fy/z
    J(1, 2) = fy*y*z_inv_2;       // fy*y/z^2
    J(1, 3) = fy + y*J(1, 2);     // fy + fy*y^2/z^2
    J(1, 4) = -fy*x*y*z_inv_2;    // -fy*x*y/z^2
    J(1, 5) = -fy*x*z_inv;       // -fy*x/z

    return J;
}

void BasicAlignment::PlotAlignment(const Sophus::SE3& Tcw)
{
    cv::Mat prev_color = (ref_frame_->_color).clone();
    cv::Mat color = (cur_frame_->_color).clone();

    // plot the feature points
    cv::Mat img_show ( color.rows*2, color.cols, CV_8UC3 );
    prev_color.copyTo ( img_show ( cv::Rect ( 0, 0, color.cols, color.rows ) ) );
    color.copyTo ( img_show ( cv::Rect ( 0, color.rows, color.cols,  color.rows ) ) );
    for ( Feature* fea : _features )
    {
        if ( rand() > RAND_MAX/5 )
            continue;
        Eigen::Vector3d p = fea->_mappoint->_pos_world;
        Eigen::Vector2d pixel_prev = fea->_pixel;
        Eigen::Vector2d pixel_now = _pCamera->World2Pixel(p, Tcw);
        if ( pixel_now(0,0)<0 || pixel_now(0,0)>=color.cols || pixel_now(1,0)<0 || pixel_now(1,0)>=color.rows )
            continue;

        float b = 255*float ( rand() ) /RAND_MAX;
        float g = 255*float ( rand() ) /RAND_MAX;
        float r = 255*float ( rand() ) /RAND_MAX;
        cv::circle ( img_show, cv::Point2d ( pixel_prev ( 0,0 ), pixel_prev ( 1,0 ) ), 8, cv::Scalar ( b,g,r ), 2 );
        cv::circle ( img_show, cv::Point2d ( pixel_now ( 0,0 ), pixel_now ( 1,0 ) +color.rows ), 8, cv::Scalar ( b,g,r ), 2 );
        cv::line ( img_show, cv::Point2d ( pixel_prev ( 0,0 ), pixel_prev ( 1,0 ) ), cv::Point2d ( pixel_now ( 0,0 ), pixel_now ( 1,0 ) +color.rows ), cv::Scalar ( b,g,r ), 1 );
    }
    cv::imshow ( "result", img_show );
    cv::waitKey ( 0 );
}