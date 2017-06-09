#include <fast/fast.h>
#include "Detector.h"

Detector::Setting Detector::_setting;

Detector::Detector()
{
    _setting._image_width = Config::Get<int>("image.width");
    _setting._image_height = Config::Get<int>("image.height");
    _setting._cell_size = Config::Get<int>("feature.cell");
    _setting._grid_rows = ceil(double ( _setting._image_height ) / _setting._cell_size);
    _setting._grid_cols = ceil(double ( _setting._image_width ) / _setting._cell_size);
    _setting._detection_threshold = Config::Get<double>("feature.detection_threshold");

    _old_features = std::vector<Feature*>(_setting._grid_cols * _setting._grid_rows, nullptr);
}

void Detector::Detect(Frame *frame, bool overwrite_existing_features)
{
    _new_features = std::vector<Feature*>(_setting._grid_cols * _setting._grid_rows, nullptr);

    if (overwrite_existing_features)
    {
        _old_features = std::vector<Feature*>(_setting._grid_cols * _setting._grid_rows, nullptr);
        frame->CleanAllFeatures();
    }
    else
    {
        SetExistingFeatures(frame);
    }

    for (int L = 0; L < Frame::_setting._pyramid_level; ++L)
    {
        const int scale = (1 << L);
        std::vector<fast::fast_xy> fast_corners;

#if __SSE2__
        fast::fast_corner_detect_10_sse2((fast::fast_byte*)frame->_pyramid[L].data,
                                         frame->_pyramid[L].cols,
                                         frame->_pyramid[L].rows,
                                         frame->_pyramid[L].cols,
                                         _setting._detection_threshold,
                                         fast_corners);
#elif HAVE_FAST_NEON
        fast::fast_corner_detect_9_neon((fast::fast_byte*)frame->_pyramid[L].data,
                                        frame->_pyramid[L].cols,
                                        frame->_pyramid[L].rows,
                                        frame->_pyramid[L].cols,
                                        _setting._detection_threshold,
                                        fast_corners);
#else
        fast::fast_corner_detect_10((fast::fast_byte*)frame->_pyramid[L].data,
                                    frame->_pyramid[L].cols,
                                    frame->_pyramid[L].rows,
                                    frame->_pyramid[L].cols,
                                    _setting._detection_threshold,
                                    fast_corners);
#endif
        // nms: non-maximum suprression
        std::vector<int> scores, nm_corners;
        fast::fast_corner_score_10((fast::fast_byte*)frame->_pyramid[L].data, frame->_pyramid[L].cols,
                                   fast_corners, _setting._detection_threshold, scores);
        fast::fast_nonmax_3x3(fast_corners, scores, nm_corners);

        for(auto it = nm_corners.begin(); it != nm_corners.end(); ++it)
        {
            fast::fast_xy& xy = fast_corners.at(*it);
            if (!frame->InFrame(Eigen::Vector2d(xy.x, xy.y), L, 20))   // ***** diff
                continue;

            const int gy = static_cast<int>((xy.y * scale) / _setting._cell_size);
            const int gx = static_cast<int>((xy.x * scale) / _setting._cell_size);
            const size_t k = gy * _setting._grid_cols + gx;

            //if (k > _new_features.size()) continue;  // shouldn't happen

            if (_old_features[k]) continue;     // ????????? bu fu gai?

            const float score = this->ShiTomasiScore(frame->_pyramid[L], xy.x, xy.y);
            if (_new_features[k])
            {
                if (score > _new_features[k]->_score)
                    delete _new_features[k];
                else
                    continue;
            }

            _new_features[k] = new Feature(frame, L, scale*Eigen::Vector2d(xy.x, xy.y), score);
        }
    }

    int cnt_new_features = 0;
    LOG(INFO)<< "old features: " << frame->_features.size() << std::endl;
    for (Feature* fea: _new_features)
    {
        if (!fea) continue;
        if (fea->_score < _setting._detection_threshold) continue;

        cnt_new_features++;
        frame->_features.push_back( fea );
        fea->_frame = frame;
    }

    LOG(INFO) << "add total " << cnt_new_features << " new features." << std::endl;
}

float Detector::ShiTomasiScore(const cv::Mat& img, const int& u, const int& v) const
{
    assert(img.type() == CV_8UC1);

    float dXX = 0.0;
    float dYY = 0.0;
    float dXY = 0.0;

    const int half_box_size = 4;
    const int box_size = 2 * half_box_size;
    const int box_area = box_size * box_size;
    const int x_min = u - half_box_size;
    const int x_max = u + half_box_size;
    const int y_min = v - half_box_size;
    const int y_max = v + half_box_size;

    // patch is too close to the boundary
    if (x_min < 1 || x_max >= img.cols - 1 || y_min < 1 || y_max >= img.rows -1 )
        return 0.0;

    const int stride = img.step.p[0];
    for (int y = y_min; y < y_max; ++y)
    {
        const uint8_t* ptr_left = img.data + stride * y + x_min - 1;
        const uint8_t* ptr_right = img.data + stride * y + x_min + 1;
        const uint8_t* ptr_top = img.data + stride * (y - 1) + x_min;
        const uint8_t* ptr_bottom = img.data + stride * (y + 1) + x_min;
        for(int x = 0; x < box_size; ++x, ++ptr_left, ++ptr_right, ++ptr_top, ++ptr_bottom)
        {
            float dx = *ptr_right - *ptr_left;
            float dy = *ptr_bottom - *ptr_top;
            dXX += dx * dx;
            dYY += dy * dy;
            dXY += dx * dy;
        }
    }

    dXX = dXX / (2.0 * box_area);
    dYY = dYY / (2.0 * box_area);
    dXY = dXY / (2.0 * box_area);
    return 0.5 * (dXX + dYY - sqrt((dXX+dYY)*(dXX+dYY) - 4 * (dXX*dYY-dXY*dXY)));
}

void Detector::SetExistingFeatures(Frame* frame)
{
    for(Feature*& fea : _old_features)
        fea = nullptr;

    for(Feature* fea : frame->_features)
    {
        int gx = static_cast<int>(fea->_pixel[0] / _setting._cell_size);
        int gy = static_cast<int>(fea->_pixel[1] / _setting._cell_size);
        size_t k = gy * _setting._grid_cols + gx;

//        if (k > _old_features.size())
//        {
//            continue;
//        } // should not happen

        _old_features[k] = fea;
    }
}
