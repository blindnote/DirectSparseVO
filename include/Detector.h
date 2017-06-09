//
// Created by rochelle on 17-5-25.
//

#ifndef DIRECTSPARSEVO_DETECTOR_H
#define DIRECTSPARSEVO_DETECTOR_H

#include "Common.h"
#include "Frame.h"

class Detector
{
public:
    struct Setting
    {
        int _image_width, _image_height;
        int _cell_size;
        int _grid_rows, _grid_cols;
        double _detection_threshold;
    } static _setting;

    Detector();

    void Detect(Frame* frame, bool overwrite_existing_features = true);


private:
    void SetExistingFeatures(Frame* frame);

    float ShiTomasiScore(const cv::Mat& img, const int& u, const int& v) const;

    std::vector<Feature*> _old_features;
    std::vector<Feature*> _new_features;
};

#endif //DIRECTSPARSEVO_DETECTOR_H
