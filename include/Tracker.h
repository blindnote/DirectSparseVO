//
// Created by rochelle on 17-5-17.
//

#ifndef DIRECTSPARSEVO_TRACKER_H
#define DIRECTSPARSEVO_TRACKER_H

#include "Common.h"
#include "Frame.h"
#include "Camera.h"

class Tracker
{
public:
    enum TrackStatus
    {
        TRACK_NOT_READY,
        TRACK_GOOD,
        TRACK_LOST
    };

    struct Setting
    {
        int min_feature_tracking = 50;
    } _setting;

    Tracker(PinholeCamera* camera);

    void SetReference(Frame* ref);
    void TrackRefFrame(Frame* frame);

private:
    Frame* _ref = nullptr;
    TrackStatus _status = TrackStatus::TRACK_NOT_READY;

    PinholeCamera* _pCamera = nullptr;
};

#endif //DIRECTSPARSEVO_TRACKER_H
