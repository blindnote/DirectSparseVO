#include "Tracker.h"
#include "Config.h"
#include "SparseAlignment.h"

Tracker::Tracker(PinholeCamera* camera)
: _pCamera(camera)
{
    _setting.min_feature_tracking = Config::Get<int>("tracker.min_features");
}

void Tracker::SetReference(Frame* ref)
{
    if (ref->_features.size() < _setting.min_feature_tracking)
    {
        LOG(WARNING) << "Track a reference frame without enough features: " << ref->_features.size() << ", abort." << std::endl;
        _status = TrackStatus::TRACK_NOT_READY;
        return;
    }

    this->_ref = ref;
    _status = TrackStatus::TRACK_GOOD;
}

void Tracker::TrackRefFrame(Frame* frame)
{
    if(_status == TRACK_NOT_READY)
    {
        LOG(WARNING) << "reference frame is not ready, please set reference first! " << std::endl;
        return;
    }
    if(_status == TRACK_LOST)
    {
        LOG(WARNING) << "track is lost, please reset it" << std::endl;
        return;
    }

    LOG(INFO) << "Tracking ..." << std::endl;
    /*
    if ( _px_curr.size() < _setting.min_feature_tracking )
    {
        _status = TRACK_LOST;
        LOG(WARNING) << "Track with little features, set it as lost." << std::endl;
    }
     */

    std::shared_ptr<SparseAlignment> pAlign = std::make_shared<SparseAlignment>(30, SparseAlignment::GaussNewton,
                                                                                _pCamera, true);
    pAlign->run(_ref, frame);
}