//
// Created by rochelle on 17-5-17.
//

#include "Memory.h"

std::shared_ptr<Memory> Memory::_mem(new Memory());

Frame* Memory::CreateFrame()
{
    Frame* frame = new Frame;
    frame->_id = _mem->_id_frames;
    _mem->_id_frames++;
    _mem->_frames[ frame->_id ] = frame;
    return frame;
}

MapPoint* Memory::CreateMapPoint()
{
    MapPoint* pm = new MapPoint;
    pm->_id = _mem->_id_points;
    _mem->_id_points++;
    _mem->_points[ pm->_id ] = pm;
    return pm;
}

Frame* Memory::GetFrame(unsigned long id)
{
    auto iter = _mem->_frames.find(id);
    if (iter == _mem->_frames.end())
        return nullptr;
    return iter->second;
}

MapPoint* Memory::GetMapPoint(unsigned long id)
{
    auto iter = _mem->_points.find(id);
    if (iter == _mem->_points.end())
        return nullptr;
    return iter->second;
}

void Memory::Clean()
{
    for(auto iter = _frames.begin(); iter != _frames.end(); iter++)
    {
        LOG(INFO) << "delete frame " << iter->first << ", " << iter->second << std::endl;
        delete iter->second;
    }
    _mem->_frames.clear();

    for(auto& point_pair: _mem->_points)
    {
        delete point_pair.second;
    }
    _mem->_points.clear();
}