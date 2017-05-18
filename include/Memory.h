//
// Created by rochelle on 17-5-17.
//

#ifndef DIRECTSPARSEVO_MEMORY_H
#define DIRECTSPARSEVO_MEMORY_H

#include "Common.h"
#include "MapPoint.h"
#include "Frame.h"

class Memory
{
public:
    Memory(const Memory&) = delete;
    Memory& operator = (const Memory&) = delete;
    ~Memory()
    {
        if (_mem != nullptr)
            _mem->Clean();
        _mem = nullptr;
    }

    static Frame* CreateFrame();
    static MapPoint* CreateMapPoint();

    static Frame* GetFrame(unsigned long id);
    static MapPoint* GetMapPoint(unsigned long id);

    void Clean();

private:
    Memory() {}

private:
    std::unordered_map<unsigned long, Frame*> _frames;
    std::unordered_map<unsigned long, MapPoint*> _points;
    unsigned long _id_frames = 0, _id_points = 0;
    static std::shared_ptr<Memory> _mem;
};

#endif //DIRECTSPARSEVO_MEMORY_H
