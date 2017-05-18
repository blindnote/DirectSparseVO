//
// Created by rochelle on 17-5-17.
//

#ifndef DIRECTSPARSEVO_MAPPOINT_H
#define DIRECTSPARSEVO_MAPPOINT_H

#include "Common.h"

struct MapPoint
{
    unsigned long _id;

    Eigen::Vector3d _pos_world = Eigen::Vector3d(0, 0, 0);
};

#endif //DIRECTSPARSEVO_MAPPOINT_H
