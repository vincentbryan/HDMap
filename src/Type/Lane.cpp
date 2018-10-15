//
// Created by vincent on 18-10-8.
//

#include "Lane.h"

using namespace hdmap;

double Lane::DEFAULT_WIDTH = 2.0;
Lane::Lane(int _lane_id, CubicFunction _width, LANE_TYPE _type)
{
    land_id = _lane_id;
    width = _width;
    type = _type;
}

Lane::~Lane() {}
