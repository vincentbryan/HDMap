//
// Created by vincent on 18-10-8.
//

#include "Lane.h"

using namespace hdmap;

double Lane::DEFAULT_WIDTH = 2.0;
Lane::Lane(int _lane_id, std::shared_ptr<Curve> p_width, LANE_TYPE _type)
{
    land_id = _lane_id;
    pWidth = p_width;
    type = _type;
}

Lane::~Lane()
{
    pWidth = nullptr;
}
