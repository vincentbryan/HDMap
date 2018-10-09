//
// Created by vincent on 18-10-8.
//

#include "Lane.h"

using namespace hdmap;

Lane::Lane(int _lane_id, int _level, LANE_TYPE _type, CubicPoly _width) : width(_width)
{
    land_id = _lane_id;
    level = _level;
    type = _type;
}

void Lane::AddLink(std::pair<int, int> _link)
{
    links.emplace_back(_link);
}