//
// Created by vincent on 18-10-8.
//

#ifndef HDMAP_LANES_H
#define HDMAP_LANES_H

#include "CubicPoly.h"
#include "LaneSection.h"
namespace hdmap
{
class Lanes
{
public:
    CubicPoly lane_offset;
    LaneSection lane_section;

    Lanes(CubicPoly c, LaneSection s);
};
}



#endif //HDMAP_LANES_H
