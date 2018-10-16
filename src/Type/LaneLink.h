//
// Created by vincent on 18-10-16.
//

#ifndef HDMAP_LANELINK_H
#define HDMAP_LANELINK_H

#include "../Math/Bezier.h"

namespace hdmap
{
class LaneLink
{
public:
    int iFromIndex;
    int iToIndex;
    Bezier mReferLine;

    LaneLink(int from_lane_idx, int to_lane_idx, Bezier bezier)
    {
        iFromIndex = from_lane_idx;
        iToIndex = to_lane_idx;
        mReferLine = bezier;
    }
};

}

#endif //HDMAP_LANELINK_H
