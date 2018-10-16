//
// Created by vincent on 18-10-16.
//

#ifndef HDMAP_ROADLINK_H
#define HDMAP_ROADLINK_H

#include "LaneLink.h"
namespace hdmap
{
class RoadLink
{
public:
    unsigned int iFromRoadId;
    unsigned int iToRoadId;
    std::vector<LaneLink> vLaneLinks;

public:
    explicit RoadLink(unsigned int _from_road_id = 0, unsigned int _to_road_id = 0)
    {
        iFromRoadId = _from_road_id;
        iToRoadId = _to_road_id;
    }

    void AddLaneLink(int _from_lane_idx, int _to_lane_idx, Bezier bezier)
    {
        vLaneLinks.emplace_back(LaneLink(_from_lane_idx, _to_lane_idx, bezier));
    }
};
}

#endif //HDMAP_ROADLINK_H
