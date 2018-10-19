//
// Created by vincent on 18-10-8.
//

#ifndef HDMAP_ROAD_H
#define HDMAP_ROAD_H


#include "LaneSection.h"
#include <algorithm>

namespace hdmap
{
class Road
{
public:
    static unsigned int ROAD_ID;
    unsigned int iRoadId;
    double dLength;

    std::vector<LaneSection> mSections;

    std::vector<unsigned int> vPrevRoadId;
    std::vector<unsigned int> vNextRoadId;

    explicit Road();

    void AddSection(const LaneSection & s)
    {
        mSections.emplace_back(s);
    }

    std::pair<int, double> Distance(const Vector2d & v);

    std::vector<unsigned int> GetPrevRoads()
    {
        return vPrevRoadId;
    }

    std::vector<unsigned int> GetNextRoads()
    {
        return vNextRoadId;
    }

    void AddNextRoadId(unsigned int id)
    {
        if(std::find(vNextRoadId.begin(), vNextRoadId.end(), id) == vNextRoadId.end())
        {
            vNextRoadId.emplace_back(id);
        }
    }

    void AddPrevRoadId(unsigned int id)
    {
        if(std::find(vPrevRoadId.begin(), vPrevRoadId.end(), id) == vPrevRoadId.end())
        {
            vPrevRoadId.emplace_back(id);
        }
    }

    std::vector<Pose> Trajectory(int begin_lane_idx, int end_lane_idx);
};
}

#endif //HDMAP_ROAD_H
