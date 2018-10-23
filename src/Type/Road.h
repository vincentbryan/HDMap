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

    int iPrevJid;
    int iNextJid;

    explicit Road();

    void AddSection(const LaneSection & s)
    {
        mSections.emplace_back(s);
    }

    void SetPrevJid(unsigned int jid)
    {
        iPrevJid = jid;
    }

    int GetPrevJid()
    {
        return iPrevJid;
    }

    void SetNextJid(unsigned int jid)
    {
        iNextJid = jid;
    }

    int GetNextJid()
    {
        return iNextJid;
    }

    int AdjacentJid(int direction)
    {
        if(direction > 0) return iNextJid;
        else return iPrevJid;
    }

    std::vector<Pose> Trajectory(int begin_lane_idx, int end_lane_idx);

    Pose GetStartPose(int direction = 1);

    Pose GetEndPose(int direction = 1);

    std::pair<unsigned int, int> Locate(const Vector2d & v);

    std::vector<std::vector<Pose>> GetLanePosesByDirection(int direction);

};
}

#endif //HDMAP_ROAD_H
