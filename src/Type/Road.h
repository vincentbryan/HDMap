//
// Created by vincent on 18-10-8.
//

#ifndef HDMAP_ROAD_H
#define HDMAP_ROAD_H


#include "LaneSection.h"
#include "Signal.h"
#include <algorithm>

namespace hdmap
{
class Road;
class SubRoad: public IView
{
public:
    int iRoadId = -1;
    int direction = 0;
    int iPrevJid = -1;
    int iNextJid = -1;

    std::shared_ptr<Road> pBaseRoad;
    std::vector<LaneSection> mSubRoadSection;
    std::vector<Signal> mSubRoadSignals;

public:
    SubRoad(int direction);

    void Init(std::shared_ptr<Road> p_road);

    Pose GetStartPose();

    std::vector<std::vector<Pose>> GetLanePose();

    void Send(Sender &sender) override;
};


class Road : public IView
{
public:
    static unsigned int ROAD_ID;
    unsigned int iRoadId;
    double dLength;

    std::vector<LaneSection> mSections;

    int iPrevJid;
    int iNextJid;

    SubRoad mForwardRoad;
    SubRoad mBackwardRoad;

    std::vector<Signal> mSignals;

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

    Pose GetStartPose(int direction);

    Pose GetEndPose(int direction);

    std::pair<unsigned int, int> Locate(const Vector2d & v);

    std::vector<std::vector<Pose>> GetLanePosesByDirection(int direction);

    void Send(Sender &sender) override;

    std::shared_ptr<SubRoad> GetSubRoadPtr(int dir);

    void InitSubRoad();
};
}

#endif //HDMAP_ROAD_H
