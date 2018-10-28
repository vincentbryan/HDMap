//
// Created by vincent on 18-10-8.
//

#ifndef HDMAP_ROAD_H
#define HDMAP_ROAD_H

#include "LaneSection.h"
#include "Signal.h"
#include "common.h"
#include <algorithm>

namespace hdmap
{
class SubRoad: public IView
{
public:
    int iRoadId = -1;
    int direction = 0;
    int iPrevJid = -1;
    int iNextJid = -1;

    std::shared_ptr<Road> pBaseRoad;
    std::vector<SecPtr> mSubRoadSecPtrs;
    std::vector<SigPtr> mSubRoadSigPtrs;

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

    std::vector<SecPtr> mSecPtrs;
    std::vector<SigPtr> mSigPtrs;

    int iPrevJid;
    int iNextJid;

    Pose mStartPose;
    Pose mEndPose;

    SubRoad mForwardRoad;
    SubRoad mBackwardRoad;

    explicit Road(Pose _start_pose = Pose());

    SecPtr AddSection(const Pose & _end_pose, double _ctrl_len1 = 1.0, double _ctrl_len2 = 1.0);
    void AddSignal(Vector2d v, int dir, std::string _type, std::string _info);

    int GetPrevJid()
    {
        return iPrevJid;
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
