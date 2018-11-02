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

class SubRoad: public IView, public IXML
{
public:
    int mRoadId = -1;
    int mDirection = 0;
    int mPrevJid = -1;
    int mNextJid = -1;

    std::shared_ptr<Road> pBaseRoad;
    std::vector<SecPtr> mSubRoadSecPtrs;
    std::vector<SigPtr> mSubRoadSigPtrs;

public:
    SubRoad(int direction);

    void Init(std::shared_ptr<Road> p_road);

    Pose GetStartPose();

    std::vector<std::vector<Pose>> GetLanePose();

    void Send(Sender &sender) override;
    boost::property_tree::ptree ToXML() override;
    void FromXML(const pt::ptree &p) override;
};


class Road : public IView, public IXML
{
public:
    static unsigned int ROAD_ID;
    unsigned int mRoadId;
    double mLength;
    int mDirection;

    std::vector<SecPtr> mSecPtrs;
    std::vector<SigPtr> mSigPtrs;

    int mPrevJid;
    int mNextJid;

    Pose mStartPose;
    Pose mEndPose;

    
    SubRoad mForwardRoad;
    SubRoad mBackwardRoad;
    
    explicit Road(Pose _start_pose = Pose());

    SecPtr AddSection(const Pose & _end_pose, double _ctrl_len1 = 1.0, double _ctrl_len2 = 1.0);
    void AddSignal(Vector2d v, int dir, std::string _type, std::string _info);

    int GetPrevJid() { return mPrevJid; }

    int GetNextJid() { return mNextJid; }

    int AdjacentJid(int direction)
    {
        if(direction > 0) return mNextJid;
        else return mPrevJid;
    }

    std::vector<Pose> Trajectory(int begin_lane_idx, int end_lane_idx);

    Pose GetStartPose(int direction);
    Pose GetEndPose(int direction);

    std::pair<unsigned int, int> Locate(const Vector2d & v);

    std::vector<std::vector<Pose>> GetLanePosesByDirection(int direction);

    void Send(Sender &sender) override;

    std::shared_ptr<SubRoad> GetSubRoadPtr(int dir);

    void InitSubRoad();

    double Distance(const Vector2d & v);

    boost::property_tree::ptree ToXML() override;
    void FromXML(const pt::ptree &p) override;
};
}

#endif //HDMAP_ROAD_H
