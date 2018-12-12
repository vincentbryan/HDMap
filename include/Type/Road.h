//
// Created by vincent on 18-10-8.
//

#ifndef HDMAP_ROAD_H
#define HDMAP_ROAD_H

#include "LaneSection.h"
#include "Signal.h"
#include "common.h"
#include "kdtree.hpp"
#include <algorithm>

namespace hdmap
{
class Road : public IView, public IXML, IGeometry
{

public:

    static unsigned int ROAD_ID;

    unsigned int ID;

    double Lenght;

    int Direction;

private:

    kt::kdtree<double> mKdtree;

    std::vector<std::vector<double>> mKdtreeData;

    std::vector<Pose> mRegionPoses;


public:

    std::vector<SecPtr> mSecPtrs;
    std::vector<SigPtr> mSigPtrs;

    int mPrevJid;
    int mNextJid;

    Pose mStartPose;
    Pose mEndPose;

    explicit Road(Pose _start_pose = Pose());

    SecPtr AddSection(const Pose & _end_pose, double _ctrl_len1 = 1.0, double _ctrl_len2 = 1.0);

    void AddSignal(double _x, double _y, double _z, Angle dir, std::string _type, std::string _info);

    std::vector<SigPtr> GetSignals();

    int GetPrevJid() { return mPrevJid; }

    int GetNextJid() { return mNextJid; }

    int AdjacentJid(int direction)
    {
        if(direction > 0) return mNextJid;
        else return mPrevJid;
    }

    std::vector<Pose> Trajectory(int begin_lane_idx, int end_lane_idx);

    std::vector<Pose> GetReferenceLinePoses();

    std::vector<Pose> GetRightmostLinePoses();

    double GetDistanceFromCoor(const Coor &v);

    void GenerateRegionPoses();

    std::vector<Pose> GetRegionPoses();

    void Send(Sender &sender) override;

    boost::property_tree::ptree ToXML() override;

    void FromXML(const pt::ptree &p) override;

    bool Cover(const Coor &v) override;
};
}

#endif //HDMAP_ROAD_H
