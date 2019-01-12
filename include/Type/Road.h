//
// Created by vincent on 18-10-8.
//

#ifndef HDMAP_ROAD_H
#define HDMAP_ROAD_H

#include "LaneSection.h"
#include "Signal.h"
#include "Common/pointer_typedef.h"
#include "Common/kdtree.hpp"
#include <algorithm>

namespace hdmap
{
class Road : public IView, public IXML, public IGeometry
{

public:

    static unsigned int ROAD_ID;

    unsigned int ID;

    double Lenght;

    int Direction;

private:

    void GenerateRegionPoses() override;

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

    std::vector<Pose> GetReferenceLinePoses();

    std::vector<Pose> GetRightmostLinePoses();

    void OnSend(Sender &sender) override;

    boost::property_tree::ptree ToXML() override;

    void FromXML(const pt::ptree &p) override;
};
}

#endif //HDMAP_ROAD_H
