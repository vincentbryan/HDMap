//
// Created by vincent on 18-10-16.
//

#ifndef HDMAP_LANELINK_H
#define HDMAP_LANELINK_H

#include "../Math/Bezier.h"
#include "../Interface/IXML.h"

namespace hdmap
{
class LaneLink : public IXML
{
public:
    int mFromLaneIndex;
    int mToLaneIndex;
    Bezier mReferLine;

public:
    LaneLink(int _from_lane_idx = 0, int _to_lane_idx = 0, Bezier _bezier = Bezier());
    boost::property_tree::ptree ToXML() override;
    void FromXML(const pt::ptree &p) override;
};
}

#endif //HDMAP_LANELINK_H
