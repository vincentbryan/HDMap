#ifndef HDMAP_LANELINK_H
#define HDMAP_LANELINK_H

#include "Math/Bezier.h"
#include "Interface/IXML.h"
#include "Interface/IView.h"

namespace hdmap
{
class LaneLink : public IXML, public IView
{
public:
    int mFromLaneIndex;
    int mToLaneIndex;
    Bezier mReferLine;

public:
    LaneLink(int _from_lane_idx = 0, int _to_lane_idx = 0, Bezier _bezier = Bezier());
    
    boost::property_tree::ptree ToXML() override;
    void FromXML(const pt::ptree &p) override;
    void OnSend(Sender &sender) override;
};
}

#endif //HDMAP_LANELINK_H
