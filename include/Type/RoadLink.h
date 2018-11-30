//
// Created by vincent on 18-10-16.
//

#ifndef HDMAP_ROADLINK_H
#define HDMAP_ROADLINK_H

#include "LaneLink.h"
#include "Interface/IView.h"
#include "Interface/IXML.h"
#include "Interface/IGeometry.h"

namespace hdmap
{
class RoadLink: public IXML, public IView
{
public:
    unsigned int mFromRoadId;
    unsigned int mToRoadId;
    std::string mDirection;
    std::vector<LaneLink> mLaneLinks;

public:
    explicit RoadLink(unsigned int _from_road_id = 0, unsigned int _to_road_id = 0, std::string _direction="");

    void AddLaneLink(int _from_lane_idx, int _to_lane_idx, Bezier _bezier);

    std::vector<Pose> GetPose(int _from_lane_idx, int _to_lane_idx);

    std::vector<std::vector<Pose>> GetAllPose();

    void Send(Sender &sender) override;
    boost::property_tree::ptree ToXML() override;
    void FromXML(const pt::ptree &p) override;
};
}

#endif //HDMAP_ROADLINK_H
