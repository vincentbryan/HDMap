//
// Created by vincent on 18-10-16.
//

#include "RoadLink.h"
using namespace hdmap;

RoadLink::RoadLink(unsigned int _from_road_id, unsigned int _to_road_id)
{
    mFromRoadId = _from_road_id;
    mToRoadId = _to_road_id;
}

void RoadLink::AddLaneLink(int _from_lane_idx, int _to_lane_idx, Bezier _bezier)
{
    mLaneLinks.emplace_back(LaneLink(_from_lane_idx, _to_lane_idx, _bezier));
}

std::vector<Pose> RoadLink::GetPose(int _from_lane_idx, int _to_lane_idx)
{
    for(auto x : mLaneLinks)
    {
        if(x.mFromLaneIndex == _from_lane_idx && x.mToLaneIndex == _to_lane_idx)
        {
            return x.mReferLine.GetAllPose(0.1);
        }
    }
}

std::vector<std::vector<Pose>> RoadLink::GetAllPose()
{
    std::vector<std::vector<Pose>> res;

    for(auto & x : mLaneLinks)
    {
        res.emplace_back(x.mReferLine.GetAllPose(0.1));
    }

    return res;
}

boost::property_tree::ptree RoadLink::ToXML()
{
    pt::ptree p_road_link;
    p_road_link.add("<xmlattr>.from_road", mFromRoadId);
    p_road_link.add("<xmlattr>.to_road", mToRoadId);

    for(auto & k : mLaneLinks)
        p_road_link.add_child("laneLink", k.ToXML());

    return p_road_link;
}

void RoadLink::FromXML(const pt::ptree &p)
{
    for(auto & r : p.get_child(""))
    {
        if(r.first == "<xmlattr>")
        {
            mFromRoadId = r.second.get<int>("from_road");
            mToRoadId = r.second.get<int>("to_road");
        }

        if(r.first == "laneLink")
        {
            LaneLink lane_link;
            lane_link.FromXML(r.second);
            mLaneLinks.emplace_back(lane_link);
        }
    }
}
