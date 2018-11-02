//
// Created by vincent on 18-10-16.
//

#include "RoadLink.h"
using namespace hdmap;
/*
void SubRoadLink::Send(Sender &sender)
{
    for(auto & x : mLaneLinks)
    {
        auto p = x.mReferLine.GetAllPose(0.1);
        sender.SendPoses(p, 1.0, 156.0/255, 30.0/255, 1.0, 0.0, 0.08);
    }
}

boost::property_tree::ptree SubRoadLink::ToXML()
{
    pt::ptree p_road_link;
    p_road_link.add("<xmlattr>.from_road", mFromRoadId);
    p_road_link.add("<xmlattr>.to_road", mToRoadId);

    for(auto & m : mLaneLinks)
        p_road_link.add_child("laneLink", m.ToXML());

    return p_road_link;
}
void SubRoadLink::FromXML(const pt::ptree &p)
{

}
*/

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

/*
SubRoadLink RoadLink::operator()(int _form_dir, int _to_dir)
{
    SubRoadLink slk;
    slk.mFromRoadId = mFromRoadId;
    slk.mToRoadId = mToRoadId;

    for(auto const & x : mLaneLinks)
    {
        if(x.mFromLaneIndex * _form_dir > 0 and x.mToLaneIndex * _to_dir > 0)
            slk.mLaneLinks.emplace_back(x);
    }

    return slk;
}
*/

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
