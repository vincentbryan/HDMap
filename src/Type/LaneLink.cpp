//
// Created by vincent on 18-10-16.
//

#include "LaneLink.h"
using namespace hdmap;

LaneLink::LaneLink(int _from_lane_idx, int _to_lane_idx, hdmap::Bezier _bezier)
{
    mFromLaneIndex = _from_lane_idx;
    mToLaneIndex = _to_lane_idx;
    mReferLine = _bezier;
}

boost::property_tree::ptree LaneLink::ToXML()
{
    pt::ptree p_lane_link;
    p_lane_link.add("<xmlattr>.from_lane", mFromLaneIndex);
    p_lane_link.add("<xmlattr>.to_lane", mToLaneIndex);

    for(auto & n : mReferLine.GetParam())
        p_lane_link.add("param", n);

    return p_lane_link;
}

void LaneLink::FromXML(const pt::ptree &p)
{
    std::vector<double> res;
    for(auto r : p.get_child(""))
    {
        if(r.first == "<xmlattr>")
        {
            mFromLaneIndex = r.second.get<int>("from_lane");
            mToLaneIndex = r.second.get<int>("to_lane");
        }

        if(r.first == "param")
        {
            res.emplace_back(std::atof(r.second.data().c_str()));
        }
    }
    assert(res.size() == 8);
    mReferLine = Bezier(res);
}

void LaneLink::Send(Sender &sender)
{
    auto x = mReferLine.GetAllPose(1.0);
    
    auto m = sender.GetLineStrip(x, 234.0/255, 247.0/255, 134.0/255, 1.0);
    sender.array.markers.emplace_back(m);
    
    auto from = sender.GetArrow(x.front(), 95.0/255, 217.0/255, 205.0/255, 1.0);
    sender.array.markers.emplace_back(from);
    
    auto to = sender.GetArrow(x.back(), 234.0/255, 247.0/255, 134.0/255, 1.0);
    sender.array.markers.emplace_back(to);
    
    sender.Send();
}
