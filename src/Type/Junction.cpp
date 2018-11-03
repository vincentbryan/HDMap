//
// Created by vincent on 18-10-13.
//

#include "Junction.h"

using namespace hdmap;

unsigned int Junction::JUNCTION_ID = 0;

Junction::Junction()
{
    mJunctionId = JUNCTION_ID++;
}

void Junction::AddConnection(unsigned int _from_road_id, int _from_lane_idx, Pose _from_lane_pose,
                             unsigned int _to_road_id, int _to_lane_idx, Pose _to_lane_pose,
                             double _ctrl_len1, double _ctrl_len2)
{
    std::pair<unsigned int, unsigned int> p(_from_road_id, _to_road_id);
    if(mRoadLinks.find(p) != mRoadLinks.end())
    {
        mRoadLinks[p].AddLaneLink(_from_lane_idx,
                                  _to_lane_idx,
                                  Bezier(_from_lane_pose, _to_lane_pose, _ctrl_len1, _ctrl_len2));
    }
    else
    {
        RoadLink road_link(_from_road_id, _to_road_id);
        road_link.AddLaneLink(_from_lane_idx,
                              _to_lane_idx,
                              Bezier(_from_lane_pose, _to_lane_pose, _ctrl_len1, _ctrl_len2));
        mRoadLinks[p] = road_link;
    }
}

std::vector<std::vector<Pose>> Junction::GetAllPose()
{
    if(mPoses.empty())
        GenerateAllPose();

    return mPoses;
}

bool Junction::Check(std::pair<unsigned int, unsigned int> links)
{
    return mRoadLinks.find(links) != mRoadLinks.end();
}


std::pair<int, int> Junction::GetLink(std::pair<unsigned int, unsigned int> RoadPair)
{
    auto it = mRoadLinks.find(RoadPair);
    return  it->first;
}


std::vector<Pose> Junction::GetPose(unsigned int from_road_id,
                                    int from_lane_idx,
                                    unsigned int to_road_id,
                                    int to_lane_idx)
{
    return mRoadLinks[{from_road_id, to_road_id}].GetPose(from_lane_idx, to_lane_idx);
}

void Junction::Send(Sender &sender)
{
    if(mPoses.empty())
        GenerateAllPose();

    for(auto & x : mPoses)
    {
        auto conn = sender.GetLineStrip(x, 234.0/255, 247.0/255, 134.0/255, 1.0);
        sender.array.markers.emplace_back(conn);

        auto from = sender.GetArrow(x.front(), 95.0/255, 217.0/255, 205.0/255, 1.0);
        sender.array.markers.emplace_back(from);
        auto to = sender.GetArrow(x.back(), 234.0/255, 247.0/255, 134.0/255, 1.0);
        sender.array.markers.emplace_back(to);
    }
    sender.Send();
}

void Junction::GenerateAllPose()
{
    for(auto x : mRoadLinks)
    {
        for(auto y : x.second.mLaneLinks)
        {
            mPoses.emplace_back(y.mReferLine.GetAllPose(0.1));
        }
    }
}

RoadLink Junction::GetRoadLink (int rid1, int rid2)
{
    auto it = mRoadLinks.find(std::pair<unsigned int, unsigned int>(rid1, rid2));
    return it->second;
}

boost::property_tree::ptree Junction::ToXML()
{
    pt::ptree p_junc;
    p_junc.add("<xmlattr>.id", mJunctionId);

    for(auto & r : mRoadLinks)
    {
        p_junc.add_child("roadLink", r.second.ToXML());
    }

    return p_junc;
}

void Junction::FromXML(const pt::ptree &p)
{
    for(auto & n : p.get_child(""))
    {
        if(n.first == "<xmlattr>")
        {
            mJunctionId = n.second.get<int>("id");
        }

        if(n.first == "roadLink")
        {
            RoadLink r;
            r.FromXML(n.second);
            mRoadLinks[{r.mFromRoadId, r.mToRoadId}] = r;
        }
    }
}
double Junction::Distance(const Vector2d &v)
{
    double min_dist = 1000000;
    for(auto & ps : GetAllPose())
    {
        for(auto & p : ps)
        {
            double t = sqrt((p.x - v.x)*(p.x - v.x) + (p.y - v.y)*(p.y - v.y));
            if(t < min_dist)
                min_dist = t;
        }
    }
    return min_dist;
}

