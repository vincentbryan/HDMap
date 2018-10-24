//
// Created by vincent on 18-10-13.
//

#include "Junction.h"
#include <algorithm>
#include "../Math/Line.h"

using namespace hdmap;

unsigned int Junction::JUNCTION_ID = 0;

Junction::Junction()
{
    iJunctionId = JUNCTION_ID++;
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
    if(vPoses.empty())
        GenerateAllPose();

    return vPoses;
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
    if(vPoses.empty())
        GenerateAllPose();

    for(auto & x : vPoses)
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
        for(auto y : x.second.vLaneLinks)
        {
            vPoses.emplace_back(y.mReferLine.GetAllPose(0.1));
        }
    }
}

