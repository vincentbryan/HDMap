//
// Created by vincent on 18-10-16.
//

#ifndef HDMAP_ROADLINK_H
#define HDMAP_ROADLINK_H

#include "LaneLink.h"
#include "../Interface/IView.h"
namespace hdmap
{

class SubRoadLink : public IView
{
public:
    unsigned int iFromRoadId;
    unsigned int iToRoadId;
    std::vector<LaneLink> vLaneLinks;
    void Send(Sender &sender) override;
};

class RoadLink
{
public:
    unsigned int iFromRoadId;
    unsigned int iToRoadId;
    std::vector<LaneLink> vLaneLinks;

public:
    explicit RoadLink(unsigned int _from_road_id = 0, unsigned int _to_road_id = 0)
    {
        iFromRoadId = _from_road_id;
        iToRoadId = _to_road_id;
    }

    void AddLaneLink(int _from_lane_idx, int _to_lane_idx, Bezier bezier)
    {
        vLaneLinks.emplace_back(LaneLink(_from_lane_idx, _to_lane_idx, bezier));
    }

    std::vector<Pose> GetPose(int from_lane_idx, int to_lane_idx)
    {
        for(auto x : vLaneLinks)
        {
            if(x.iFromIndex == from_lane_idx && x.iToIndex == to_lane_idx)
            {
                return x.mReferLine.GetAllPose(0.1);
            }
        }
    }

    std::vector<std::vector<Pose>> GetAllPose()
    {
        std::vector<std::vector<Pose>> res;

        for(auto & x : vLaneLinks)
        {
            res.emplace_back(x.mReferLine.GetAllPose(0.1));
        }

        return res;
    }


    SubRoadLink operator ()(int _form_dir, int _to_dir);
};
}

#endif //HDMAP_ROADLINK_H
