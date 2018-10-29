//
// Created by vincent on 18-10-8.
//

#include "LaneSection.h"
#include <algorithm>
#include "../Math/Line.h"
#include "common.h"

using namespace hdmap;

LaneSection::LaneSection(unsigned int section_id, double _s,
                         Bezier refer_line,
                         CubicFunction lane_offset)
{
    iSectionId = section_id;
    s = _s;
    mReferLine = refer_line;
    mLaneOffset = lane_offset;
}

void LaneSection::AddLane(int lane_idx, double _start_width, double _end_width, std::vector<int> _pred, std::vector<int> _succ)
{
    if(lane_idx > 0) most_right_lane_idx = std::max(most_right_lane_idx, lane_idx);
    if(lane_idx < 0) most_left_lane_idx = std::min(most_left_lane_idx, lane_idx);

    int lane_id = iSectionId * 10 + 5 + lane_idx;
    CubicFunction width(_start_width, mReferLine.Length(), _end_width);
    Lane lane(lane_id, width);
    lane.predecessors = std::move(_pred);
    lane.successors = std::move(_succ);

    mLanes.insert(std::pair<int, Lane>(lane_idx, lane));
}

std::vector<Pose> LaneSection::GetReferPose()
{
    if(mAllLanePose.empty())
        GenerateAllPose(0.1);
    return mAllLanePose[0];
}

void LaneSection::GenerateAllPose(double ds)
{
    for(auto x : mLanes)
    {
        mAllLanePose[x.first] = std::vector<Pose>();
    }

    double s_ = 0;
    double len = mReferLine.Length();

    while (true)
    {
        AppendPose(s_);
        s_ += ds;
        if(s_ >= len)
        {
            AppendPose(len);
            break;
        }
    }
}

void LaneSection::AppendPose(double s_)
{
    Pose refer_pose = mReferLine.GetPose(s_);
    mAllLanePose[0].emplace_back(refer_pose);
    double width = 0;
    int idx = 0;
    for(idx = 1; idx <= most_right_lane_idx; idx++)
    {

        Angle angle = refer_pose.GetAngle();
        angle.Rotate(-90.0);

        double w = mLanes[idx].width.Value(s_);
        Pose t = refer_pose.GetTranslation(w, angle);
        mAllLanePose[idx].emplace_back(t);
    }

    for(idx = -1; idx >= most_left_lane_idx; idx--)
    {
        Angle angle = refer_pose.GetAngle();
        angle.Rotate(90.0);

        double w = mLanes[idx].width.Value(s_);
        Pose t = refer_pose.GetTranslation(w, angle);

        mAllLanePose[idx].emplace_back(t);
    }
}

std::map<int, std::vector<Pose>> LaneSection::GetAllPose()
{
    if(mAllLanePose.empty())
        GenerateAllPose(0.1);
    return mAllLanePose;
}

void LaneSection::Send(Sender &sender)
{
    if(mAllLanePose.empty()) GenerateAllPose(0.1);

    for(auto & x : mAllLanePose)
    {
        if(x.first == 0)
        {
            auto refer_line = sender.GetLineStrip(x.second, 199.0/255, 166.0/255, 33.0/255, 1.0);
            sender.array.markers.push_back(refer_line);
        }
        else
        {
            auto solid_line = sender.GetLineStrip(x.second, 0.7, 0.7, 0.7, 0.3);
            sender.array.markers.push_back(solid_line);
//            double w = std::max(mLanes[x.first].width.y0, mLanes[x.first].width.y1);
//            auto poses = sender.Translate(x.second, w/2, -90.0);
//            auto solid_line = sender.GetLineStrip(poses, 0.7, 0.7, 0.7, 0.3);
//            sender.array.markers.push_back(solid_line);
        }
        auto lane_idx = sender.GetText(std::to_string(x.first), x.second[x.second.size()/2].GetPosition());
        sender.array.markers.push_back(lane_idx);
    }
    sender.Send();
}

SecPtr LaneSection::GetSubSection(int direction)
{
    SecPtr res(new LaneSection());
    res->iSectionId = iSectionId;
    res->mReferLine = mReferLine;
    res->mLaneOffset = mLaneOffset;
    res->s = s;
    if(direction > 0)
    {
        res->most_right_lane_idx = most_right_lane_idx;
        res->most_left_lane_idx = 0;
        for(auto & x : mLanes)
        {
            if(x.first > 0)
                res->mLanes.insert(x);
        }
    }
    else
    {
        res->most_left_lane_idx = most_left_lane_idx;
        res->most_right_lane_idx = 0;
        for(auto & x : mLanes)
        {
            if(x.first < 0)
                res->mLanes.insert(x);
        }
    }
    return res;
}

