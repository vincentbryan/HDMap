//
// Created by vincent on 18-10-8.
//

#include "LaneSection.h"
#include <algorithm>
#include "../Math/Line.h"

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

void LaneSection::AddLane(int lane_idx, unsigned int lane_id, double _start_width, double _end_width)
{
    if(lane_idx > 0) most_right_lane_idx = std::max(most_right_lane_idx, lane_idx);
    if(lane_idx < 0) most_left_lane_idx = std::min(most_left_lane_idx, lane_idx);

    CubicFunction width(_start_width, mReferLine.Length(), _end_width);
    Lane lane(lane_id, width);
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

    for(auto & x : mAllLanePose)
    {
        if(x.first < 0)
        {
            std::reverse(x.second.begin(), x.second.end());
            for(auto & y : x.second)
            {
                y.Rotate(180.0);
            }
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
        double m = 0;
        if(w < Lane::DEFAULT_WIDTH)
            m = width + w - Lane::DEFAULT_WIDTH/2;
        else
            m = width + w/2;

        Pose t = refer_pose.GetTranslation(m, angle);
        mAllLanePose[idx].emplace_back(t);

        width += w;
    }

    width = 0;
    for(idx = -1; idx >= most_left_lane_idx; idx--)
    {

        Angle angle = refer_pose.GetAngle();
        angle.Rotate(90.0);

        double w = mLanes[idx].width.Value(s_);
        double m = 0;
        if(w < Lane::DEFAULT_WIDTH)
            m = width + w - Lane::DEFAULT_WIDTH/2;
        else
            m = width + w/2;

        Pose t = refer_pose.GetTranslation(m, angle);

        mAllLanePose[idx].emplace_back(t);
        width += w;
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
            auto track = sender.GetLineStrip(x.second, 95.0/255, 217.0/255, 205.0/255, 1.0);
            sender.array.markers.push_back(track);

            double w = std::max(mLanes[x.first].width.y0, mLanes[x.first].width.y1);
            auto poses = sender.Translate(x.second, w/2, -90.0);
            auto solid_line = sender.GetLineStrip(poses, 0.7, 0.7, 0.7, 0.3);
            sender.array.markers.push_back(solid_line);
        }
        auto lane_idx = sender.GetText(std::to_string(x.first), x.second[x.second.size()/2].GetPosition());
        sender.array.markers.push_back(lane_idx);
    }
    sender.Send();
}

LaneSection LaneSection::GetSubSection(int direction)
{
    LaneSection res;
    res.iSectionId = iSectionId;
    if(direction > 0)
    {
        res.mReferLine = mReferLine;
        res.mLaneOffset = mLaneOffset;
        res.most_right_lane_idx = most_right_lane_idx;
        res.most_left_lane_idx = 0;
        for(auto & x : mLanes)
        {
            if(x.first > 0)
                res.mLanes.insert(x);
        }
    }
    else
    {
        Pose s = mReferLine.GetStartPose();
        Pose e = mReferLine.GetEndPose();
        s.Rotate(180);
        e.Rotate(180);
        res.mReferLine = Bezier(e, s, mReferLine.ctrl_len2, mReferLine.ctrl_len1);
        res.most_left_lane_idx = 0;
        res.most_right_lane_idx = std::abs(most_left_lane_idx);
        for(auto & x : mLanes)
        {
            if(x.first < 0)
            {
                CubicFunction width(x.second.width.y1,
                                    res.mReferLine.Length(),
                                    x.second.width.y0);
                Lane lane(x.second.land_id, width);

                for(auto & y : x.second.predecessors) lane.successors.emplace_back(y);
                for(auto & y : x.second.successors) lane.predecessors.emplace_back(y);

                res.mLanes.insert({std::abs(x.first), lane});
            }
        }
    }
    return res;
}

