//
// Created by vincent on 18-10-8.
//

#include "LaneSection.h"
#include "Line.h"

using namespace hdmap;

LaneSection::LaneSection(unsigned int section_id, double _s,
                         std::shared_ptr<Curve> p_refer_line,
                         std::shared_ptr<Curve> p_lane_offset)
{
    iSectionId = section_id;
    s = _s;

    if(p_refer_line == nullptr)
        pReferLine.reset(new Line(0, Pose(), Pose()));
    else
        pReferLine = p_lane_offset;

    if(p_lane_offset == nullptr)
        pLaneOffset.reset(new Line(0, Pose(), Pose(pReferLine->Length(), 0)));
    else
        pLaneOffset = p_lane_offset;

}

void LaneSection::AddLane(int lane_idx, unsigned int lane_id, bool is_constant_width)
{
    if(lane_idx > 0) most_rigjt_lane_idx = std::max(most_rigjt_lane_idx, lane_idx);
    if(lane_idx < 0) most_left_lane_idx = std::min(most_left_lane_idx, lane_idx);

    std::shared_ptr<Curve> width(new Line(s, Pose(0, Lane::DEFAULT_WIDTH),
                           Pose(pReferLine->Length(), Lane::DEFAULT_WIDTH)));
    Lane lane(lane_id, width);
    mLanes.insert(std::pair<int, Lane>(lane_idx, lane));
}

std::vector<Pose> LaneSection::GetReferPose()
{
       return mAllLanePose[0];
}

void LaneSection::GenerateAllPose(double ds)
{
    for(auto x : mLanes)
    {
        mAllLanePose[x.first] = std::vector<Pose>();
    }

    double s_ = 0;
    double len = pReferLine->Length();

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
    Pose refer_pose = pReferLine->GetPose(s_);
    mAllLanePose[0].emplace_back(refer_pose);
    double width = 0;
    int idx = 0;
    for(idx = 1; idx <= most_rigjt_lane_idx; idx++)
    {
        Lane t = mLanes[idx];
        Pose t1 = mLanes[idx].pWidth->GetPose(s_);
        Pose t2 = refer_pose.GetTranslation(width+t1.y/2, refer_pose.yaw);
        mAllLanePose[idx].emplace_back(t2);
        width += t1.y;
    }

    width = 0;
    for(idx = -1; idx >= most_left_lane_idx; idx--)
    {
        auto t1 =  mLanes[idx].pWidth->GetPose(s_);
        auto t2 = refer_pose.GetTranslation(width+t1.y/2, refer_pose.yaw+180.0);
        mAllLanePose[idx].emplace_back(t2);
        width += t1.y;
    }
}

std::map<int, std::vector<Pose>> LaneSection::GetAllPose()
{
    if(mAllLanePose.empty())
        GenerateAllPose(0.1);
    return mAllLanePose;
}

