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
    std::shared_ptr<Curve> width(new Line(s, Pose(Lane::DEFAULT_WIDTH, 0),
                           Pose(Lane::DEFAULT_WIDTH, pReferLine->Length())));
    Lane lane(lane_id, width);
    mLanes.insert(std::pair<int, Lane>(lane_idx, lane));
}

std::vector<Pose> LaneSection::GetReferPose()
{
       return pReferLine->GeneratePose(1.0);
}