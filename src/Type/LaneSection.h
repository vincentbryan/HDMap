//
// Created by vincent on 18-10-8.
//

#ifndef HDMAP_LANESECTION_H
#define HDMAP_LANESECTION_H

#include <map>
#include <memory>
#include "Lane.h"
#include "Line.h"
namespace hdmap
{
class LaneSection
{
public:
    double s;
    unsigned int iSectionId = 0;

    std::shared_ptr<Curve> pReferLine;
    std::shared_ptr<Curve> pLaneOffset;

    std::map<int, Lane>mLanes;
    std::map<int, std::vector<Pose>> mAllLanePose;

    int most_left_lane_idx = 0;
    int most_right_lane_idx = 0;

    std::vector<Pose> GetReferPose();

public:
    explicit LaneSection(unsigned int section_id = 0, double _s = 0,
                         std::shared_ptr<Curve> p_refer_line = nullptr,
                         std::shared_ptr<Curve> p_lane_offset = nullptr);

    void AddLane(int lane_idx, unsigned int lane_id, bool is_constant_width);

    std::map<int, std::vector<Pose>> GetAllPose();

    void Clear()
    {
        most_left_lane_idx = most_right_lane_idx = 0;
        mLanes.clear();
        mAllLanePose.clear();
    };

private:
    void AppendPose(double s_);
    void GenerateAllPose(double ds);

};
}
#endif //HDMAP_LANESECTION_H
