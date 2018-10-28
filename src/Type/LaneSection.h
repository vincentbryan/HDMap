//
// Created by vincent on 18-10-8.
//

#ifndef HDMAP_LANESECTION_H
#define HDMAP_LANESECTION_H

#include <map>
#include <memory>
#include "Lane.h"
#include "../Math/Line.h"
#include "../Math/Bezier.h"
#include "../Interface/IView.h"
#include "../Tool/Sender.h"
#include "common.h"

namespace hdmap
{
class LaneSection : public IView
{
public:
    double s;
    unsigned int iSectionId = 0;

    Bezier mReferLine;
    CubicFunction mLaneOffset;

    std::map<int, Lane>mLanes;
    std::map<int, std::vector<Pose>> mAllLanePose;

    int most_left_lane_idx = 0;
    int most_right_lane_idx = 0;

    std::vector<Pose> GetReferPose();

public:
    explicit LaneSection(unsigned int section_id = 0, double _s = 0,
                         Bezier refer_line = Bezier(),
                         CubicFunction lane_offset = CubicFunction());

    void AddLane(int lane_idx, double _start_width, double _end_width, std::vector<int> _pred, std::vector<int> _succ);

    std::map<int, std::vector<Pose>> GetAllPose();

    void Clear()
    {
        most_left_lane_idx = most_right_lane_idx = 0;
        mLanes.clear();
        mAllLanePose.clear();
    };

    std::map<int, Lane> GetLanes()
    {
        return mLanes;
    };

    Lane GetLaneByIndex(int idx)
    {
        return mLanes[idx];
    }

    std::vector<Pose> GetLanePoseByIndex(int idx)
    {
        if(mAllLanePose.empty())
            GetAllPose();
        return mAllLanePose[idx];
    }

    void Send(Sender &sender) override;

    SecPtr GetSubSection(int direction);

private:
    void AppendPose(double s_);
    void GenerateAllPose(double ds);
};
}
#endif //HDMAP_LANESECTION_H
