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
class LaneSection : public IView, public IXML
{
public:
    double mStartS;
    unsigned int mSectionId = 0;

    Bezier mReferLine;
    CubicFunction mLaneOffset;

    std::map<int, Lane>mLanes;
    std::map<int, std::vector<Pose>> mAllLanePose;

    int mLeftBoundary = 0;
    int mRightBoundary = 0;

private:
    void AppendPose(double s_);
    void GenerateAllPose(double ds);

public:
    explicit LaneSection(unsigned int _section_id = 0,
                         double _start_s = 0,
                         Bezier _refer_line = Bezier(),
                         CubicFunction _lane_offset = CubicFunction());

    void AddLane(int _lane_idx, double _start_offset, double _end_offset, std::vector<int> _pred, std::vector<int> _succ);

    std::map<int, std::vector<Pose>> GetAllPose();
    std::vector<Pose> GetReferPose();

    std::map<int, Lane> GetLanes() { return mLanes; };

    Lane GetLaneByIndex(int idx) { return mLanes[idx]; }

    std::vector<Pose> GetLanePoseByIndex(int _index);
//    SecPtr GetSubSection(int direction);
    double Distance(const Vector2d &v);

    void Send(Sender &sender) override;
    boost::property_tree::ptree ToXML() override;
    void FromXML(const pt::ptree &p) override;
};
}
#endif //HDMAP_LANESECTION_H
