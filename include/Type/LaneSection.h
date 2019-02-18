#ifndef HDMAP_LANESECTION_H
#define HDMAP_LANESECTION_H

#include <map>
#include <memory>
#include "Lane.h"
#include "Math/Line.h"
#include "Math/Bezier.h"
#include "Interface/IView.h"
#include "Tool/Sender.h"
#include "Common/pointer_typedef.h"
#include "Interface/IGeometry.h"

namespace hdmap
{
class LaneSection : public IView, public IXML, public IGeometry
{
public:

    double mStartS;  /// start length in current road.

    unsigned int ID = 0;

    Bezier mReferLine;

    std::map<int, Lane> mLanes;

    std::map<int, std::vector<Pose>> mAllLanePose;

    int mLeftBoundary = 0;

    int mRightBoundary = 0;

private:

    void AppendPose(double s_);

    void GenerateRegionPoses() override;

public:
    explicit LaneSection(unsigned int _section_id = 0,
                         double _start_s = 0,  Bezier _refer_line = Bezier());

    void AddLane(int _lane_idx, double _start_offset, double _end_offset, std::vector<int> _pred, std::vector<int> _succ);

    std::vector<Pose> GetReferenceLinePose();

    std::vector<Pose> GetLanePoseByIndex(int _index);

    void OnSend(Sender &sender) override;

    boost::property_tree::ptree ToXML() override;

    void FromXML(const pt::ptree &p) override;
};
}
#endif //HDMAP_LANESECTION_H
