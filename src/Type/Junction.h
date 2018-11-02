//
// Created by vincent on 18-10-13.
//

#ifndef HDMAP_JUNCTION_H
#define HDMAP_JUNCTION_H

#include <utility>
#include <vector>
#include <map>
#include <memory>
#include "../Math/Curve.h"
#include "Lane.h"
#include "../Math/Bezier.h"
#include "RoadLink.h"
#include "../Interface/IView.h"

namespace hdmap
{
class Junction : public IView, public IXML
{
public:
    static unsigned int JUNCTION_ID;
    unsigned int mJunctionId;
    std::map<std::pair<unsigned int, unsigned int>, RoadLink> mRoadLinks;
    std::vector<std::vector<Pose>> mPoses;

private:
    void GenerateAllPose();

public:
    explicit Junction();

    void AddConnection(unsigned int _from_road_id, int _from_lane_idx, Pose _from_lane_pose,
                       unsigned int _to_road_id, int _to_lane_idx, Pose _to_lane_pose,
                       double _ctrl_len1, double _ctrl_len2);

    std::vector<std::vector<Pose>> GetAllPose();

    bool Check(std::pair<unsigned int, unsigned int> links);

    std::pair<int, int>GetLink(std::pair<unsigned int, unsigned int>RoadPair);

    std::vector<Pose> GetPose(unsigned int from_road_id, int from_lane_idx, unsigned int to_road_id, int to_lane_idx);

    RoadLink GetSubRoadLink (int rid1, int rid2 = -1);
    double Distance(const Vector2d & v);
    void Send(Sender &sender) override;
    boost::property_tree::ptree ToXML() override;
    void FromXML(const pt::ptree &p) override;
};
}
#endif //HDMAP_JUNCTION_H
