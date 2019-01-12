#ifndef HDMAP_JUNCTION_H
#define HDMAP_JUNCTION_H

#include <utility>
#include <vector>
#include <map>
#include <memory>
#include "Math/Curve.h"
#include "Lane.h"
#include "Math/Bezier.h"
#include "RoadLink.h"
#include "Interface/IView.h"
#include "Interface/IGeometry.h"

namespace hdmap
{
class Junction : public IView, public IXML, public IGeometry
{
public:

    unsigned int ID;

    static unsigned int JUNCTION_ID;

    std::map<std::pair<unsigned int, unsigned int>, RoadLink> RoadLinks;

private:

    MapPtr mMapInstantPtr;

    std::vector<hdmap::Bezier> mBoundaryCurves;

    void GenerateRegionPoses() override;

public:

    explicit Junction(MapPtr ptr = nullptr);

    void AddConnection(unsigned int _from_road_id, int _from_lane_idx, Pose _from_lane_pose,
                       unsigned int _to_road_id, int _to_lane_idx, Pose _to_lane_pose,
                       double _ctrl_len1, double _ctrl_len2);

    std::vector<hdmap::Bezier> GetBoundaryCurves();

    void OnSend(Sender &sender) override;

    void FromXML(const pt::ptree &p) override;

    boost::property_tree::ptree ToXML() override;

    RoadLink GetRoadLink(int rid1, int rid2 = -1);

};
}
#endif //HDMAP_JUNCTION_H
