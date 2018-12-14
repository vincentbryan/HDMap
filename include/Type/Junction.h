//
// Created by vincent on 18-10-13.
//

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
#include "kdtree.hpp"

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

    std::vector<std::vector<double>> mKdtreeData;

    kt::kdtree<double> mKdtree;

    std::vector<Pose> mRegionPoses;

    std::vector<hdmap::Bezier> mBoundaryCurves;

    void GenerateRegionPoses() override;

public:

    explicit Junction(MapPtr ptr = nullptr);

    void AddConnection(unsigned int _from_road_id, int _from_lane_idx, Pose _from_lane_pose,
                       unsigned int _to_road_id, int _to_lane_idx, Pose _to_lane_pose,
                       double _ctrl_len1, double _ctrl_len2);


    void Send(Sender &sender) override;

    void FromXML(const pt::ptree &p) override;

    bool Cover(const Coor &v) override;

    std::vector<Pose> GetRegionPoses() override;

    std::vector<hdmap::Bezier> GetBoundaryCurves();

    boost::property_tree::ptree ToXML() override;

    RoadLink GetRoadLink(int rid1, int rid2 = -1);

    double GetDistanceFromCoor(const Coor &v);

};
}
#endif //HDMAP_JUNCTION_H
