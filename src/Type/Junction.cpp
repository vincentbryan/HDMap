#include <utility>

//
// Created by vincent on 18-10-13.
//

#include <Type/Junction.h>

#include "Type/Map.h"

using namespace hdmap;

unsigned int Junction::JUNCTION_ID = 0;

Junction::Junction(MapPtr ptr) {
    ID = JUNCTION_ID++;
    mMapInstantPtr = std::move(ptr);
}

void Junction::AddConnection(unsigned int _from_road_id, int _from_lane_idx, Pose _from_lane_pose,
                             unsigned int _to_road_id, int _to_lane_idx, Pose _to_lane_pose,
                             double _ctrl_len1, double _ctrl_len2)
{
    std::pair<unsigned int, unsigned int> p(_from_road_id, _to_road_id);
    if (RoadLinks.find(p) != RoadLinks.end())
    {
        RoadLinks[p].AddLaneLink(_from_lane_idx,
                                 _to_lane_idx,
                                 Bezier(_from_lane_pose, _to_lane_pose, _ctrl_len1, _ctrl_len2));
    }
    else
    {
        RoadLink road_link(_from_road_id, _to_road_id);
        road_link.AddLaneLink(_from_lane_idx,
                              _to_lane_idx,
                              Bezier(_from_lane_pose, _to_lane_pose, _ctrl_len1, _ctrl_len2));
        RoadLinks[p] = road_link;
    }
}


void Junction::Send(Sender &sender)
{
    if (mRegionPoses.empty()) GenerateRegionPoses();

    std::string text = "Junction[" + std::to_string(ID) + "]";

    Coor center_coor;
    for (auto &b: mBoundaryCurves)
    {
        center_coor += b.GetStartPose().GetPosition();
        center_coor += b.GetEndPose().GetPosition();
    }
    center_coor /= 2 * mBoundaryCurves.size();
    sender.array.markers.emplace_back(sender.GetText(text, center_coor));


    // region boundary
    visualization_msgs::Marker m;
    std::vector<Pose> _region_boundary;
    for (auto _bezire: mBoundaryCurves)
    {
        auto it = _bezire.GetPoses(0.5);
        _region_boundary.insert(_region_boundary.end(), it.begin(), it.end());
    }

    m = sender.GetLineStrip(_region_boundary, 180.0 / 255.0, 76.0 / 255.0, 231.0 / 255.0, 1.0, 0, 0.15);
    sender.array.markers.emplace_back(m);


    sender.Send();
    for (auto &x : RoadLinks)
    {
        x.second.Send(sender);
    }
}

RoadLink Junction::GetRoadLink (int rid1, int rid2)
{
    auto it = RoadLinks.find(std::pair<unsigned int, unsigned int>(rid1, rid2));
    return it->second;
}

boost::property_tree::ptree Junction::ToXML()
{
    pt::ptree p_junc;
    p_junc.add("<xmlattr>.id", ID);

    if (mBoundaryCurves.empty()) GenerateRegionPoses();

    pt::ptree p_vec;
    pt::ptree p_v;

    for (auto &bezier: mBoundaryCurves)
    {
        for (auto &p : bezier.GetParam())
            p_v.add("param", p);
        p_vec.add_child("bezier", p_v);
        p_v.clear();
    }
    p_junc.add_child("regionBoundary", p_vec);


    for (auto &r : RoadLinks)
    {
        p_junc.add_child("roadLink", r.second.ToXML());
    }

    return p_junc;
}

void Junction::FromXML(const pt::ptree &p)
{

    for(auto & n : p.get_child(""))
    {
        if(n.first == "<xmlattr>")
        {
            ID = n.second.get<int>("id");
        }

        if(n.first == "roadLink")
        {
            RoadLink r;
            r.FromXML(n.second);
            RoadLinks[{r.mFromRoadId, r.mToRoadId}] = r;
        }

        if (n.first == "regionBoundary")
        {
            pt::ptree p_vec = n.second;

            for (auto &p_bezier : p_vec.get_child(""))
            {
                std::vector<double> res;
                for (auto &param: p_bezier.second.get_child(""))
                {
                    res.emplace_back(std::atof(param.second.data().c_str()));
                }
                mBoundaryCurves.emplace_back(res);
            }
            GenerateRegionPoses();
        }
    }
}

double Junction::GetDistanceFromCoor(const Coor &v) {
    if (mRegionPoses.empty()) GenerateRegionPoses();

    assert(!mRegionPoses.empty());
    std::vector<int> indices;
    std::vector<double> distances;
    mKdtree.NearestSearch({v.x, v.y}, indices, distances, 1);
    return distances[0];
}


void Junction::GenerateRegionPoses() {

    if (!mRegionPoses.empty()) return;

    if (mBoundaryCurves.empty()) {

        if (mMapInstantPtr == nullptr) {
            printf("Junction should be initialized with Map pointer which contains information of roads.");
        }

        // acquire poses
        std::vector<std::pair<Pose, int>> _vec;
        std::set<unsigned int> _to_id_set, _from_id_set;
        Coor center_point;
        for (const auto &_link: RoadLinks) {
            unsigned int _id_from = _link.second.mFromRoadId;
            unsigned int _id_to = _link.second.mToRoadId;
            if (_to_id_set.count(_id_to) == 0) {
                _to_id_set.insert(_id_to);
                RoadPtr to_road_ptr = mMapInstantPtr->GetRoadPtrById(_id_to);
                if (to_road_ptr != nullptr) {
                    Pose _p_to = mMapInstantPtr->GetRoadPtrById(_id_to)->mSecPtrs.front()->GetLanePoseByIndex(
                            -1).front();
                    center_point += _p_to.GetPosition();
                    _vec.emplace_back(_p_to, _id_to);
                }
            }
            if (_from_id_set.count(_id_from) == 0) {
                _from_id_set.insert(_id_from);
                RoadPtr from_road_ptr = mMapInstantPtr->GetRoadPtrById(_id_from);
                if (from_road_ptr != nullptr) {
                    Pose _p_from = mMapInstantPtr->GetRoadPtrById(_id_from)->mSecPtrs.back()->GetLanePoseByIndex(
                            -1).back();
                    center_point += _p_from.GetPosition();
                    _vec.emplace_back(_p_from, _id_from);
                }
            }
        }

        if (_vec.empty())
        {
            printf("GenerateRegionPoses Fail\n");
            return;
        }

        center_point /= _vec.size();

        // re-range position of element of _vec according to Pose, clockwise.
        sort(_vec.begin(), _vec.end(), [center_point]
                (std::pair<Pose, int> &o1, std::pair<Pose, int> &o2) -> bool {
            Angle aa(o1.first.GetPosition() - center_point);
            Angle ab(o2.first.GetPosition() - center_point);
            return aa.Value() > ab.Value();
        });

        for (int i = 0; i < _vec.size(); ++i)
        {
            int s = i == 0 ? _vec.size() - 1 : i - 1;
            int e = i;

            Pose _bezier_sta_p = _vec[s].first;
            Pose _bezier_end_p = _vec[e].first;
            double dis = _bezier_sta_p.DistanceFrom(_bezier_end_p);

            //  (_vec[s].second|1) == (_vec[e].second|1)
            auto road_1 = mMapInstantPtr->GetRoadPtrById(_vec[e].second);
            auto road_2 = mMapInstantPtr->GetRoadPtrById(_vec[s].second);

            if (mMapInstantPtr->GetRoadNeighbor(road_1)->ID == road_2->ID) {
                Angle _angle(_bezier_end_p.GetPosition() - _bezier_sta_p.GetPosition());
                _bezier_sta_p.GetAngle().SetAngle(_angle.Value());
                _bezier_end_p.GetAngle().SetAngle(_angle.Value());
                mBoundaryCurves.emplace_back(_bezier_sta_p, _bezier_end_p, dis / 2, dis / 2);
            } else {
                _bezier_sta_p.GetAngle().Rotate(180);
                _bezier_end_p.GetAngle().Rotate(180);
                mBoundaryCurves.emplace_back(_bezier_sta_p, _bezier_end_p, dis / 2, dis / 2);
            }
        }
    }


    // get all pose points from bezier curves.
    assert(!mBoundaryCurves.empty());
    for (auto &b: mBoundaryCurves) {
        const auto &_tmp = b.GetPoses(1);
        mRegionPoses.insert(mRegionPoses.end(), _tmp.begin(), _tmp.end());
    }

    // store kdtree catch for distance search
    mKdtreeData.clear();
    for (auto &p: mRegionPoses) {
        std::vector<double> _tmp{p.x, p.y};
        mKdtreeData.emplace_back(_tmp);
    }
    mKdtree.SetData(mKdtreeData, kt::VARIANCE);

}


bool Junction::Cover(const Coor &v)
{
    if (mBoundaryCurves.empty()) {
        GenerateRegionPoses();
    }

    return IGeometry::Cover(mRegionPoses, {v});
}

std::vector<Pose> Junction::GetRegionPoses() {
    if (mRegionPoses.empty()) {
        GenerateRegionPoses();
    }
    return mRegionPoses;
}

std::vector<hdmap::Bezier> Junction::GetBoundaryCurves() {
    if (mBoundaryCurves.empty()) {
        GenerateRegionPoses();
    }
    return mBoundaryCurves;
}

