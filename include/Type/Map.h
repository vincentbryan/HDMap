//
// Created by vincent on 18-10-9.
//

#pragma once

#include <unordered_map>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose2D.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <memory>
#include "Road.h"
#include "Junction.h"
#include "Tool/Sender.h"
#include "../../src/Type/common.h"

namespace hdmap
{
//需要前向声明
class Sender;
class Map : public IXML
{
public:
    std::vector<RoadPtr> mRoadPtrs;
    std::vector<JuncPtr> mJuncPtrs;
    std::shared_ptr<Sender> pSender;

public:
    Map();
    void SetSender(std::shared_ptr<Sender> _sender);

    RoadPtr AddRoad(const Pose & _start_pose);
    void CommitRoadInfo();

    JuncPtr AddJunction();
    void AddConnection(JuncPtr p,
                       unsigned int from_road_id, int from_lane_idx,
                       unsigned int to_road_id, int to_lane_idx,
                       double _ctrl_len1 = Bezier::DEFAULT_LENGTH,
                       double _ctrl_len2 = Bezier::DEFAULT_LENGTH);

    void Load(const std::string &file_name);
    void Save(const std::string &file_name);
    void Clear();

    std::vector<RoadPtr> AdjacentRoadInfo(RoadPtr p_road);
//    void Trajectory(std::vector<std::pair<unsigned int, int>> sequences);

    RoadPtr Locate(const Vector2d & v);

    void Send();
    boost::property_tree::ptree ToXML() override;
    void FromXML(const pt::ptree &p) override;

    RoadPtr GetRoadPtrById(unsigned int _road_id);
    JuncPtr GetJuncPtrById(unsigned int _junc_id);
};
}

