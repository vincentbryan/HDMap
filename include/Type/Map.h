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
#include "Common/pointer_typedef.h"

namespace hdmap
{
//需要前向声明
class Sender;
class Map : public IXML
{

    std::unordered_map<unsigned int, RoadPtr> mRoadIdToPtr;

    std::unordered_map<unsigned int, JuncPtr> mJuncIdToPtr;



public:
    std::vector<RoadPtr> RoadPtrs;

    std::vector<JuncPtr> JuncPtrs;

    std::shared_ptr<Sender> pSender;

    Map();

    void SetSender(std::shared_ptr<Sender> sender);

    RoadPtr AddRoad(const Pose &start_pose);

    JuncPtr AddJunction();

    void AddRoadFromPtr(RoadPtr road);

    void AddJunctionFromPtr(JuncPtr junction);

    unsigned long GetRoadSize();

    unsigned long GetJunctionSize();

    void CommitRoadInfo();

    RoadPtr GetRoadNeighbor(RoadPtr ptr);

    void AddConnection(JuncPtr p,
                       unsigned int from_road_id, int from_lane_idx,
                       unsigned int to_road_id, int to_lane_idx,
                       double _ctrl_len1 = Bezier::DEFAULT_LENGTH,
                       double _ctrl_len2 = Bezier::DEFAULT_LENGTH);

    void AddRoadLink(JuncPtr p,
                     unsigned _from_road_id,
                     unsigned _to_road_id,
                     std::string _direction,
                     std::vector<std::tuple<int, int, double, double>>_lane_links);

    void Load(const std::string &file_name);
    void Save(const std::string &file_name);
    void Clear();

    std::vector<RoadPtr> AdjacentRoadInfo(RoadPtr p_road);

    void Send();

    boost::property_tree::ptree ToXML() override;

    void FromXML(const pt::ptree &p) override;

    RoadPtr GetRoadPtrById(unsigned int road_id);

    JuncPtr GetJuncPtrById(unsigned int junc_id);

    std::vector<RoadPtr> GetRoadPtrByDistance(const Coor& coor, double distance, bool keep_one = false);
    
    std::vector<JuncPtr> GetJuncPtrByDistance(const Coor& coor, double distance, bool keep_one = false);

    std::tuple<RoadPtr, SecPtr, int> GetLaneInfoByPose(const Pose &pose);

    std::tuple<JuncPtr, RoadLink, int> GetRoadLinkByPose(const Pose& pose);

};
}

