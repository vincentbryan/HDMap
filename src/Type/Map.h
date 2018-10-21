//
// Created by vincent on 18-10-9.
//

#pragma once

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose2D.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <memory>
#include "Road.h"
#include "Junction.h"
#include "../Tool/Sender.h"
#include "LocalMap.h"
#include "Pose2DArray.h"

namespace hdmap
{
//需要前向声明
class Sender;
class Map
{
private:
    std::vector<Road> mRoads;
    std::vector<Junction> mJunctions;

    Pose mCurrPose;
    Pose mPrevPose;

    LaneSection mPrevSection;
    LaneSection mCurrSection;

    std::vector<std::tuple<int, double, double>>vTempLane;
    std::vector<std::pair<int, int>>vTempLink;

    Vector2d mStartPoint;
    Vector2d mEndPoint;

    unsigned int CalcuSectionId(unsigned int road, unsigned int section);
    unsigned int CalcuLaneId(unsigned int section, int lane);

    std::shared_ptr<Sender> pSender;

public:
    Map();

    //region Setter And Getter
    void SetSender(std::shared_ptr<Sender> _sender);

    void SetCurrPose(const Pose &pose);
    Pose GetCurrPose() const;

    void SetPrevPose(const Pose &pose);
    Pose GetPrevPose() const;

    LaneSection GetCurrentSection() const;
    Junction GetCurrentJunction() const;
    std::vector<LaneSection> GetAllSection();
    std::vector<Junction> GetAllJunction();

    void SetStartPoint(const Vector2d & v);
    Vector2d GetStartPoint() const;
    void SetEndPoint(const Vector2d & v);
    Vector2d GetEndPoint() const;

    //endregion

    void StartRoad(const Pose & _start_pose);
    void EndRoad();
    void EndSection(const Pose & p, double _ctrl_len1 = 1.0, double _ctrl_len2 = 1.0);
    void StartSection(std::vector<std::tuple<int, double, double>> new_lane, std::vector<std::pair<int, int>>links);

    void StartJunction();
    void AddConnection(unsigned int from_road_id, int from_lane_idx,
                       unsigned int to_road_id, int to_lane_idx,
                       double _ctrl_len1 = Bezier::DEFAULT_LENGTH,
                       double _ctrl_len2 = Bezier::DEFAULT_LENGTH);
    void EndJunction();
    void Load(const std::string &file_name);
    void Save(const std::string &file_name);

    void Summary();
    void GlobalPlanning();
    void Send();

    void Trajectory(std::vector<std::pair<unsigned int, int>> sequences);

    void Test();

    bool OnRequest(HDMap::LocalMap::Request &request, HDMap::LocalMap::Response & response);

};
}

