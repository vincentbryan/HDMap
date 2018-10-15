//
// Created by vincent on 18-10-9.
//

#ifndef HDMAP_HDMAP_H
#define HDMAP_HDMAP_H

#include <geometry_msgs/Pose2D.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include "Road.h"
#include "Junction.h"

namespace hdmap
{
class HDMap
{
private:
    std::vector<Road> mRoads;
    std::vector<Junction> mJunctions;

    Pose mStartPose;
    Pose mEndPose;

    LaneSection mPrevSection;
    LaneSection mCurrSection;

    std::vector<std::tuple<int, double, double>>vTempLane;
    std::vector<std::pair<int, int>>vTempLink;

    unsigned int CalcuSectionId(unsigned int road, unsigned int section);
    unsigned int CalcuLaneId(unsigned int section, int lane);

public:
    HDMap();

    void SetStartPose(const Pose &pose) {mStartPose = pose;};
    Pose GetStartPose(){return mStartPose; };

    void SetEndPose(const Pose &pose){mEndPose = pose; };
    Pose GetEndPose(){return mEndPose; };

    void AddRoad();

    void EndSection(Pose p);
    void StartSection(std::vector<std::tuple<int, double, double>> new_lane, std::vector<std::pair<int, int>>links);

    LaneSection GetCurrentSection(){return mCurrSection;}
    Junction GetCurrentJunction(){return mJunctions.back();};

    std::vector<LaneSection> GetAllSection();
    std::vector<Junction> GetAllJunction();

    void AddJunction();
    void AddConnection(unsigned int from_road, int from_lane_idx,
                       unsigned int to_road, int to_lane_idx);

//    void Load(const std::string &file_name);
//    void Save(const std::string &file_name);
};
}

#endif //HDMAP_HDMAP_H
