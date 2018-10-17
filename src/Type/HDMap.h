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

    Pose mCurrPose;
    Pose mPrevPose;

    LaneSection mPrevSection;
    LaneSection mCurrSection;

    std::vector<std::tuple<int, double, double>>vTempLane;
    std::vector<std::pair<int, int>>vTempLink;

    unsigned int CalcuSectionId(unsigned int road, unsigned int section);
    unsigned int CalcuLaneId(unsigned int section, int lane);

    Vector2d mStartPoint;
    Vector2d mEndPoint;

public:
    HDMap();

    void SetCurrPose(const Pose &pose) {mCurrPose = pose;};
    Pose GetCurrPose(){return mCurrPose; };

    void SetEndPose(const Pose &pose){mPrevPose = pose; };
    Pose GetEndPose(){return mPrevPose; };

    unsigned int GetCurrentRoadId()
    {
        assert(!mRoads.empty());
        return mRoads.back().iRoadId;
    }

    void StartRoad();
    void EndRoad();
    void EndSection(Pose p);
    void StartSection(std::vector<std::tuple<int, double, double>> new_lane, std::vector<std::pair<int, int>>links);

    LaneSection GetCurrentSection(){return mCurrSection;}
    Junction GetCurrentJunction(){return mJunctions.back();};

    std::vector<LaneSection> GetAllSection();
    std::vector<Junction> GetAllJunction();

    void AddJunction();
    void AddConnection(unsigned int from_road_id, int from_lane_idx,
                       unsigned int to_road_id, int to_lane_idx,
                       double _ctrl_len1 = Bezier::DEFAULT_LENGTH,
                       double _ctrl_len2 = Bezier::DEFAULT_LENGTH);

    void Load(const std::string &file_name);
//    void Save(const std::string &file_name);

    void Summary();

    void SetStartPoint(const Vector2d & v)
    {
        mStartPoint = v;
    }

    Vector2d GetStartPoint() const
    {
        return mStartPoint;
    }

    void SetEndPoint(const Vector2d & v)
    {
        mEndPoint = v;
    }

    Vector2d GetEndPoint() const
    {
        return mEndPoint;
    }

    void GlobalPlanning();
};
}

#endif //HDMAP_HDMAP_H
