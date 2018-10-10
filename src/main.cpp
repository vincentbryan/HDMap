//
// Created by vincent on 18-10-7.
//
#include <ros/ros.h>
#include <thread>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose2D.h>
#include <queue>
#include <mutex>
#include "SyncQueue.hpp"
#include "Type/HDMap.h"

using namespace hdmap;

int main( int argc, char** argv )
{
    ros::init(argc, argv, "hdmap");
    ros::NodeHandle n;

    HDMap map;
    Pose p(0, 0, 270);

    map.AddRoad();
    map.SetStartPose(p);

    std::vector<std::pair<int, bool>> new_lanes = {{-2, true}, {-1, true}, {1, true}};
    std::vector<std::pair<int, int>> links;
    map.StartSection(new_lanes, links);

    p.x = 10;
    p.y = 0;
    p.yaw = 270;
    map.EndSection(p);

    auto m = map.GetCurrentSection().GetAllPose();
    for(auto x : m)
    {
        std::cout << "[" << x.first << "]: ";
        for(auto y : x.second)
        {
            std::cout << "(" << y.x << " " << y.y << " " << y.yaw << "), ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;

    return 0;
}
