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


    /*
    HDMap map;
    Pose p(0, 0, 0);

    map.AddRoad();
    map.SetStartPose(p);
    std::vector<std::pair<int, bool>> new_lanes = {{-1, true}, {1, true}};
    std::vector<std::pair<int, int>> links;
    map.StartSection(new_lanes, links);

    p.x = 10;
    p.y = 10;
    p.theta = 0;
    map.EndSection(p);

    auto m = map.GetCurrentSection().GetReferPose();
    for(auto x : m)
    {
        std::cout << x.x << " " <<  x.y << " " << x.theta << std::endl;
    }
*/
    return 0;
}
