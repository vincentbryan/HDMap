//
// Created by vincent on 18-10-7.
//
#include <ros/ros.h>
#include <thread>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose2D.h>
#include <queue>
#include <mutex>
#include "SyncQueue.hpp"
#include "Type/HDMap.h"
#include "Sender.h"
//#define DEBUG
using namespace hdmap;

int main( int argc, char** argv )
{
    ros::init(argc, argv, "hdmap");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("HDMap", 10);

#ifdef DEBUG
#else
    HDMap map;
    Pose p(0, 0, 315);

    map.AddRoad();
    map.SetStartPose(p);

    //-------------------------------------------------------------
    std::vector<std::pair<int, bool>> lanes1 = {{-2, true}, {-1, true}, {1, true}};
    std::vector<std::pair<int, int>> links1;
    map.StartSection(lanes1, links1);

    p.x = 5;
    p.y = 5;
    p.yaw = 315;
    map.EndSection(p);
    ros::Rate r(1);

    char c;
    while (std::cin >> c)
    {
        if(c != 'e')
            Sender::SendSection(map.GetCurrentSection(), marker_pub);
        else
            break;
    }
    std::cout << "Sec 1" << std::endl;
    //-------------------------------------------------------------
    std::vector<std::pair<int, bool>> lanes2 = {{-1, true}, {1, true}};
    std::vector<std::pair<int, int>> links2;
    map.StartSection(lanes2, links2);

    p.x = 10;
    p.y = 10;
    p.yaw = 315;
    map.EndSection(p);

    while (std::cin >> c)
    {
        if(c != 'e')
            Sender::SendSection(map.GetCurrentSection(), marker_pub);
        else
            break;
    }
    std::cout << "Sec 2" << std::endl;
    return 0;
#endif
}


