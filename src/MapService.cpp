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
#include "Type/Map.h"
#include "Tool/Sender.h"
#include "Math/Bezier.h"
#include "Math/CubicFunction.h"
#include "Tool/Planner.h"


using namespace hdmap;
using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_service");
    if(argc != 2)
    {
        std::cout << "Usage: ./MapService file_name" << std::endl;
        return 1;
    }

    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<visualization_msgs::MarkerArray>("HDMap", 1000);
    shared_ptr<Sender> p_sender(new Sender(pub));

    Map map;
    map.SetSender(p_sender);
    map.Load(argv[1]);
//    map.Summary();

/*
    map.SetStartPoint({-221.360, 11.736});
    map.SetEndPoint({-38.530, 70.328});
    map.GlobalPlanning();
*/

    ROS_INFO("Input 's' to start");

/*
    ros::ServiceServer server = n.advertiseService("local_map", &Map::OnRequest, &map);
    ROS_INFO("HDMap is ready...");
    ros::spin();
*/


    Planner planner(map, p_sender);
    planner.SetStartPoint({-221.360, 11.736});
    planner.SetEndPoint({-38.530, 70.328});
    planner.GlobalPlanning();

    planner.Send();

    char c;
    while (std::cin >> c)
    {
        if(c == 's')
            planner.Send();
        else
            break;
    }
    return 0;
}

