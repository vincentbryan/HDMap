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
#include "Tool/Planner.h"

using namespace hdmap;
using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_service");
    if(argc != 2)
    {
        std::cout << "Usage: ./MapService input_map" << std::endl;
        return 1;
    }

    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<visualization_msgs::MarkerArray>("HDMap", 1000);
    shared_ptr<Sender> p_sender(new Sender(pub));

    Map map;
    map.SetSender(p_sender);
    map.Load(argv[1]);
    Planner planner(map, p_sender);

    ros::ServiceServer server = n.advertiseService("map_service", &Planner::OnRequest, &planner);

    ROS_INFO("MapService is ready...");

    ros::spin();

    return 0;
}

