//
// Created by vincent on 18-10-7.
//
#include <ros/ros.h>
#include <thread>
#include <visualization_msgs/MarkerArray.h>
#include "Type/Map.h"
#include "Tool/Planner.h"
#include "Tool/Resource.h"

using namespace hdmap;
using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_service");
    ros::NodeHandle n("~");

    std::string route_net_path;
    n.param<std::string>("route_net_path", route_net_path, "");
    if(route_net_path.empty())
    {
        ROS_ERROR("[ Map Service ] route_net_path is empty.");
        return -1;
    } else{
        ROS_INFO("[ Map Service ] route_net_path: %s", route_net_path.c_str());
    }

    ros::Publisher pub = n.advertise<visualization_msgs::MarkerArray>("/HDMap", 100);
    shared_ptr<Sender> p_sender(new Sender(pub));

    Resource resource(route_net_path);
    resource.SetSender(p_sender);

    Planner planner(resource.GetMap(), p_sender);

    ros::ServiceServer plan_server = n.advertiseService("/map_plan_service", &Planner::OnRequest, &planner);
    ros::ServiceServer data_server = n.advertiseService("/map_data_service", &Resource::OnRequest, &resource);
    ROS_INFO("MapService is ready...");

    ros::spin();

    return 0;
}

