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
    if(argc != 2)
    {
        std::cout << "Too few arguments, Usage: ./MapService input_map" << std::endl;
        return 1;
    }

    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<visualization_msgs::MarkerArray>("HDMap", 100);
    shared_ptr<Sender> p_sender(new Sender(pub));

    Resource resource(argv[1]);
    resource.SetSender(p_sender);

    Planner planner(resource.GetMap(), p_sender);

    ros::ServiceServer plan_server = n.advertiseService("map_plan_service", &Planner::OnRequest, &planner);
    ros::ServiceServer data_server = n.advertiseService("map_data_service", &Resource::OnRequest, &resource);
    ROS_INFO("MapService is ready...");

    ros::spin();

    return 0;
}

