//
// Created by iceytan on 19-1-4.
//

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <HDMap/srv_map_cmd.h>
using namespace std;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "fake_map_command");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<HDMap::srv_map_cmd>("map_command");

    std::string route_info;

    HDMap::srv_map_cmd srv;
    srv.request.cmd = "r2r";
    for(int i = 1; i < argc; ++i)
    {
        route_info += string(argv[i])+" ";
        srv.request.argv.push_back(stoi(argv[i]));
    }

    auto timerCallback = [&](const ros::TimerEvent&)
    {
        if(client.call(srv))
        {
            ROS_INFO("Request Plan : %s.", route_info.c_str());
        } else{
            ROS_ERROR("Request Plan : %s fail.", route_info.c_str());
        }
    };

    ros::Timer timer = n.createTimer(ros::Duration(5), timerCallback);

    ros::spin();
}