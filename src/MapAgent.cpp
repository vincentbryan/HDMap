//
// Created by vincent on 18-10-31.
//

#include <ros/ros.h>
#include "Tool/Client.h"

using namespace hdmap;
using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_agent");
    ros::NodeHandle n;

    ROS_INFO_STREAM("Map agent is ready...");

    Client r(n);

    ros::Rate rate(5);

    while(ros::ok())
    {
        r.Process();
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}