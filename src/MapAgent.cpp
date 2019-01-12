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

    if(argc != 2)
    {
        std::cout << "Too few arguments, Usage: ./MapAgent point_cloud_path" << std::endl;
        return 1;
    }

    ros::NodeHandle n;

    ROS_INFO_STREAM("Map agent is ready...");

    Client client(n);
    client.SetInputPointCloud(argv[1]);

    ros::Rate rate(10);

    while(ros::ok())
    {
        client.Process();
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}