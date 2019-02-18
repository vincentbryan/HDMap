#include <ros/ros.h>
#include "Tool/Client.h"

using namespace hdmap;
using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_agent");
    ros::NodeHandle n("~");

    std::string route_net_path;
    std::string point_cloud_path;

    n.param<std::string>("point_cloud_path", point_cloud_path, "");
    ROS_INFO("[ Map Agent ] point_cloud_path: %s", point_cloud_path.empty()? "not set":point_cloud_path.c_str());


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