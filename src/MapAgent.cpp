//
// Created by vincent on 18-10-31.
//

#include <ros/ros.h>
#include "Type/Map.h"
#include "Tool/Routing.h"

using namespace hdmap;
using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_agent");
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<visualization_msgs::MarkerArray>("HDMap", 1000);
    shared_ptr<Sender> p_sender(new Sender(pub));

    Map map;
    map.SetSender(p_sender);
    map.Load("/media/vincent/DATA/Ubuntu/Project/catkin_ws/src/HDMap/data/planner02.xml");

    ROS_INFO_STREAM("Map agent is ready...");
    map.Send();
    Routing r(map);
    ros::Subscriber sub = n.subscribe("Localization", 1000, &Routing::CallBack, &r);

    ros::spin();

    return 0;
}