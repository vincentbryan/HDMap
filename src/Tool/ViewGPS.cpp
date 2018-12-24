//
// Created by vincent on 18-10-19.
//


#include <ros/ros.h>
#include "nox_location.h"
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>

class ViewGPS
{
public:
    ros::Publisher pub;
    
    int id = 0;

    void CallBack(const nox_msgs::Location & msg)
    {
        visualization_msgs::Marker marker;

        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = "/world";
        marker.id = id++;
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.pose.position.x = msg.x;
        marker.pose.position.y = msg.y;
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;
        marker.color.a = 1;
        marker.color.r = 1;
        marker.color.g = 1;
        marker.color.b = 1;

        pub.publish(marker);
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ViewGPS");
    ros::NodeHandle n;

    ViewGPS viewGPS;
    viewGPS.pub = n.advertise<visualization_msgs::Marker>("ViewGPS", 0);

    ros::Subscriber sub = n.subscribe("Localization", 1000, &ViewGPS::CallBack, &viewGPS);

    ros::spin();

    return 0;
}