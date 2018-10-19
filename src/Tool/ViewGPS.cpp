//
// Created by vincent on 18-10-19.
//


#include <ros/ros.h>
#include <Location.h>
#include <visualization_msgs/Marker.h>
#include "../Type/Angle.h"

class ViewGPS
{
public:
    ros::Publisher pub;
    std::vector<std::pair<double, double>> poses;
    int id = 0;


    void CallBack(const HDMap::Location & msg)
    {
        hdmap::Angle a;
        a.FromYaw(msg.yaw);
        ROS_INFO("[%6.3f %6.3f %6.3f]", msg.x, msg.y, a.Value());

        std::pair<double, double> p(msg.x, msg.y);
        poses.emplace_back(p);

        if(poses.size() > 5)
        {
            visualization_msgs::Marker points;
            points.header.frame_id = "/hdmap";
            points.header.stamp = ros::Time::now();
            points.action = visualization_msgs::Marker::ADD;

            points.pose.orientation.w = 0;
            points.id = id++;
            points.type = visualization_msgs::Marker::POINTS;

            points.scale.x = 0.08;
            points.scale.y = 0.08;
            points.color.g = 1.0f;
            points.color.a = 1.0;

            geometry_msgs::Point p1;
            p1.x = poses.front().first;
            p1.y = poses.front().second;
            p1.z = 1.0;
            points.points.push_back(p1);

            geometry_msgs::Point p2;
            p2.x = poses.back().first;
            p2.y = poses.back().second;
            p2.z = 1.0;
            points.points.push_back(p2);

            pub.publish(points);
            poses.clear();
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ViewGPS");
    ros::NodeHandle n;

    ViewGPS viewGPS;
    viewGPS.pub = n.advertise<visualization_msgs::Marker>("ViewGPS", 1000);
    ros::Subscriber sub = n.subscribe("Localization", 1000, &ViewGPS::CallBack, &viewGPS);

    ros::spin();

    return 0;
}