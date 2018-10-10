//
// Created by vincent on 18-10-9.
//

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Pose2D.h>
#include <sstream>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

    ros::Publisher pose_pub = n.advertise<geometry_msgs::Pose2D>("pose2d", 1000);
    ros::Rate loop_rate(10);

    int count = 0;
    double x = 0;
    double y = 0;
    double theta = 45.0 / 180.0 * M_PI;

    while (ros::ok())
    {

        geometry_msgs::Pose2D pose;

        x += 0.1;
        y += 0.1;
        if(x > 1000) x = 0;
        if(y > 1000) y = 0;

        pose.x = x;
        pose.y = y;
        pose.theta = theta;

        ROS_INFO("Send: %lf\t%lf\t%lf", x, y, theta);
//        std::cout << x << " " << y << " " << yaw << std::endl;
        pose_pub.publish(pose);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }


    return 0;
}


