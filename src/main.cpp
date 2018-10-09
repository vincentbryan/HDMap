//
// Created by vincent on 18-10-7.
//
#include <ros/ros.h>
#include <thread>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose2D.h>
#include <queue>
#include <mutex>
#include "SyncQueue.hpp"

using namespace hdmap;

int main( int argc, char** argv )
{
    ros::init(argc, argv, "hdmap");
    ros::NodeHandle n;
    ros::Rate rate(1);
    SyncQueue<geometry_msgs::Pose2D> buffer(1000);
    ros::Subscriber sub = n.subscribe("pose2d", 1000, &SyncQueue<geometry_msgs::Pose2D>::Push, &buffer);

    std::thread collector([&](){
        ros::spin();
    });

    while (true)
    {
        std::cout << buffer.Size() << std::endl;
    }

    if(collector.joinable()) collector.join();

    return 0;
}
