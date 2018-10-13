//
// Created by vincent on 18-10-12.
//

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include "Sender.h"
#include<iostream>

using namespace hdmap;

unsigned int  Sender::id = 0;
visualization_msgs::Marker Sender::GetLineStrip(std::vector<Pose> poses, unsigned long color_)
{
    static int id = 0;
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "/hdmap";
    line_strip.header.stamp = ros::Time::now();
    line_strip.id = id++;
    line_strip.action = visualization_msgs::Marker::MODIFY;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    line_strip.scale.x = 0.02;

    line_strip.color.r = ((color_ & 0xFF000000) >> 24) / 255.0;
    line_strip.color.g = ((color_ & 0x00FF0000) >> 16) / 255.0;
    line_strip.color.b = ((color_ & 0x0000FF00) >>  8) / 255.0;
    line_strip.color.a = ((color_ & 0x000000FF)) / 255.0;

    for(auto p : poses)
    {
        geometry_msgs::Point point;
        point.x = p.x;
        point.y = p.y;
        point.z = 0;
        line_strip.points.push_back(point);
    }

    return line_strip;
}

void Sender::SendSection(LaneSection section, ros::Publisher pub)
{
    visualization_msgs::MarkerArray array;
    for(auto x : section.GetAllPose())
    {
        visualization_msgs::Marker line_strip = Sender::GetLineStrip(x.second, 0xFF00FFFF);
        array.markers.push_back(line_strip);

        visualization_msgs::Marker m = GetText(std::to_string(x.first), x.second[x.second.size()/2]);
        array.markers.push_back(m);
    }

    visualization_msgs::Marker refer_line = Sender::GetLineStrip(section.GetReferPose(), 0xFFFFFF);
    array.markers.push_back(refer_line);

    pub.publish(array);
}

visualization_msgs::Marker Sender::GetText(const std::string &content, Pose p)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id="/hdmap";
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;

    marker.id = id++;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    marker.scale.z = 0.5;
    marker.color.b = 1;
    marker.color.g = 1;
    marker.color.r = 1;
    marker.color.a = 1;

    geometry_msgs::Pose pose;
    pose.position.x = p.x;
    pose.position.y = p.y;
    pose.position.z = 0;
    marker.text= content;
    marker.pose=pose;

    return marker;
}