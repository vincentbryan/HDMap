//
// Created by vincent on 18-10-12.
//

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include "Sender.h"

using namespace hdmap;

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
    }
    visualization_msgs::Marker refer_line = Sender::GetLineStrip(section.GetReferPose(), 0xFFFFFF);
    array.markers.push_back(refer_line);
    pub.publish(array);
}