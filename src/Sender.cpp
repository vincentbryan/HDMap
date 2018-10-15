//
// Created by vincent on 18-10-12.
//

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include "Sender.h"

using namespace hdmap;

unsigned int Sender::id = 0;

Sender::Sender(ros::Publisher pub_) : frame_id("/hdmap"), pub(pub_)
{};

visualization_msgs::Marker Sender::GetLineStrip(std::vector<Pose> poses, double r, double g, double b, double a)
{
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = frame_id;
    line_strip.header.stamp = ros::Time::now();
    line_strip.id = id++;
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    line_strip.scale.x = 0.05;

    auto func = [](double x)
    {
        x = std::abs(x);
        if(x > 1)
            return x - std::floor(x);
        else
            return x;
    };
    line_strip.color.r = func(r);
    line_strip.color.g = func(g);
    line_strip.color.b = func(b);
    line_strip.color.a = func(a);

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

visualization_msgs::Marker Sender::GetText(const std::string &content, Pose p)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
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

void Sender::Send()
{
    pub.publish(array);
}

void Sender::SendPoses(std::vector<Pose> poses)
{
    auto line_strip = GetLineStrip(poses, 0.5, 0.5, 0.5, 1.0);
    array.markers.emplace_back(line_strip);
    pub.publish(array);
}

void Sender::AddSection(LaneSection section)
{
    auto sid = section.iSectionId;
    for(auto x : section.GetAllPose())
    {
        visualization_msgs::Marker line_strip;
        if(x.first == 0)
            line_strip = GetLineStrip(x.second, 0.5 + 0.2 * (sid % 10), 0.3 + 0.3 * (sid % 10), 0.2 * (sid % 10), 1.0);
        else
            line_strip = GetLineStrip(x.second, 0.5 + 0.2 * (sid % 10), 0.3, 0.2 * (sid % 10), 1.0);
        array.markers.push_back(line_strip);

        visualization_msgs::Marker m = GetText(std::to_string(x.first), x.second[x.second.size()/2]);
        array.markers.push_back(m);
    }
}

void Sender::AddJunction(Junction junciton)
{
    auto jid = junciton.iJunctionId;
    for(auto & x : junciton.GetAllPose())
    {
        visualization_msgs::Marker line_strip = GetLineStrip(x,
                                                             0.5 + 0.2 * (jid % 10),
                                                             0.7 + 0.3 * (jid % 10),
                                                             0.2 * (jid % 10), 1.0);;
        array.markers.push_back(line_strip);
    }
}
void Sender::AddMap(HDMap &map)
{
    for(auto & sec : map.GetAllSection())
    {
        AddSection(sec);
    }

    for(auto & jun : map.GetAllJunction())
    {
        AddJunction(jun);
    }

}