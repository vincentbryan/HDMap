//
// Created by vincent on 18-10-12.
//

#pragma once

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <mutex>
#include "Type/Pose.h"
#include "Type/LaneSection.h"
#include "Type/Junction.h"
#include "Type/HDMap.h"

namespace hdmap
{
class Sender
{
private:
    ros::Publisher pub;
    visualization_msgs::MarkerArray array;
    const std::string frame_id;
    std::vector<Pose> Translate(std::vector<Pose> poses, double length, double theta);

public:
    static unsigned int id;
    explicit Sender(ros::Publisher pub_);

    visualization_msgs::Marker GetLineStrip(std::vector<Pose> poses, double r, double g, double b, double a);
    visualization_msgs::Marker GetText(const std::string &content, Pose p);
    visualization_msgs::Marker GetArrow(const Vector2d & v, double r, double g, double b, double a);

    void Send();
    void SendPoses(std::vector<Pose> poses);
    void AddSection(LaneSection section);
    void AddJunction(Junction junction);
//    void AddMap(HDMap &map);
    void AddRoadId(Pose p, int id);
    void AddStartPoint(const Vector2d & v);
    void AddEndPoint(const Vector2d & v);
    void Clear()
    {
        array.markers.clear();
    }
};
}
