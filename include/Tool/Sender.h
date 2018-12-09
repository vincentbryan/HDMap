//
// Created by vincent on 18-10-12.
//

#pragma once

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <mutex>
#include "Type/Pose.h"

namespace hdmap
{
class Map;
class Sender
{
private:
    ros::Publisher pub;
    const std::string frame_id = "/hdmap";

public:
    visualization_msgs::MarkerArray array;
    std::vector<Pose> Translate(std::vector<Pose> poses, double length, double theta);

    static unsigned int id;
    explicit Sender(ros::Publisher pub_);

    visualization_msgs::Marker GetLineStrip(std::vector<Pose> poses, double r, double g, double b, double a, double z = 0, double width = 0.15);

    visualization_msgs::Marker
    GetText(const std::string &content, Coor v, double r = 1.0, double g = 1.0, double b = 1.0, double a = 1.0,
            double z = 0, double scale = 1.0);

    visualization_msgs::Marker GetCone(const Coor &v, double r, double g, double b, double a, double scale = 1.0) const;
    visualization_msgs::Marker GetArrow(const Pose & p, double r, double g, double b, double a, double scale = 1.0);

    void Send();
    void SendPoses(std::vector<Pose> poses, double r = 0.5, double g = 0.5, double b = 0.5, double a = 1.0, double z = 0.0, double width = 0.08);

    void AddStartPoint(const Coor &v);

    void AddEndPoint(const Coor &v);
    void AddAnchor(const Pose & p, int id);
    void Clear();
};
}
