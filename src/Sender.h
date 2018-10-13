//
// Created by vincent on 18-10-12.
//

#ifndef HDMAP_SENDER_H
#define HDMAP_SENDER_H

#include <visualization_msgs/Marker.h>
#include "Type/Pose.h"
#include "Type/LaneSection.h"
namespace hdmap
{
class Sender
{
public:
    static unsigned int id;
    static visualization_msgs::Marker GetLineStrip(std::vector<Pose> poses, unsigned long color_);
    static visualization_msgs::Marker GetText(const std::string &content, Pose p);
    static void SendSection(LaneSection section, ros::Publisher pub);
};
}
#endif //HDMAP_SENDER_H
