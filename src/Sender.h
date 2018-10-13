//
// Created by vincent on 18-10-12.
//

#ifndef HDMAP_SENDER_H
#define HDMAP_SENDER_H

#include <visualization_msgs/Marker.h>
#include <mutex>
#include "Type/Pose.h"
#include "Type/LaneSection.h"

namespace hdmap
{
class Sender
{
private:
    ros::Publisher pub;
    visualization_msgs::MarkerArray array;
    const std::string frame_id;
    std::mutex m;
public:
    explicit Sender(ros::Publisher pub_);
    static unsigned int id;
    visualization_msgs::Marker GetLineStrip(std::vector<Pose> poses, double r, double g, double b, double a);
    visualization_msgs::Marker GetText(const std::string &content, Pose p);
    void Send();
    void AddSection(LaneSection section);
};
}
#endif //HDMAP_SENDER_H
