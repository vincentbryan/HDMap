//
// Created by vincent on 18-10-31.
//

#ifndef HDMAP_ROUTING_H
#define HDMAP_ROUTING_H

#include <nox_location.h>
#include "../Type/Map.h"
#include "Location.h"

namespace hdmap
{
class Routing
{
private:
    Map mRouting;
    ros::Publisher mMapToTrafficLight;
    ros::Publisher mSender;

    struct
    {
        bool is_init = false;
        int curr_idx = -1;
        int curr_rid = -1;
        int curr_jid = -1;
        int next_rid = -1;
    }mRecord;

public:
    explicit Routing(ros::NodeHandle & n, const Map & map);

    void CallBack(const nox_msgs::Location & msg);

    void SendMap();

    void SendGPS(const Vector2d &v);

    void TrafficInfo(const Vector2d & v);
};
}


#endif //HDMAP_ROUTING_H
