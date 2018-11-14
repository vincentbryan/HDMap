//
// Created by vincent on 18-10-31.
//

#ifndef HDMAP_ROUTING_H
#define HDMAP_ROUTING_H

#include <nox_location.h>
#include "Type/Map.h"
#include <HDMap/msg_map_cmd.h>
#include <HDMap/srv_map_cmd.h>

namespace hdmap
{
class Route
{
private:
    Map mHDMap;

    ros::Publisher mPubTrafficLight;
    ros::Publisher mPubPlanner;
    ros::Publisher mPubVIZ;
    ros::Subscriber mSubGPS;
    ros::ServiceServer mServer;
    ros::ServiceClient mClient;

    std::mutex mLock;
    Vector2d mCurrentPosition;

    class Record
    {
    public:
        bool is_init = false;
        int curr_idx = -1;
        int curr_rid = -1;
        int curr_jid = -1;
        int next_rid = -1;

    public:
        void Reset()
        {
            is_init = false;
            curr_idx = -1;
            curr_rid = -1;
            curr_jid = -1;
            next_rid = -1;
        }
    }mRecord;

public:
    Route(ros::NodeHandle & n);

    void LocationCallBack(const nox_msgs::Location &msg);
    bool OnCommandRequest(HDMap::srv_map_cmd::Request &req, HDMap::srv_map_cmd::Response &res);

    void SendMap();

    void SendGPS(const Vector2d &v);

    void SendTrafficInfo(const Vector2d &v);

    void Process();
};
}


#endif //HDMAP_ROUTING_H
