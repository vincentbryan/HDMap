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
class Client
{
private:
    Map mCurPlanMap;

    ros::Publisher mPubRouteRegion;
    ros::Publisher mPubTrafficLight;
    ros::Publisher mPubPlanner;
    ros::Publisher mPubVIZ;
    ros::Subscriber mSubGPS;
    ros::ServiceServer mServer;
    ros::ServiceClient mPlanClient;
    ros::ServiceClient mDataClient;

    std::mutex mLock;
    Coor mCurrentPosition;

    class Record
    {
    public:
        bool is_init = false;
        int curr_idx = -1;
        int curr_rid = -1;
        int curr_jid = -1;
        int next_idx = -1;


    public:
        void Reset()
        {
            is_init = false;
            curr_idx = -1;
            curr_rid = -1;
            curr_jid = -1;
            next_idx = -1;
        }
    }mRecord;

public:
    Client(ros::NodeHandle &n);

    void LocationCallBack(const nox_msgs::Location &msg);

    bool OnCommandRequest(HDMap::srv_map_cmd::Request &req, HDMap::srv_map_cmd::Response &res);

    void SendMap();

    void SendGPS(const Coor &v);

    void SendTrafficInfo(const Coor &v, RoadPtr target_road = nullptr);

    void SendNearPolygonRegion(const Coor &v, double radius = 100);

    void Process();
private:

    bool RePlanRoute(const Coor& cur);

    bool PlanByCommand(const std::string& method, std::vector<int> argv);

    std::string GetNearRoadPtrs(std::vector<RoadPtr>& near_roads, const Coor& coor, double distance = 50);

    std::string GetNearJunctionPtrs(std::vector<JuncPtr>& near_juncs, const Coor& coor, double distance = 50);

};
}


#endif //HDMAP_ROUTING_H
