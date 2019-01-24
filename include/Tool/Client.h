//
// Created by vincent on 18-10-31.
//

#ifndef HDMAP_ROUTING_H
#define HDMAP_ROUTING_H

#include "Type/Map.h"
#include "nox_msg/nox_location.h"

#include <HDMap/msg_map_cmd.h>
#include <HDMap/srv_map_cmd.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <nav_msgs/Odometry.h>

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
    ros::Publisher mPubPointCloud;
    ros::Publisher mPubRouteInfo;
    ros::Subscriber mOdomSubscribe;
    ros::Subscriber mLocaSubscribe;
    ros::ServiceServer mServer;
    ros::ServiceClient mPlanClient;
    ros::ServiceClient mDataClient;

    std::mutex mLock;
    Pose mCurrentPose;

    pcl::PointCloud<pcl::PointXYZI>::Ptr mLocalPointCloudPtr;
    pcl::KdTreeFLANN <pcl::PointXYZI> mKdTree;

    class Record
    {
    public:
        bool is_init = false;
        int curr_idx = -1;
        int curr_rid = -1;
        int curr_jid = -1;
        int next_idx = -1;
        Pose start_pose = Pose(-1, -1, 0);
        Pose target_pose = Pose(-1, -1, 0);
        std::string plan_method = "undefined";

    public:
        void Reset()
        {
            is_init = false;
            curr_idx = -1;
            curr_rid = -1;
            curr_jid = -1;
            next_idx = -1;
            start_pose = Pose(-1, -1, 0);
            target_pose = Pose(-1, -1, 0);
            plan_method = "undefined";
        }
    }mRecord;

public:
    Client(ros::NodeHandle &n);

    void OdometryCallBack(const nav_msgs::Odometry &msg);

    void LocationCallBack(const nox_msgs::Location &msg);

    bool OnCommandRequest(HDMap::srv_map_cmd::Request &req, HDMap::srv_map_cmd::Response &res);

    void SendMap();

    void SendGPS(const Coor &v);

    void SendTrafficInfo(const Coor &v, RoadPtr target_road = nullptr);

    void SendNearPolygonRegion(const Coor &v, double radius = 100);

    void SendPointCloud(const Coor &v, const double& distance);

    void SetInputPointCloud(const std::string& path);

    void Process();

private:

    bool RePlanRoute(const Pose& cur_pose);

    bool PlanByCommand(const std::string& method, std::vector<double> argv);

    std::string GetNearRoadPtrs(std::vector<RoadPtr>& near_roads, const Coor& coor, double distance = 50);

    std::string GetNearJunctionPtrs(std::vector<JuncPtr>& near_juncs, const Coor& coor, double distance = 50);


};
}


#endif //HDMAP_ROUTING_H
