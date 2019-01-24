//
// Created by iceytan on 18-11-26.
//
#include <utility>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <HDMap/msg_route_region.h>
#include <mutex>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <Type/Pose.h>

#include "Type/Angle.h"
class ViewDetectRegion
{

    using PointT =  pcl::PointXYZRGB;
    using PointCloud = pcl::PointCloud<PointT >;
    using Kdtree = pcl::KdTreeFLANN <PointT> ;

    const char* CAR_RESOURCE_PATH= "package://HDMap/res/car_texture/car.dae";


public:
    enum DisPlayMode { ABSOLUTE, RELATIVE};
    static char* FRAME_ID;

    explicit ViewDetectRegion(ros::NodeHandle& n)
    : mNode(n),mRegionPoints(new PointCloud),mode(RELATIVE) {
        mCurY=mCurX=mCurYaw = 0;
        mPubRegionPoint = n.advertise<sensor_msgs::PointCloud2>("/ViewDetectRegion/RegionPoint",1);
        mPubCarView = n.advertise<visualization_msgs::MarkerArray>("/ViewDetectRegion/CarInfo", 1);
        mSubGPS = n.subscribe("/odom", 2,  &ViewDetectRegion::LocationCallBack, this);
        mSubRegion = n.subscribe("/map_pub_route_region", 2, &ViewDetectRegion::RoadRegionCallBack,this);
    }

    void  RenderCarInfo(){
        visualization_msgs::MarkerArray array;
        visualization_msgs::Marker carmarker;
        visualization_msgs::Marker textmarker;
        visualization_msgs::Marker triangle;

        if( mode == RELATIVE )
        {
            carmarker.pose.position.x = 0;
            carmarker.pose.position.y = 0;
        }
        else
        {
            carmarker.pose.position.x = mCurX;
            carmarker.pose.position.y = mCurY;
            carmarker.pose.orientation = tf::createQuaternionMsgFromYaw(mCurYaw);

            triangle.pose.position.x = mCurX;
            triangle.pose.position.y = mCurY;
            triangle.pose.orientation = tf::createQuaternionMsgFromYaw(mCurYaw+TFSIMD_HALF_PI);

        }

        carmarker.header.frame_id = FRAME_ID;
        carmarker.header.stamp = ros::Time::now();
        carmarker.id = 1;
        carmarker.action = visualization_msgs::Marker::ADD;
        carmarker.pose.position.z = 0;
        carmarker.scale.x =  1.8;
        carmarker.scale.y =  4.0;
        carmarker.scale.z =  1.0;
        carmarker.color.a = carmarker.color.g = carmarker.color.r = carmarker.color.b = 1;
        carmarker.type = visualization_msgs::Marker::CUBE;


        geometry_msgs::Point top;
        geometry_msgs::Point bottom_left;
        geometry_msgs::Point bottom_right;

        triangle.header.frame_id = FRAME_ID;
        triangle.header.stamp = ros::Time::now();
        triangle.id = 2;
        triangle.action = visualization_msgs::Marker::ADD;
        triangle.pose.position.z = 1.0;
        triangle.scale.x =  1.5;
        triangle.scale.y =  0.5;
        triangle.scale.z =  0.1;
        triangle.color.a = triangle.color.g = triangle.color.r  = 1;
        triangle.color.b = 0;
        triangle.type = visualization_msgs::Marker::ARROW;



        char _buf[32];
        sprintf(_buf,"(%3.2f, %3.2f, %3.1f)",mCurX,mCurY, mCurYaw*180.0/M_PI+90);
        textmarker.text= _buf;

        textmarker.header.frame_id = FRAME_ID;
        textmarker.header.stamp = ros::Time::now();
        textmarker.id = 0;
        textmarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        textmarker.action = visualization_msgs::Marker::ADD;
        textmarker.pose.position = carmarker.pose.position;
        textmarker.pose.position.z = 5;
        textmarker.scale.x = 1;
        textmarker.scale.y = 1;
        textmarker.scale.z = 1;
        textmarker.color.a = 1;
        textmarker.color.b = 1;
        textmarker.color.g = 1;
        textmarker.color.r = 1;

        array.markers.emplace_back(carmarker);
        array.markers.emplace_back(textmarker);
        array.markers.emplace_back(triangle);
        mPubCarView.publish(array);
    }

    void RenderRegionPoint()
    {
        if(mRegionPoints->points.empty()) return;

        const double distance_range = 100;
        PointCloud::Ptr transform_cloud(new PointCloud);

        /// 只显示一定范围内的
        PointT searchPoint;
        searchPoint.x= static_cast<float>(mCurX);
        searchPoint.y= static_cast<float>(mCurY);
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        regionTree.radiusSearch(searchPoint,distance_range,
                pointIdxRadiusSearch,pointRadiusSquaredDistance);

        pcl::copyPointCloud(*mRegionPoints, pointIdxRadiusSearch, *transform_cloud);


        if (mode == RELATIVE)
        {
            /// 转换坐标系
            /// 原点在curX curY 方向为Yaw
            Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
            transform(0,3) = -mCurX;
            transform(1,3) = -mCurY;
            pcl::transformPointCloud (*transform_cloud, *transform_cloud, transform);
            const double theta =  M_PI_2 -mCurYaw;
            transform(0,0) = cos (theta);
            transform(0,1) = -sin(theta);
            transform(1,0) = sin (theta);
            transform(1,1) = cos (theta);
            transform(0,3) = 0;
            transform(1,3) = 0;
            pcl::transformPointCloud (*transform_cloud, *transform_cloud, transform);
        }

        sensor_msgs::PointCloud2 _tmp;
        pcl::toROSMsg(*transform_cloud,_tmp);

        _tmp.header.frame_id = FRAME_ID;
        _tmp.header.stamp = ros::Time::now();
        mPubRegionPoint.publish(_tmp);
    }

    void LocationCallBack(const nav_msgs::Odometry & msg)
    {
        std::lock_guard<std::mutex> lock(mLock);
        /*
        // 位置调整
        const double lx = -0.13;
        const double ly = 1.34;
        mCurYaw = msg.yaw -1.5;
        const double theta = mCurYaw * M_PI / 180.0;

        mCurX = msg.x + lx * cos(theta) - ly * sin(theta);
        mCurY = msg.y + lx * sin(theta) + ly * cos(theta);
        */

        const auto& _position = msg.pose.pose.position;
        mCurX = _position.x;
        mCurY = _position.y;

        geometry_msgs::Quaternion orientation = msg.pose.pose.orientation;
        tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
        double _pitch, _roll;
        mat.getEulerYPR(mCurYaw, _pitch, _roll);

        RenderCarInfo();
        RenderRegionPoint();
    }
    void RoadRegionCallBack(const HDMap::msg_route_region & msg)
    {
        mLock.lock();
        mRegionPoints->points.clear();
        std::vector<hdmap::Coor> _effect_points;
        for(auto& polygon: msg.polygons)
        {
            for (auto &p: polygon.points) {
                PointT _tmp_p{255, 255, 255};
                _tmp_p.x = static_cast<float>(p.x);
                _tmp_p.y = static_cast<float>(p.y);
                mRegionPoints->points.push_back(_tmp_p);
            }

            regionTree.setInputCloud(mRegionPoints);
        }

        mLock.unlock();
    }

    void SetMode(DisPlayMode mode)
    {
        assert(mode == RELATIVE || mode == ABSOLUTE);
        this->mode = mode;
    }

private:
    ros::NodeHandle& mNode;
    ros::Publisher mPubCarView;
    ros::Publisher mPubRegionPoint;
    ros::Subscriber mSubGPS;
    ros::Subscriber mSubRegion;
    double mCurX,mCurY,mCurYaw;
    PointCloud::Ptr mRegionPoints;
    Kdtree  regionTree;
    std::mutex mLock;
    DisPlayMode mode;

};
char* ViewDetectRegion::FRAME_ID = "/map";

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ViewDetectRegion");
    ros::NodeHandle n("~");
    ViewDetectRegion viewDetectRegion(n);

    std::string view_mode;

    n.param<std::string>("view_mode", view_mode, "ABSOLUTE");
    if (view_mode != "ABSOLUTE" && view_mode != "RELATIVE")
    {
        ROS_WARN("[ viewDetectRegion ] view_mode should be set to ABSOLUTE or RELATIVE, %s is illegal, ABSOLUTE will be used.", view_mode.c_str());
        view_mode = "ABSOLUTE";
    }
    ROS_INFO("[viewDetectRegion ] view mode: %s", view_mode.c_str());

    if (view_mode == "RELATIVE")
    {
        ViewDetectRegion::FRAME_ID = "/velodyne";
        viewDetectRegion.SetMode(ViewDetectRegion::RELATIVE);
    }
    else
    {
        viewDetectRegion.SetMode(ViewDetectRegion::ABSOLUTE);
    }

    ros::spin();
    return 0;
}