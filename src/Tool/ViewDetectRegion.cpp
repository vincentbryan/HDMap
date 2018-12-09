//
// Created by iceytan on 18-11-26.
//
#include <utility>
#include <ros/ros.h>
#include "nox_location.h"
#include <visualization_msgs/Marker.h>
#include "Type/Angle.h"
#include <visualization_msgs/MarkerArray.h>
#include <HDMap/msg_route_region.h>
#include <Tool/polygonScan.h>
#include <mutex>
#include <sensor_msgs/PointCloud2.h>
#include <termios.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_datatypes.h>

class ViewDetectRegion
{

    using PointT =  pcl::PointXYZRGB;
    using PointCloud = pcl::PointCloud<PointT >;
    using Kdtree = pcl::KdTreeFLANN <PointT> ;

    const char* CAR_RESOURCE_PATH= "package://HDMap/res/car2/car.dae";
    const char* FRAME_ID = "hdmap";

public:
    enum DisPlayMode { ABSOLUTE, RELATIVE};


    explicit ViewDetectRegion(ros::NodeHandle& n)
    : mNode(n),mRegionPoints(new PointCloud),mode(RELATIVE) {
        mCurY=mCurX=mCurYaw = 0;
        mPubRegionPoint = n.advertise<sensor_msgs::PointCloud2>("/ViewDetectRegion/RegionPoint",1);
        mPubCarView = n.advertise<visualization_msgs::MarkerArray>("/ViewDetectRegion/CarInfo", 1);
        mSubGPS = n.subscribe("Localization", 2,  &ViewDetectRegion::LocationCallBack, this);
        mSubRegion = n.subscribe("map_pub_route_region", 2, &ViewDetectRegion::RoadRegionCallBack,this);
    }

    void  RenderCarInfo(){
        visualization_msgs::MarkerArray array;
        visualization_msgs::Marker carmarker;
        visualization_msgs::Marker textmarker;

        if( mode == RELATIVE )
        {
            carmarker.pose.position.x = 0;
            carmarker.pose.position.y = 0;
            carmarker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(TFSIMD_HALF_PI,0,TFSIMD_PI);
        }
        else
        {
            carmarker.pose.position.x = mCurX;
            carmarker.pose.position.y = mCurY;
            carmarker.pose.orientation =
                    tf::createQuaternionMsgFromRollPitchYaw(TFSIMD_HALF_PI,0,TFSIMD_PI+mCurYaw*M_PI/180);
        }

        carmarker.header.frame_id = FRAME_ID;
        carmarker.header.stamp = ros::Time();
        carmarker.id = 0;
        carmarker.type = visualization_msgs::Marker::MESH_RESOURCE;
        carmarker.action = visualization_msgs::Marker::ADD;
        carmarker.pose.position.z = 0.7;
        carmarker.scale.x =  0.009;
        carmarker.scale.y =  0.009;
        carmarker.scale.z =  0.009;
        carmarker.mesh_resource = CAR_RESOURCE_PATH;
        carmarker.mesh_use_embedded_materials = 1;

        char _buf[32];
        sprintf(_buf,"(%3.2f, %3.2f, %3.1f)",mCurX,mCurY,mCurYaw);
        textmarker.text= _buf;

        textmarker.header.frame_id = FRAME_ID;
        textmarker.header.stamp = ros::Time();
        textmarker.id = 1;
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
            const double theta =  -mCurYaw*M_PI/180.0;
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
        _tmp.header.stamp = ros::Time();
        mPubRegionPoint.publish(_tmp);
    }

    void LocationCallBack(const nox_msgs::Location & msg)
    {
        std::lock_guard<std::mutex> lock(mLock);
        mCurX = msg.x;
        mCurY = msg.y;
        mCurYaw = msg.yaw;
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
            if(polygon.points.size() < 10)
            {
                _effect_points.clear();
                double minX = std::numeric_limits<int>::max();
                double minY = std::numeric_limits<int>::max();
                double maxX = std::numeric_limits<int>::min();
                double maxY = std::numeric_limits<int>::min();
                for(auto& p: polygon.points)
                {
                    _effect_points.emplace_back(p.x,p.y);
                    if(p.x > maxX) maxX = p.x;
                    if(p.y > maxY) maxY = p.y;
                    if(p.x < minX) minX = p.x;
                    if(p.y < minY) minY = p.y;
                }
                // 平移（使得所有点大于0）,放大（0.5步长）并给扫描线算法使用
                const double scale_step = 4;
                std::vector<PolygonScan::Point> ps;
                for(auto& p: _effect_points)
                {
                    ps.emplace_back(scale_step*(p.x-minX),scale_step*(p.y-minY));
                }

                auto _res_points = PolygonScan::polygonScan(ps);

                // 缩小并平移到点云中
                for(auto&p:_res_points)
                {
                    PointT _tmp_p{255,255,255};
                    _tmp_p.x = static_cast<float>(p.x / scale_step + minX);
                    _tmp_p.y = static_cast<float>(p.y / scale_step + minY);
                    mRegionPoints->points.push_back(_tmp_p);
                }

            } else{
                for(auto& p: polygon.points)
                {
                    PointT _tmp_p{255,255,255};
                    _tmp_p.x = static_cast<float>(p.x);
                    _tmp_p.y = static_cast<float>(p.y);
                    mRegionPoints->points.push_back(_tmp_p);
                }
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


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ViewDetectRegion");
    ros::NodeHandle n;
    ViewDetectRegion viewDetectRegion(n);

    if (argc==1 || (argc == 2 && strcmp(argv[1],"RELATIVE" )==0))
    {
        viewDetectRegion.SetMode(ViewDetectRegion::RELATIVE);
        ROS_INFO("ViewDetectRegion mode: RELATIVE");
    }
    else if (argc == 2 && strcmp(argv[1],"ABSOLUTE" )==0)
    {
        viewDetectRegion.SetMode(ViewDetectRegion::ABSOLUTE);
        ROS_INFO("ViewDetectRegion mode: ABSOLUTE");
    }
    else
    {
        ROS_ERROR("Invalid input: %s",argv[1]);
    }

    ros::spin();

    return 0;
}