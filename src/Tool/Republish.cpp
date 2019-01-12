//
// Created by yul on 18-12-19.
//

#include <ros/ros.h>
#include <algorithm>
#include <math.h>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <mutex>
#include <Eigen/Core>

#include "pcl/common/time.h"
#include "pcl/common/transforms.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/search/impl/kdtree.hpp"
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/concatenate.h>
#include <pcl/common/copy_point.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/simple_filter.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <pcl/filters/voxel_grid.h>
#include <nav_msgs/Odometry.h>

std::vector<double> velo2imu_XYZ;
std::vector<double> velo2imu_RPY;
ros::Publisher velo_pub;
pcl::PointCloud<pcl::PointXYZI>::Ptr velodyne_global_save(new  pcl::PointCloud<pcl::PointXYZI>);
pcl::VoxelGrid<pcl::PointXYZI> sor;


void filter_callback(const sensor_msgs::PointCloud2::ConstPtr &pcl_msg, const nav_msgs::Odometry::ConstPtr &imu_msg){
//  ROS_INFO_STREAM("update pointcloud");

    pcl::PointCloud<pcl::PointXYZI>::Ptr current_sensor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*pcl_msg, *current_sensor_cloud_ptr);
    tf::Transform transform_velo;
    transform_velo.setOrigin( tf::Vector3(velo2imu_XYZ[0],velo2imu_XYZ[1],velo2imu_XYZ[2]) );
    tf::Quaternion q;
    q.setRPY(velo2imu_RPY[0]/180.0*M_PI, velo2imu_RPY[1]/180.0*M_PI, velo2imu_RPY[2]/180.0*M_PI);
    transform_velo.setRotation(q);


    auto _pos = imu_msg->pose.pose.position;
    auto _qua = imu_msg->pose.pose.orientation;

    tf::Transform gps_pose({_qua.x,_qua.y,_qua.z,_qua.w},{_pos.x,_pos.y,_pos.z});


    tf::StampedTransform transform_local2global_;
    transform_local2global_.setData(gps_pose);
    tf::Transform transform_temp;
    transform_temp = (transform_local2global_ * transform_velo.inverse());
    Eigen::Affine3d t1;
    tf::transformTFToEigen(transform_local2global_,t1);
    //ROS_INFO_STREAM(t1.matrix());

    Eigen::Affine3d t;
    tf::transformTFToEigen(transform_temp,t);
    pcl::PointCloud<pcl::PointXYZI>::Ptr velodyne_global(new  pcl::PointCloud<pcl::PointXYZI>);
    Eigen::Matrix4d local2global_ = t.matrix();
    pcl::transformPointCloud(*current_sensor_cloud_ptr,*velodyne_global,local2global_.cast<float>() );
    sensor_msgs::PointCloud2 point_pub;
    pcl::toROSMsg(*velodyne_global,point_pub);

//    velodyne_global_save->insert(velodyne_global_save->end(),velodyne_global->begin(),velodyne_global->end());
//    sor.setInputCloud (velodyne_global_save);
//    sor.filter(*velodyne_global_save);

    point_pub.header.frame_id="/map";
    point_pub.header.stamp = ros::Time::now();
    velo_pub.publish(point_pub);
    tf::Transform tf_pub;
    tf::transformEigenToTF(t.inverse(),tf_pub);
    static tf::TransformBroadcaster br;
    ROS_INFO_STREAM("sending");
    br.sendTransform(tf::StampedTransform(tf_pub, ros::Time::now(), "velodyne", "map"));
}



int main(int argc, char **argv){
    ros::init(argc, argv, "republish");
    ros::NodeHandle nh_("~");
    nh_.param("velo2imu_XYZ", velo2imu_XYZ);
    nh_.param("velo2imu_RPY", velo2imu_RPY);
    if(velo2imu_XYZ.size() != 3) velo2imu_XYZ = {0.0593174, -1.45416, -1.35861};
    if(velo2imu_RPY.size() != 3) velo2imu_RPY = {1.19488, 2.25248, 94.6435};

    sor.setLeafSize (0.1f, 0.1f, 0.1f);

    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh_,"/velodyne_points",1);

    message_filters::Subscriber<nav_msgs::Odometry> imu_sub(nh_,"odom",10);

    typedef message_filters::sync_policies::ApproximateTime< sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy;
    // // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100),  cloud_sub, imu_sub);
    sync.setMaxIntervalDuration(ros::Duration(0.02));
    sync.registerCallback(boost::bind(&filter_callback, _1, _2));
    velo_pub = nh_.advertise<sensor_msgs::PointCloud2>("/points_global", 1);

    ros::spin();
    // std::cout <<"saving...\n";
    //pcl::io::savePCDFileASCII (std::to_string(ros::Time::now().sec)+".pcd", *velodyne_global_save);
    return 0;
}