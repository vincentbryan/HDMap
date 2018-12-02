//
// Created by vincent on 18-10-19.
//


#include <ros/ros.h>
#include "nox_location.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>

class ViewGPS
{
public:



    ros::Publisher pub;


    void  renderCar(ros::Publisher& MarkerPublisher, double x, double y, double radius){

        visualization_msgs::Marker carmarker;
        carmarker.header.frame_id = "hdmap";
        carmarker.header.stamp = ros::Time();
        carmarker.ns = "map_test";
        carmarker.id = 0;
        carmarker.type = visualization_msgs::Marker::MESH_RESOURCE;
        carmarker.action = visualization_msgs::Marker::ADD;
        carmarker.pose.position.x = x;
        carmarker.pose.position.y = y;
        carmarker.pose.position.z = 0.7;
        carmarker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(TFSIMD_HALF_PI,0,TFSIMD_PI+radius);
        carmarker.scale.x = 0.009;
        carmarker.scale.y = 0.009;
        carmarker.scale.z = 0.009;
        carmarker.mesh_resource = "package://HDMap/res/car2/car.dae";
        carmarker.mesh_use_embedded_materials = 1;

        visualization_msgs::Marker textmarker;

        char _buf[32];
        sprintf(_buf,"(%3.2f, %3.2f, %3.1f)",x,y,radius*180.0/TFSIMD_PI);
        textmarker.text= _buf;

        textmarker.header.frame_id = "hdmap";
        textmarker.header.stamp = ros::Time();
        textmarker.ns = "map_test";
        textmarker.id = 1;
        textmarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        textmarker.action = visualization_msgs::Marker::ADD;
        textmarker.pose.position.x = x;
        textmarker.pose.position.y = y;
        textmarker.pose.position.z = 5;
        textmarker.scale.x = 1;
        textmarker.scale.y = 1;
        textmarker.scale.z = 1;
        textmarker.color.a = 1;
        textmarker.color.b = 1;
        textmarker.color.g = 1;
        textmarker.color.r = 1;

        visualization_msgs::MarkerArray array;
        array.markers.emplace_back(carmarker);
        array.markers.emplace_back(textmarker);
        MarkerPublisher.publish(array);
    }



    void CallBack(const nox_msgs::Location & msg)
    {
        renderCar(this->pub,msg.x,msg.y,msg.yaw*TFSIMD_PI/180.0);
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ViewGPS");
    ros::NodeHandle n;

    ViewGPS viewGPS;
    viewGPS.pub = n.advertise<visualization_msgs::MarkerArray>("ViewGPS", 0);

    ros::Subscriber sub = n.subscribe("Localization", 1000, &ViewGPS::CallBack, &viewGPS);

    ros::spin();

    return 0;
}