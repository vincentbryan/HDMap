#include <ros/ros.h>
#include <thread>
#include <chrono>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include "Tool/Sender.h"
#include "Type/Map.h"

using namespace std;
using namespace hdmap;
using PointT = pcl::PointXYZI;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_viewer");
    ros::NodeHandle n("~");

    std::string view_mode;
    std::string route_net_path;
    std::string point_cloud_path;

    n.param<std::string>("view_mode", view_mode, "ABSOLUTE");
    if (view_mode != "ABSOLUTE" && view_mode != "RELATIVE")
    {
        ROS_WARN("[ Map Viewer ] view_mode should be set to ABSOLUTE or RELATIVE, %s is illegal, ABSOLUTE will be used.", view_mode.c_str());
        view_mode = "ABSOLUTE";
    }
    ROS_INFO("[ Map Viewer ] view mode: %s", view_mode.c_str());

    n.param<std::string>("route_net_path", route_net_path, "");
    if(route_net_path.empty())
    {
        ROS_ERROR("[ Map Viewer ] route_net_path is empty.");
        return -1;
    } else{
        ROS_INFO("[ Map Viewer ] route_net_path: %s", route_net_path.c_str());
    }

    n.param<std::string>("point_cloud_path", point_cloud_path, "");
    ROS_INFO("[ Map Viewer ] point_cloud_path: %s", point_cloud_path.empty()?"not set":point_cloud_path.c_str());


    if(view_mode != "ABSOLUTE")
    {
        ros::spin();
    }
    else
    {
        ros::Publisher pub = n.advertise<visualization_msgs::MarkerArray>("/HDMap", 5, true);
        ros::Publisher globalmap_pub = n.advertise<sensor_msgs::PointCloud2>("/PublishPCD", 5, true);
        shared_ptr<Sender> p_sender(new Sender(pub));

        Map map;
        map.SetSender(p_sender);
        map.Load(route_net_path);

        pcl::PointCloud<PointT>::Ptr globalmap(new pcl::PointCloud<PointT>());
        pcl::io::loadPCDFile(point_cloud_path, *globalmap);
        globalmap->header.frame_id = p_sender->frame_id;

        char c;
        std::cout << "Input 'r' to view it on rviz or 'q' to quit\n";

        std::this_thread::sleep_for(std::chrono::seconds(2));

        map.Send();
        globalmap_pub.publish(globalmap);

        while (ros::ok())
        {
            std::cin >> c;
            if(c != 'q')
            {
                map.Send();
                globalmap_pub.publish(globalmap);
            }
            else
                break;
        }
    }
}
