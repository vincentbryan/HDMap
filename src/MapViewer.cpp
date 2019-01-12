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
    ros::NodeHandle n;

    if(argc != 4)
    {
        std::cout << "Usage: ./MapViewer map_file_path point_cloud_file_path" << std::endl;
        return 1;
    }

    if( strcmp("ABSOLUTE", argv[3])!=0 )
    {
        ros::spin();
    }
    else
    {
        ros::Publisher pub = n.advertise<visualization_msgs::MarkerArray>("HDMap", 5);
        ros::Publisher globalmap_pub = n.advertise<sensor_msgs::PointCloud2>("/PublishPCD", 5, true);
        shared_ptr<Sender> p_sender(new Sender(pub));

        Map map;
        map.SetSender(p_sender);
        map.Load(argv[1]);

        pcl::PointCloud<PointT>::Ptr globalmap(new pcl::PointCloud<PointT>());
        pcl::io::loadPCDFile(argv[2], *globalmap);
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
