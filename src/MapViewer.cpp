//
// Created by vincent on 18-11-12.
//

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include "Tool/Sender.h"
#include "Type/Map.h"

using namespace std;
using namespace hdmap;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_viewer");
    ros::NodeHandle n;

    if(argc < 2)
    {
        std::cout << "Usage: ./MapViewer file_name" << std::endl;
        return 1;
    }
    /// !!! 用于launch文件 !!!
    if(argc==3 && strcmp("ABSOLUTE",argv[2])!=0)
    {
        ros::spin();
    }
    else
    {
        ros::Publisher pub = n.advertise<visualization_msgs::MarkerArray>("HDMap", 5000);
        shared_ptr<Sender> p_sender(new Sender(pub));

        Map map;
        map.SetSender(p_sender);
        map.Load(argv[1]);

        char c;
        std::cout << "Input 'r' to view it on rviz or 'q' to quit\n";

        while (std::cin >> c)
        {
            if(c != 'q')
                map.Send();
            else
                break;
        }
    }


}
