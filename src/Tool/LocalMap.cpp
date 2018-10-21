//
// Created by vincent on 18-10-21.
//

#include <ros/ros.h>
#include <LocalMap.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "LocalMap");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<HDMap::LocalMap>("local_map");
    while(true)
    {
        std::string cmd;
        std::cin >> cmd;
        if(cmd == "r")
        {
            HDMap::LocalMap srv;
            srv.request.x = 0;
            srv.request.y = 0;

            if(client.call(srv))
            {
                ROS_INFO("recv: %d\t%d\t%d\t%d\t%d",
                         srv.response.curr_road.lanes.size(),
                         srv.response.junction.in.size(),
                         srv.response.junction.out.size(),
                         srv.response.junction.conns.size(),
                         srv.response.next_road.lanes.size());
            }
            else
            {
                ROS_ERROR("Failed");
            }
        }
        else if(cmd == "e")
        {
            break;
        }
    }
}