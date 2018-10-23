//
// Created by vincent on 18-10-21.
//

#include <ros/ros.h>
#include <LocalMap.h>
#include <rosbag/bag.h>
#include "LocalMapRequest.h"
#include "LocalMapResponse.h"
#include "Sender.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <Location.h>

class LocalMap
{
public:
    std::shared_ptr<hdmap::Sender> pSender;

    bool hasMap;
    std::vector<HDMap::Pose2DArray> curr_roads;
    std::vector<HDMap::Pose2DArray> next_roads;
    std::vector<int64_t> in;
    std::vector<int64_t> out;
    std::vector<HDMap::Pose2DArray> conns;

    LocalMap() : hasMap(false){};

    void Send()
    {
        for(auto & ps : curr_roads)
        {
            std::vector<hdmap::Pose> v;
            for(auto & p : ps.lane)
            {
                hdmap::Angle a;
                a.FromYaw(p.theta);
                v.emplace_back(hdmap::Pose({p.x, p.y}, a));
            }
            pSender->SendPoses(v, 0, 1.0, 0, 1.0, 1.0, 0.10);
        }

        for(auto & ps: next_roads)
        {
            std::vector<hdmap::Pose> v;
            for(auto & p : ps.lane)
            {
                hdmap::Angle a;
                a.FromYaw(p.theta);
                v.emplace_back(hdmap::Pose({p.x, p.y}, a));
            }
            pSender->SendPoses(v, 0, 1.0, 0, 1.0, 1.0, 0.10);
        }

        for(auto & ps : conns)
        {
            std::vector<hdmap::Pose> v;
            for(auto & c : ps.lane)
            {
                hdmap::Angle a;
                a.FromYaw(c.theta);
                v.emplace_back(hdmap::Pose({c.x, c.y}, a));
            }
            pSender->SendPoses(v, 0, 1.0, 0, 1.0, 1.0, 0.10);
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "LocalMap");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<HDMap::LocalMap>("local_map");
    ros::Publisher pub = n.advertise<visualization_msgs::MarkerArray>("t_local_map", 1000);
    std::shared_ptr<hdmap::Sender> p_sender(new hdmap::Sender(pub));
    LocalMap map;
    map.pSender = p_sender;

    rosbag::Bag bag;
    bag.open("/media/vincent/DATA/Ubuntu/Project/catkin_ws/src/HDMap/data/2018-10-17-21-20-44.bag");
    ros::Rate r(10);

    for(rosbag::MessageInstance const & m : rosbag::View(bag))
    {
        HDMap::Location::ConstPtr p = m.instantiate<HDMap::Location>();
        if(p != nullptr)
        {
            HDMap::LocalMap srv;
            srv.request.x = p->x;
            srv.request.y = p->y;
            srv.request.has_local_map = map.hasMap;
            if(client.call(srv))
            {
                ROS_INFO("recv: %d  %d  %d  %d  %d",
                         srv.response.curr_road.lanes.size(),
                         srv.response.junction.in.size(),
                         srv.response.junction.out.size(),
                         srv.response.junction.conns.size(),
                         srv.response.next_road.lanes.size());
                map.hasMap = true;
                if(srv.response.need_update)
                {
                    map.curr_roads = srv.response.curr_road.lanes;
                    map.next_roads = srv.response.next_road.lanes;
                    map.in = srv.response.junction.in;
                    map.out = srv.response.junction.out;
                    map.conns = srv.response.junction.conns;
                    map.Send();
                }
            }
            else
            {
                ROS_ERROR("Failed");
            }
        }
        r.sleep();
    }
/*
    while(true)
    {
        std::string cmd;
        std::cin >> cmd;
        if(cmd == "r")
        {
            HDMap::LocalMap srv;
            srv.request.x = 106.649;
            srv.request.y = 181.677;
            srv.request.has_local_map = map.hasMap;

            if(client.call(srv))
            {
                ROS_INFO("recv: %d  %d  %d  %d  %d",
                         srv.response.curr_road.lanes.size(),
                         srv.response.junction.in.size(),
                         srv.response.junction.out.size(),
                         srv.response.junction.conns.size(),
                         srv.response.next_road.lanes.size());
                map.hasMap = true;
                if(srv.response.need_update)
                {
                    map.curr_roads = srv.response.curr_road.lanes;
                    map.next_roads = srv.response.next_road.lanes;
                    map.in = srv.response.junction.in;
                    map.out = srv.response.junction.out;
                    map.conns = srv.response.junction.conns;
                }
                map.Send();
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

    */
}