//
// Created by vincent on 18-10-18.
//

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "Location.h"
#include "../Type/Angle.h"
#include "../Type/Pose.h"
#include "Sender.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <fstream>

/*
class Listener
{
public:
    ros::Publisher pub;
    std::vector<std::pair<double, double>> poses;
    int id = 0;


    void CallBack(const Map::Location & msg)
    {
        hdmap::Angle a;
        a.FromYaw(msg.yaw);
        ROS_INFO("[%6.3f %6.3f %6.3f]", msg.x, msg.y, a.Value());

        std::pair<double, double> p(msg.x, msg.y);
        poses.emplace_back(p);

        if(poses.size() > 10)
        {
            visualization_msgs::Marker points;
            points.header.frame_id = "/hdmap";
            points.header.stamp = ros::Time::now();
            points.action = visualization_msgs::Marker::ADD;

            points.pose.orientation.w = 0;
            points.id = id++;
            points.type = visualization_msgs::Marker::POINTS;

            points.scale.x = 0.08;
            points.scale.y = 0.08;
            points.color.g = 1.0f;
            points.color.a = 1.0;

            geometry_msgs::Point p1;
            p1.x = poses.front().first;
            p1.y = poses.front().second;
            p1.z = 0;
            points.points.push_back(p1);

            geometry_msgs::Point p2;
            p2.x = poses.back().first;
            p2.y = poses.back().second;
            p2.z = 0;
            points.points.push_back(p2);

            pub.publish(points);
            poses.clear();
        }
    }
};
*/

static double dist_threshold = 5.0;
static int anchor_id = 0;

void AnchorProcess(hdmap::Sender & sender, std::vector<hdmap::Pose> & anchors, hdmap::Pose anchor)
{
    sender.AddAnchor(anchor, anchor_id);
    anchors.emplace_back(anchor);
    sender.Send();

    anchor_id++;

}

//TODO roslaunch
int main(int argc, char **argv)
{
    ros::init(argc, argv, "DataProcessNode");
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<visualization_msgs::MarkerArray>("DataProcess", 1000);
    hdmap::Sender sender(pub);

    rosbag::Bag bag;
    bag.open("/media/vincent/DATA/Ubuntu/Project/catkin_ws/src/Map/data/2018-10-17-21-20-44.bag");
    std::vector<hdmap::Pose> anchors;

    bool has_guiding_anchor = false;
    hdmap::Pose guiding_anchor;
    std::queue<hdmap::Pose> buffer;

    std::cout << "Input s to start\n";
    char c;
    while (std::cin >> c)
    {
        if(c == 's') break;
    }

    for(rosbag::MessageInstance const & m : rosbag::View(bag))
    {
        HDMap::Location::ConstPtr p = m.instantiate<HDMap::Location>();
        if(p != nullptr)
        {
            hdmap::Angle angle;
            angle.FromYaw(p->yaw);
            hdmap::Pose pose(p->x, p->y, angle.Value());

            if(buffer.empty() || buffer.front().DistanceFrom(buffer.back()) < dist_threshold)
            {
                if(buffer.size() >= 2)
                {
                    if(hdmap::Pose::InLine(buffer.front(), buffer.back()))
                        buffer.push(pose);
                    else
                        buffer = std::queue<hdmap::Pose>();
                }
                else buffer.push(pose);
            }

            if(!buffer.empty() && buffer.front().DistanceFrom(buffer.back()) >= dist_threshold)
            {
                if(hdmap::Pose::InLine(buffer.front(), buffer.back()))
                {
                    if(!has_guiding_anchor)
                    {
                        guiding_anchor = buffer.front();
                        has_guiding_anchor = true;
                        AnchorProcess(sender, anchors, guiding_anchor);
                    }

                    while (buffer.front().DistanceFrom(buffer.back()) >= dist_threshold)
                    {
                        buffer.pop();
                    }
                }
                else
                {
                    if(has_guiding_anchor)
                    {
                        hdmap::Pose curr = buffer.front(); buffer.pop();
                        hdmap::Pose prev = curr;
                        while(!buffer.empty() && hdmap::Pose::InLine(guiding_anchor, curr))
                        {
                            prev = curr;
                            curr = buffer.front(); buffer.pop();
                        }
                        hdmap::Pose anchor = prev;
                        has_guiding_anchor = false;
                        AnchorProcess(sender, anchors, anchor);
                    }
                    buffer = std::queue<hdmap::Pose>();//clear
                }
            }
        }
    }

    if(!buffer.empty())
    {
        hdmap::Pose anchor;
        if(hdmap::Pose::InLine(buffer.front(), buffer.back()))
        {
            anchor = buffer.back();
        }
        else
        {
            anchor = buffer.front();
        }
        AnchorProcess(sender, anchors, anchor);
    }

    bag.close();

    std::ofstream os("/media/vincent/DATA/Ubuntu/Project/catkin_ws/src/Map/data/out.txt");
    if(os.is_open())
    {
        for(int i = 0; i < anchors.size(); i++)
        {
            os << "anchor [" << i << "]: " << anchors[i].x << " "<< anchors[i].y << " " << anchors[i].GetAngle().Value() << std::endl;
        }
        os.close();
    }
    return 0;
}