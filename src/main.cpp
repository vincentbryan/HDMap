//
// Created by vincent on 18-10-7.
//
#include <ros/ros.h>
#include <thread>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose2D.h>
#include <queue>
#include <mutex>
#include "Type/HDMap.h"
#include "Sender.h"
#include "Math/Bezier.h"
#include "Math/CubicFunction.h"
#define TEST

using namespace hdmap;
using namespace std;

bool CMD_AddRoad(HDMap &map);
bool CMD_StartSection(HDMap &map);
bool CMD_EndSection(HDMap &map);
bool CMD_SendSection(HDMap &map, Sender &sender);

int main( int argc, char** argv )
{
    ros::init(argc, argv, "hdmap");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<visualization_msgs::MarkerArray>("HDMap", 1000);
    Sender sender(pub);

#ifdef CMD
    HDMap map;
    while(true)
    {
        CMD_AddRoad(map);

        while(true)
        {
            CMD_StartSection(map);
            CMD_EndSection(map);
            CMD_SendSection(map, sender);
            cout << "Continue to add section?[y/n]\n";
            cout << "[User]: ";
            string cmd;
            cin >> cmd;
            if(cmd == "n")
                break;
        }

        cout << "Continue to add road?[y/n]\n";
        cout << "[User]: ";
        string cmd;
        cin >> cmd;
        if(cmd == "n")
            break;
    }
#endif

#ifdef TEST

    HDMap map;

    //Road[0]-------------------------------------------------------------
    map.StartRoad();
    map.SetCurrPose({0, 0, 0});
    sender.AddRoadId(map.GetCurrPose(), map.GetCurrentRoadId());

    //region sec-0
    std::tuple<int, double, double> t1 (-2, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
    std::tuple<int, double, double> t2 (-1, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
    std::tuple<int, double, double> t3 ( 1, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
    std::vector<std::tuple<int, double, double>> lanes1;
    lanes1.emplace_back(t1);
    lanes1.emplace_back(t2);
    lanes1.emplace_back(t3);
    std::vector<std::pair<int, int>> links1;

    map.StartSection(lanes1, links1);
    map.EndSection({10, 0, 0});
    sender.AddSection(map.GetCurrentSection());
    //endregion

    //region sec-1
    std::tuple<int, double, double> t4 (-2, Lane::DEFAULT_WIDTH, 0);
    std::tuple<int, double, double> t5 (-1, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
    std::tuple<int, double, double> t6 ( 1, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
    std::vector<std::tuple<int, double, double>> lanes2;
    lanes2.emplace_back(t4);
    lanes2.emplace_back(t5);
    lanes2.emplace_back(t6);
    std::vector<std::pair<int, int>> links2 = {{-2, -2}, {-1, -1}, {1, 1}};

    map.StartSection(lanes2, links2);
    map.EndSection({20, 0, 0});
    sender.AddSection(map.GetCurrentSection());
    //endregion

    map.EndRoad();

    //Road[1]-------------------------------------------------------------
    map.StartRoad();
    map.SetCurrPose({25, 1, 0});
    sender.AddRoadId(map.GetCurrPose(), map.GetCurrentRoadId());

    //region sec-10
    std::vector<std::tuple<int, double, double>> lanes3;
    std::tuple<int, double, double> t7 (-1, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
    std::tuple<int, double, double> t8 (1, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
    lanes3.emplace_back(t7);
    lanes3.emplace_back(t8);

    std::vector<std::pair<int, int>> links3;
    map.StartSection(lanes3, links3);

    map.EndSection({45, 1, 0});
    sender.AddSection(map.GetCurrentSection());
    //endregion
    map.EndRoad();

    //Road[2]-------------------------------------------------------------

    map.StartRoad();
    map.SetCurrPose({50, -3, 270});
    sender.AddRoadId(map.GetCurrPose(), map.GetCurrentRoadId());

    std::vector<std::tuple<int, double, double>> lanes4;
    std::tuple<int, double, double> t9 (-1, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
    std::tuple<int, double, double> t10 (1, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
    lanes4.emplace_back(t9);
    lanes4.emplace_back(t10);

    std::vector<std::pair<int, int>> links4;
    map.StartSection(lanes4, links4);

    map.EndSection({50, -16, 270});
    sender.AddSection(map.GetCurrentSection());
    map.EndRoad();

    //Road[3]-------------------------------------------------------------

    map.StartRoad();
    map.SetCurrPose({45, -20, 180});
    sender.AddRoadId(map.GetCurrPose(), map.GetCurrentRoadId());

    std::vector<std::tuple<int, double, double>> lanes5;
    std::tuple<int, double, double> t11 (-1, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
    std::tuple<int, double, double> t12 (1, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
    lanes5.emplace_back(t11);
    lanes5.emplace_back(t12);

    std::vector<std::pair<int, int>> links5;
    map.StartSection(lanes5, links5);

    map.EndSection({25, -20, 180});
    sender.AddSection(map.GetCurrentSection());
    map.EndRoad();

    //Road[4]-------------------------------------------------------------

    map.StartRoad();
    map.SetCurrPose({22, -16, 90});
    sender.AddRoadId(map.GetCurrPose(), map.GetCurrentRoadId());

    std::vector<std::tuple<int, double, double>> lanes6;
    std::tuple<int, double, double> t13 (-1, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
    std::tuple<int, double, double> t14 (1, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
    lanes6.emplace_back(t13);
    lanes6.emplace_back(t14);

    std::vector<std::pair<int, int>> links6;
    map.StartSection(lanes5, links6);

    map.EndSection({22, -3, 90});
    sender.AddSection(map.GetCurrentSection());
    map.EndRoad();

    //-------------------------------------------------------------

    map.AddJunction();
    map.AddConnection(0, 1, 1, 1);
    map.AddConnection(0, 1, 4, -1, 1.0, 0.5);

    map.AddConnection(1, -1, 0, -1);
    map.AddConnection(1, -1, 0, -2);
    map.AddConnection(1, -1, 4, -1);

    map.AddConnection(4, 1, 1, 1);
    map.AddConnection(4, 1, 0, -1);
    map.AddConnection(4, 1, 0, -2);
    sender.AddJunction(map.GetCurrentJunction());

    map.AddJunction();
    map.AddConnection(1, 1, 2, 1);
    map.AddConnection(2, -1, 1, -1, 3.0, 3.0);
    sender.AddJunction(map.GetCurrentJunction());

    map.AddJunction();
    map.AddConnection(2, 1, 3, 1);
    map.AddConnection(3, -1, 2, -1,3.0, 3.0);
    sender.AddJunction(map.GetCurrentJunction());

    map.AddJunction();
    map.AddConnection(3, 1, 4, 1);
    map.AddConnection(4, -1, 3, -1, 3.0, 3.0);
    sender.AddJunction(map.GetCurrentJunction());



    //-------------------------------------------------------------

    map.Summary();

    char c;
    while (std::cin >> c)
    {
        if(c != 'e')
            sender.Send();
        else
            break;
    }

#endif

#ifdef XML
    HDMap map;
    map.Load("/media/vincent/DATA/Ubuntu/Project/catkin_ws/src/HDMap/data/test.xml");
    map.Summary();
    sender.AddMap(map);

    char c;
    while(cin >> c)
    {
        if(c == 'e')break;
        sender.Send();
    }
#endif

#ifdef BEZIER
    CubicFunction func(3, 5, 3);
    vector<Pose> res;
    for(double s = 0; s < 5.0; s += 0.1)
    {
        Pose(s, func.Value(s), 0);
        res.emplace_back(Pose(s, func.Value(s), 0));
    }
    char c;
    while(cin >> c)
    {
        if(c != 'e')
            sender.SendPoses(res);
    }

//    Pose start(0, 0, 270);
//    Pose end(5, 5, 270);
//
//    Bezier b(start, end);
//    cout << b.Length() << endl;
//    for(auto p : b.GetAllPose(0.05))
//    {
//        cout << p.x << " " << p.y << " " << p.yaw << endl;
//    }
//    char c;
//    while(cin >> c)
//    {
//        if(c != 'e')
//            sender.SendPoses(b.GetAllPose(0.05));
//    }
#endif
    return 0;
}

bool CMD_AddRoad(HDMap &map)
{
    cout << "To start a road, you need to input a start pose[x, y, yaw]:\n";
    cout << "[User]: ";
    Pose p;
    std::cin >> p.x >> p.y >> p.direction;
    map.StartRoad();
    map.SetCurrPose(p);
    cout << "Summary: Init road successfully\n";
    return true;
}

bool CMD_StartSection(HDMap &map)
{
    int n;
    cout << "To start a section, you need to specify the lane index and links\n";
    cout << "Please input the lane number\n";
    cout << "[User]: ";
    cin >> n;
    vector<tuple<int, double, double>> lanes;
    vector<pair<int, int>> links;
    for(int i = 0; i < n; i++)
    {
        int t;
        cout << "[User] lane " << i << " : ";
        cin >> t;
        //TODO
        tuple<int, double, double > p(t, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
        lanes.emplace_back(p);
    }
    cout << "Summary: \n";
    for(int i = 0; i < n; i++)
    {
        cout << "\tlane " << i << " : " << get<0>(lanes[i]) << " " << get<1>(lanes[i]) << endl;
    }

    cout << "Please input the links\n";
    cout << "Previous section lane index:\n";
    auto lane_info = map.GetCurrentSection().GetLanes();
    if(lane_info.empty())
    {
        cout << "empty\n";
    }
    else
    {
        for(auto x : lane_info)
        {
            cout << "\t[" << x.first << "]";
        }
        cout << endl;
    }

    cout << "Input the links number:" << endl;
    cout << "[User]: ";
    cin >> n;
    cout << "Please input " << n << " links\n";
    for(int i = 0; i < n; i++)
    {
        int a, b;
        cout << "[User]: ";
        cin >> a >> b;
        pair<int, int>p(a, b);
        links.emplace_back(p);
    }
    cout << "Summary:\n";
    cout << "\tTotal " << n << " links\n";
    for(auto x : links)
    {
        cout << "\t[" << x.first << "] --> [" << x.second << "]" << endl;
    }
    map.StartSection(lanes, links);
    return true;
}

bool CMD_EndSection(HDMap &map)
{
    cout << "To end a section, you need to input a end pose[x, y, yaw]:\n";
    cout << "[User]: ";
    Pose p;
    std::cin >> p.x >> p.y >> p.direction;
    map.EndSection(p);
    return true;
}

bool CMD_SendSection(HDMap &map, Sender &sender)
{
    sender.AddSection(map.GetCurrentSection());

    cout << "To send a new section, enter 'r', if failed enter it again\n";
    char c;
    cout << "[User]: ";
    cin >> c;
    while (c != 'e')
    {
        sender.Send();
        cout << "[User]: ";
        cin >> c;
    }
    return true;
}