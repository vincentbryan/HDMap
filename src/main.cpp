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
#include "Tool/Sender.h"
#include "Math/Bezier.h"
#include "Math/CubicFunction.h"
#define TEST
//#define XML
#define USER    "\033[33m[User]: \033[0m"
#define HINT    "\033[31m[Hint]: \033[0m"

using namespace hdmap;
using namespace std;

bool CMD_StartRoad(HDMap &map);
bool CMD_StartSection(HDMap &map);
bool CMD_EndSection(HDMap &map);
bool CMD_EndRoad(HDMap &map);

bool CMD_StartJunction(HDMap &map);
bool CMD_EndJunction(HDMap &map);

int main( int argc, char** argv )
{
    ros::init(argc, argv, "hdmap");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<visualization_msgs::MarkerArray>("HDMap", 1000);
    shared_ptr<Sender> p_sender(new Sender(pub));

#ifdef CMD
    HDMap map;
    map.SetSender(p_sender);

    while(true)
    {
        CMD_StartRoad(map);

        while(true)
        {
            CMD_StartSection(map);
            CMD_EndSection(map);
            cout << "Continue to add section?\n";
            cout << HINT << "[y/n]\n";
            cout << USER;
            string cmd;
            cin >> cmd;
            if(cmd == "n")
                break;
        }

        CMD_EndRoad(map);

        cout << "Continue to add road?\n";
        cout << HINT << "[y/n]\n";
        cout << USER;
        string cmd;
        cin >> cmd;
        if(cmd == "n")
            break;
    }

    while(true)
    {
        std::cout << "Adding Junction?\n";
        cout << HINT << "[y/n]\n";
        std::cout << USER;
        string cmd;
        cin >> cmd;
        if(cmd == "y")
        {
            CMD_StartJunction(map);
            while(true)
            {
                string c;
                std::cout << "Add a connection?\n";
                std::cout << HINT << "[y/n]\n";
                std::cout << USER;
                cin >> c;
                if(c == "y")
                {
                    std::cout << "To add connection, you need to input following data\n";
                    std::cout << HINT << "[from_road_id idx to_road_id idx]\n";
                    std::cout << USER;
                    unsigned int from_road_id, to_road_id;
                    int from_lane_idx, to_lane_idx;
                    cin >> from_road_id >> from_lane_idx >> to_road_id >> to_lane_idx;
                    map.AddConnection(from_road_id, from_lane_idx, to_road_id, to_lane_idx);
                }
                else
                {
                    CMD_EndJunction(map);
                    map.Send();
                    break;
                }
            }
        }
        else
        {
            break;
        }

        map.Save("/media/vincent/DATA/Ubuntu/Project/catkin_ws/src/HDMap/data/test2_out.xml");
    }
#endif

#ifdef TEST
    HDMap map;
    map.SetSender(p_sender);

    //Road[0]-------------------------------------------------------------
    map.StartRoad({-240.809, 1.27231, 26.4284});

    //region road[0] / sec[0]
    std::tuple<int, double, double> r0_s0_t_1 (-1, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
    std::tuple<int, double, double> r0_s0_t1 ( 1, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
    std::vector<std::tuple<int, double, double>> r0_s0_lanes;
    r0_s0_lanes.emplace_back(r0_s0_t_1);
    r0_s0_lanes.emplace_back(r0_s0_t1);
    std::vector<std::pair<int, int>> r0_s0_links;

    map.StartSection(r0_s0_lanes, r0_s0_links);
    map.EndSection({115.543, 182.945, 24.6765});
    //endregion

    map.EndRoad();
    //--------------------------------------------------------------------



    //Road[1]-------------------------------------------------------------
    map.StartRoad({143.963, 173.71, 306.621});

    //region road[1] / sec[0]
    std::vector<std::tuple<int, double, double>> r1_s0_lanes;
    std::vector<std::pair<int, int>> r1_s0_links;

    std::tuple<int, double, double> r1_s0_t_1 (-1, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
    std::tuple<int, double, double> r1_s0_t1 (1, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
    r1_s0_lanes.emplace_back(r1_s0_t_1);
    r1_s0_lanes.emplace_back(r1_s0_t1);

    map.StartSection(r1_s0_lanes, r1_s0_links);
    map.EndSection({225.937, 65.9142, 304.875});
    //endregion

    map.EndRoad();
    //--------------------------------------------------------------------



    //Road[2]-------------------------------------------------------------

    map.StartRoad({218.451, 39.7093, 207.731});

    //region road[2] / sec[0]
    std::vector<std::tuple<int, double, double>> r2_s0_lanes;
    std::vector<std::pair<int, int>> r2_s0_links;
    std::tuple<int, double, double> r2_s0_t_1 (-1, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
    std::tuple<int, double, double> r2_s0_t1 (1, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
    r2_s0_lanes.emplace_back(r2_s0_t_1);
    r2_s0_lanes.emplace_back(r2_s0_t1);

    map.StartSection(r2_s0_lanes, r2_s0_links);
    map.EndSection({191.272, 24.1459, 210.586});
    //endregion

    //region road[2] / sec[1]
    std::vector<std::tuple<int, double, double>> r2_s1_lanes;
    std::vector<std::pair<int, int>> r2_s1_links = {{1, 1}, {-1, -1}};
    std::tuple<int, double, double> r2_s1_t_1 (-1, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
    std::tuple<int, double, double> r2_s1_t1 (1, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
    r2_s1_lanes.emplace_back(r2_s1_t_1);
    r2_s1_lanes.emplace_back(r2_s1_t1);

    map.StartSection(r2_s1_lanes, r2_s1_links);
    map.EndSection({176.956, 14.2475, 218.465}, 5.0, 5.0);
    //endregion

    //region road[2] / sec[2]
    std::vector<std::tuple<int, double, double>> r2_s2_lanes;
    std::vector<std::pair<int, int>> r2_s2_links = {{1, 1}, {-1, -1}};
    std::tuple<int, double, double> r2_s2_t_1 (-1, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
    std::tuple<int, double, double> r2_s2_t1 (1, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
    r2_s2_lanes.emplace_back(r2_s2_t_1);
    r2_s2_lanes.emplace_back(r2_s2_t1);

    map.StartSection(r2_s2_lanes, r2_s2_links);
    map.EndSection({69.328, -52.1073, 209.138});
    //endregion

    map.EndRoad();
    //--------------------------------------------------------------------



    //Road[3]-------------------------------------------------------------
    map.StartRoad({44.6509, -45.716, 124.36});

    //region road[3] / sec[0]
    std::vector<std::tuple<int, double, double>> r3_s0_lanes;
    std::vector<std::pair<int, int>> r3_s0_links;
    std::tuple<int, double, double> r3_s0_t_1 (-1, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
    std::tuple<int, double, double> r3_s0_t1 (1, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
    r3_s0_lanes.emplace_back(r3_s0_t_1);
    r3_s0_lanes.emplace_back(r3_s0_t1);

    map.StartSection(r3_s0_lanes, r3_s0_links);
    map.EndSection({22.8185, -14.4651, 126.85});
    //endregion

    //region road[3] / sec[1]
    std::vector<std::tuple<int, double, double>> r3_s1_lanes;
    std::vector<std::pair<int, int>> r3_s1_links = {{1, 1}, {-1, -1}};
    std::tuple<int, double, double> r3_s1_t_1 (-1, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
    std::tuple<int, double, double> r3_s1_t1 (1, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
    r3_s1_lanes.emplace_back(r3_s1_t_1);
    r3_s1_lanes.emplace_back(r3_s1_t1);

    map.StartSection(r3_s1_lanes, r3_s1_links);
    map.EndSection({15.0057, -4.23992, 125.589});
    //endregion

    //region road[3] / sec[2]
    std::vector<std::tuple<int, double, double>> r3_s2_lanes;
    std::vector<std::pair<int, int>> r3_s2_links = {{1, 1}, {-1, -1}};
    std::tuple<int, double, double> r3_s2_t_1 (-1, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
    std::tuple<int, double, double> r3_s2_t1 (1, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
    r3_s2_lanes.emplace_back(r3_s2_t_1);
    r3_s2_lanes.emplace_back(r3_s2_t1);

    map.StartSection(r3_s2_lanes, r3_s2_links);
    map.EndSection({-41.1274, 75.0404, 123.11});
    //endregion

    map.EndRoad();
    //--------------------------------------------------------------------


    //--------------------------------------------------------------------
    map.StartJunction();
    map.AddConnection(0, 1, 1, 1, 15.0, 15.0);
    map.AddConnection(1, -1, 0, -1, 15.0, 15.0);
    map.EndJunction();


    map.StartJunction();
    map.AddConnection(1, 1, 2, 1, 15.0, 15.0);
    map.AddConnection(2, -1, 1, -1, 15.0, 15.0);
    map.EndJunction();

    map.StartJunction();
    map.AddConnection(2, 1, 3, 1, 15.0, 15.0);
    map.AddConnection(3, -1, 2, -1, 15.0, 15.0);
    map.EndJunction();
    //-------------------------------------------------------------

//    map.SetStartPoint({1, 0});
//    map.SetEndPoint({50, -10.0});
//    map.GlobalPlanning();

//    map.Save("/media/vincent/DATA/Ubuntu/Project/catkin_ws/src/HDMap/data/test2_out.xml");
    map.Test();

//    map.Summary();
    char c;
    while (std::cin >> c)
    {
        if(c != 'e')
            map.Send();
        else
            break;
    }
#endif

#ifdef XML
    HDMap map;
    map.SetSender(p_sender);
    map.Load("/media/vincent/DATA/Ubuntu/Project/catkin_ws/src/HDMap/data/test2_out.xml");
    map.Summary();

    char c;
    while(cin >> c)
    {
        if(c == 'e')break;
        map.Send();
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

bool CMD_StartRoad(HDMap &map)
{
    cout << "To start a road, you need to input a start pose:\n";
    cout << HINT << "[x, y, angle]\n";
    cout << USER;
    Pose p;
    std::cin >> p.x >> p.y >> p.direction;
    map.StartRoad(p);
    cout << "Summary: Init road successfully\n";
    return true;
}

bool CMD_StartSection(HDMap &map)
{
    int n;
    cout << "To start a section, you need to specify the lane index and links\n";
    cout << "Please input the lane number\n";
    cout << HINT << "[unsigned int]\n";
    cout << USER;
    cin >> n;
    vector<tuple<int, double, double>> lanes;
    vector<pair<int, int>> links;
    for(int i = 0; i < n; i++)
    {
        int t;
        double d1, d2;
        cout << USER;
        cout << " lane" << i << " : ";
        cin >> t >> d1 >> d2;
        //TODO
        tuple<int, double, double > p(t, d1, d2);
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
    cout << HINT << "[unsigned int]\n";
    cout << USER;
    cin >> n;
    cout << "Please input " << n << " links\n";
    for(int i = 0; i < n; i++)
    {
        int a, b;
        cout << HINT << "[int int]\n";
        cout << USER;
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
    cout << "To end a section, you need to input a end pose:\n";
    cout << HINT << "[x y angle]\n";
    cout << USER;
    Pose p;
    std::cin >> p.x >> p.y >> p.direction;
    map.EndSection(p);
    map.Send();
    return true;
}

bool CMD_EndRoad(HDMap &map)
{
    cout << "Successfully added a road..." << endl;
    map.EndRoad();
}

bool CMD_StartJunction(HDMap &map)
{
    map.StartJunction();
}

bool CMD_EndJunction(HDMap &map)
{
    map.EndJunction();
}