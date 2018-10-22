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
#include "Type/Map.h"
#include "Tool/Sender.h"
#include "Math/Bezier.h"
#include "Math/CubicFunction.h"
//#define TEST
#define XML
#define USER    "\033[33m[User]: \033[0m"
#define HINT    "\033[31m[Hint]: \033[0m"

using namespace hdmap;
using namespace std;

bool CMD_StartRoad(Map &map);
bool CMD_StartSection(Map &map);
bool CMD_EndSection(Map &map);
bool CMD_EndRoad(Map &map);

bool CMD_StartJunction(Map &map);
bool CMD_EndJunction(Map &map);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hdmap");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<visualization_msgs::MarkerArray>("HDMap", 1000);
    shared_ptr<Sender> p_sender(new Sender(pub));

#ifdef CMD
    Map map;
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

        map.Save("/media/vincent/DATA/Ubuntu/Project/catkin_ws/src/Map/data/test2_out.xml");
    }
#endif

#ifdef TEST
    Map map;
    map.SetSender(p_sender);

    double w = 1.0;
    auto Offset = [&](Pose p) -> Pose
    {
        Angle a = p.GetAngle();
        a.Rotate(90);
        return p.GetTranslation(w, a);
    };


    //Road[0]-------------------------------------------------------------
    map.StartRoad(Offset({-221.360, 11.736, 27.150}));

    //region road[0] / sec[0]
    std::vector<std::tuple<int, double, double>> r0_s0_lanes;
    r0_s0_lanes.emplace_back(-2, 3.50, 3.50);
    r0_s0_lanes.emplace_back(-1, 4.00, 4.00);
    r0_s0_lanes.emplace_back( 1, 4.00, 4.00);
    r0_s0_lanes.emplace_back( 2, 3.50, 3.50);
    r0_s0_lanes.emplace_back( 3, 3.30, 3.30);
    std::vector<std::pair<int, int>> r0_s0_links;

    map.StartSection(r0_s0_lanes, r0_s0_links);
    map.EndSection(Offset({-80.123, 83.702, 27.03}));
    //endregion

    map.EndRoad();
    //--------------------------------------------------------------------



    //Road[1]-------------------------------------------------------------
    map.StartRoad(Offset({-42.530, 73.328, 304.10}));

    //region road[1] / sec[0]
    std::vector<std::tuple<int, double, double>> r1_s0_lanes;
    std::vector<std::pair<int, int>> r1_s0_links;
    r1_s0_lanes.emplace_back(-2, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
    r1_s0_lanes.emplace_back(-1, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
    r1_s0_lanes.emplace_back(1, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);

    map.StartSection(r1_s0_lanes, r1_s0_links);
    map.EndSection(Offset({-25.445, 47.668, 304.36}));
    //endregion

    //region road[1] / sec[1]
    std::vector<std::tuple<int, double, double>> r1_s1_lanes;
    std::vector<std::pair<int, int>> r1_s1_links = {{1, 1}, {1, 2}, {-1, -1}, {-2, -2}};

    r1_s1_lanes.emplace_back(-2, Lane::DEFAULT_WIDTH, 0);
    r1_s1_lanes.emplace_back(-1, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
    r1_s1_lanes.emplace_back( 1, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
    r1_s1_lanes.emplace_back( 2, 0, Lane::DEFAULT_WIDTH);

    map.StartSection(r1_s1_lanes, r1_s1_links);
    map.EndSection(Offset({-9.152, 28.882, 304.36}), 10.0, 10.0);
    //endregion

    //region road[1] / sec[2]
    std::vector<std::tuple<int, double, double>> r1_s2_lanes;
    std::vector<std::pair<int, int>> r1_s2_links = {{1, 1}, {2, 2}, {-1, -1}, {-2, -1}};

    r1_s2_lanes.emplace_back(-1, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
    r1_s2_lanes.emplace_back(1, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
    r1_s2_lanes.emplace_back(2, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);

    map.StartSection(r1_s2_lanes, r1_s2_links);
    map.EndSection(Offset({40.15, -43.230, 303.046}));
    //endregion

    map.EndRoad();
    //--------------------------------------------------------------------



    //Road[2]-------------------------------------------------------------

    map.StartRoad(Offset({69.40, -54.40, 28.30}));

    //region road[2] / sec[0]
    std::vector<std::tuple<int, double, double>> r2_s0_lanes;
    std::vector<std::pair<int, int>> r2_s0_links;

    r2_s0_lanes.emplace_back(-2, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
    r2_s0_lanes.emplace_back(-1, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
    r2_s0_lanes.emplace_back(1, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);

    map.StartSection(r2_s0_lanes, r2_s0_links);
    map.EndSection(Offset({98.933, -37.35, 28.95}));
    //endregion

    //region road[2] / sec[1]
    std::vector<std::tuple<int, double, double>> r2_s1_lanes;
    std::vector<std::pair<int, int>> r2_s1_links = {{-2, -2}, {-1, -1}, {1, 1}};
    r2_s1_lanes.emplace_back(-2, Lane::DEFAULT_WIDTH, 0);
    r2_s1_lanes.emplace_back(-1, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
    r2_s1_lanes.emplace_back(1, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);

    map.StartSection(r2_s1_lanes, r2_s1_links);
    map.EndSection(Offset({115.21, -26.412, 29.90}), 10, 10);
    //endregion

    //region road[2] / sec[2]
    std::vector<std::tuple<int, double, double>> r2_s2_lanes;
    std::vector<std::pair<int, int>> r2_s2_links = {{-2, -1}, {-1, -1}, {1, 1}};
    r2_s2_lanes.emplace_back(-1, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
    r2_s2_lanes.emplace_back(1, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);

    map.StartSection(r2_s2_lanes, r2_s2_links);
    map.EndSection(Offset({166.287, 3.545, 30.60}));
    //endregion

    //region road[2] / sec[3]
    std::vector<std::tuple<int, double, double>> r2_s3_lanes;
    std::vector<std::pair<int, int>> r2_s3_links = {{1, 1}, {1, 2}, {1, 3}, {-1, -1}};
    r2_s3_lanes.emplace_back(-1, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
    r2_s3_lanes.emplace_back(1, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
    r2_s3_lanes.emplace_back(2, 0, Lane::DEFAULT_WIDTH);
    r2_s3_lanes.emplace_back(3, 0, Lane::DEFAULT_WIDTH);

    map.StartSection(r2_s3_lanes, r2_s3_links);
    map.EndSection(Offset({187.942, 19.417, 29.53}), 15.0, 15.0);
    //endregion

    //region road[2] / sec[4]

    std::vector<std::tuple<int, double, double>> r2_s4_lanes;
    std::vector<std::pair<int, int>> r2_s4_links = {{1, 1}, {2, 2}, {3, 3}, {-1, -1}};
    r2_s4_lanes.emplace_back(-1, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
    r2_s4_lanes.emplace_back(1, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
    r2_s4_lanes.emplace_back(2, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);
    r2_s4_lanes.emplace_back(3, Lane::DEFAULT_WIDTH, Lane::DEFAULT_WIDTH);

    map.StartSection(r2_s4_lanes, r2_s4_links);
    map.EndSection(Offset({216.912, 36.511, 29.30}));
    //endregion

    map.EndRoad();
    //--------------------------------------------------------------------



    //Road[3]-------------------------------------------------------------
    map.StartRoad(Offset({226.176, 70.656, 127.505}));

    //region road[3] / sec[0]
    std::vector<std::tuple<int, double, double>> r3_s0_lanes;
    std::vector<std::pair<int, int>> r3_s0_links;
    r3_s0_lanes.emplace_back(-3, 3.5, 3.5);
    r3_s0_lanes.emplace_back(-2, 3.8, 3.8);
    r3_s0_lanes.emplace_back(-1, 4.0, 4.0);
    r3_s0_lanes.emplace_back( 1, 4.0, 4.0);
    r3_s0_lanes.emplace_back( 2, 3.5, 3.5);

    map.StartSection(r3_s0_lanes, r3_s0_links);
    map.EndSection(Offset({147.307, 170.876, 126.747}));
    //endregion

    map.EndRoad();
    //--------------------------------------------------------------------



    //Road[4]-------------------------------------------------------------
    map.StartRoad(Offset({106.649, 181.677, 207.139}));

    //region road[4] / sec[0]
    std::vector<std::tuple<int, double, double>> r4_s0_lanes;
    std::vector<std::pair<int, int>> r4_s0_links;
    r4_s0_lanes.emplace_back(-2, 3.5, 3.5);
    r4_s0_lanes.emplace_back(-1, 4.0, 4.0);
    r4_s0_lanes.emplace_back( 1, 4.0, 4.0);
    r4_s0_lanes.emplace_back( 2, 3.5, 3.5);

    map.StartSection(r4_s0_lanes, r4_s0_links);
    map.EndSection(Offset({-31.914, 110.871, 205.507}));
    //endregion

    map.EndRoad();
    //--------------------------------------------------------------------

    //region junc0
    map.StartJunction();

    map.AddConnection(0, 1, 4, -1, 10.0, 10.0);
    map.AddConnection(0, 2, 4, -2, 10.0, 10.0);
    map.AddConnection(0, 3, 1, 1, 10.0, 10.0);

    map.AddConnection(1, -1, 0, -1, 15.0, 15.0);
    map.AddConnection(1, -1, 0, -2, 15.0, 15.0);
    map.AddConnection(1, -2, 4, -1, 15.0, 15.0);
    map.AddConnection(1, -2, 4, -2, 15.0, 15.0);

    map.AddConnection(4, 1, 0, -1, 15.0, 15.0);
    map.AddConnection(4, 1, 1,  1, 15.0, 15.0);
    map.AddConnection(4, 2, 0, -2, 15.0, 15.0);
    map.EndJunction();
    //endregion

    //region junc1
    map.StartJunction();
    map.AddConnection(1, 1, 2, 1, 15.0, 15.0);
    map.AddConnection(2, -2, 1, -1, 15.0, 15.0);
    map.EndJunction();
    //endregion

    //region junc2
    map.StartJunction();
    map.AddConnection(2, 1, 3, 1, 15.0, 15.0);
    map.AddConnection(2, 1, 3, 2, 15.0, 15.0);

    map.AddConnection(3, -3, 2, -1, 10.0, 10.0);
    map.EndJunction();
    //endregion

    //region junc3
    map.StartJunction();
    map.AddConnection(3, 1, 4, 1, 15.0, 15.0);
    map.AddConnection(3, 1, 4, 2, 15.0, 15.0);

    map.AddConnection(4, -2, 3, -1, 10.0, 15.0);
    map.AddConnection(4, -2, 3, -2, 10.0, 15.0);
    map.AddConnection(4, -2, 3, -3, 10.0, 15.0);
    map.EndJunction();
    //endregion

    map.SetStartPoint({226.176, 70.656});
    map.SetEndPoint({106.649, 181.677});
    map.GlobalPlanning();

    map.Save("/media/vincent/DATA/Ubuntu/Project/catkin_ws/src/HDMap/data/test2_out.xml");
//    map.Test();

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
    Map map;
    map.SetSender(p_sender);
    map.Load("/media/vincent/DATA/Ubuntu/Project/catkin_ws/src/HDMap/data/test2_out.xml");
//    map.Summary();

    map.SetStartPoint({106.649, 181.677});
    map.SetEndPoint({226.176, 70.656});

    map.GlobalPlanning();
    char c;
    while (std::cin >> c)
    {
        if(c != 'e')
            map.Send();
        else
            break;
    }
//    ros::ServiceServer server = n.advertiseService("local_map", &Map::OnRequest, &map);
//    ROS_INFO("HDMap is ready...");
//    ros::spin();

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

bool CMD_StartRoad(Map &map)
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

bool CMD_StartSection(Map &map)
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

bool CMD_EndSection(Map &map)
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

bool CMD_EndRoad(Map &map)
{
    cout << "Successfully added a road..." << endl;
    map.EndRoad();
}

bool CMD_StartJunction(Map &map)
{
    map.StartJunction();
}

bool CMD_EndJunction(Map &map)
{
    map.EndJunction();
}