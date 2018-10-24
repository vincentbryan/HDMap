//
// Created by vincent on 18-10-23.
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

using namespace hdmap;
using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_generator");

    if(argc != 2)
    {
        std::cout << "Usage: ./MapGenerator file_name" << std::endl;
        return 1;
    }

    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<visualization_msgs::MarkerArray>("HDMap", 1000);
    shared_ptr<Sender> p_sender(new Sender(pub));

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

    map.Save(argv[1]);
    std::cout << "Map is saved to" << argv[1] << std::endl;

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