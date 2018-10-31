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

    //region Road
    //Road[0]-------------------------------------------------------------
    auto r0 = map.AddRoad({-221.360, 11.736, 27.150});

    auto r0_s0 = r0->AddSection({-80.123, 83.702, 27.03});
    r0_s0->AddLane(-2,  7.50,  7.50, {}, {});
    r0_s0->AddLane(-1,  4.00,  4.00, {}, {});
    r0_s0->AddLane( 1,  4.00,  4.00, {}, {});
    r0_s0->AddLane( 2,  7.50,  7.50, {}, {});
    r0_s0->AddLane( 3, 10.80, 10.80, {}, {});

    r0->AddSignal({-55.123, 92.702}, 1, "SIG", "010");
    r0->AddSignal({-55.123, 90.702}, 1, "SIG", "001");


    //Road[1]-------------------------------------------------------------
    auto r1 = map.AddRoad({-42.530, 73.328, 304.10});

    auto r1_s0 = r1->AddSection({-25.445, 47.668, 304.36});
    r1_s0->AddLane(-2, 6.0, 6.0, {}, {-2});
    r1_s0->AddLane(-1, 3.0, 3.0, {}, {-1});
    r1_s0->AddLane( 1, 3.0, 3.0, {}, {1, 2});

    auto r1_s1 = r1->AddSection({-9.152, 28.882, 304.36}, 10.0, 10.0);
    r1_s1->AddLane(-2, 6.0, 3.0, {-2}, {-1});
    r1_s1->AddLane(-1, 3.0, 3.0, {-1}, {-1});
    r1_s1->AddLane( 1, 3.0, 3.0, {1}, {1});
    r1_s1->AddLane( 2, 3.0, 6.0, {1}, {2});

    auto r1_s2 = r1->AddSection({40.15, -43.230, 303.046});
    r1_s2->AddLane(-1, 3.0, 3.0, {-1, -2}, {});
    r1_s2->AddLane( 1, 3.0, 3.0, {1}, {});
    r1_s2->AddLane( 2, 6.0, 6.0, {2}, {});


    //Road[2]-------------------------------------------------------------
    auto r2 = map.AddRoad({69.40, -54.40, 28.30});

    auto r2_s0 = r2->AddSection({98.933, -37.35, 28.95});
    r2_s0->AddLane(-2, 6.0, 6.0, {}, {-2});
    r2_s0->AddLane(-1, 3.0, 3.0, {}, {-1});
    r2_s0->AddLane( 1, 3.0, 3.0, {}, {1});

    auto r2_s1 = r2->AddSection({115.21, -26.412, 29.90}, 10.0, 10.0);
    r2_s1->AddLane(-2, 6.0, 3.0, {-2}, {-1});
    r2_s1->AddLane(-1, 3.0, 3.0, {-1}, {-1});
    r2_s1->AddLane( 1, 3.0, 3.0, {1}, {1});

    auto r2_s2 = r2->AddSection({166.287, 3.545, 30.60});
    r2_s2->AddLane(-1, 3.0, 3.0, {-1, -2}, {-1});
    r2_s2->AddLane( 1, 3.0, 3.0, {1}, {1, 2, 3});

    auto r2_s3 = r2->AddSection({187.942, 19.417, 29.53}, 15.0, 15.0);
    r2_s3->AddLane(-1, 3.0, 3.0, {-1}, {-1});
    r2_s3->AddLane( 1, 3.0, 3.0, {1}, {1});
    r2_s3->AddLane( 2, 3.0, 6.0, {1}, {2});
    r2_s3->AddLane( 3, 3.0, 9.0, {1}, {3});

    auto r2_s4 = r2->AddSection({216.912, 36.511, 29.30});
    r2_s4->AddLane(-1, 3.0, 3.0, {-1}, {});
    r2_s4->AddLane( 1, 3.0, 3.0, {1}, {});
    r2_s4->AddLane( 2, 6.0, 6.0, {2}, {});
    r2_s4->AddLane( 3, 9.0, 9.0, {3}, {});


    //Road[3]-------------------------------------------------------------
    auto r3 = map.AddRoad({226.176, 70.656, 127.505});

    auto r3_s0 = r3->AddSection({147.307, 170.876, 126.747});
    r3_s0->AddLane(-3, 11.3, 11.3, {}, {});
    r3_s0->AddLane(-2, 7.8, 7.8, {}, {});
    r3_s0->AddLane(-1, 4.0, 4.0, {}, {});
    r3_s0->AddLane( 1, 4.0, 4.0, {}, {});
    r3_s0->AddLane( 2, 7.5, 7.5, {}, {});

    //Road[4]-------------------------------------------------------------
    auto r4 = map.AddRoad({106.649, 181.677, 207.139});

    auto r4_s0 = r4->AddSection({-31.914, 110.871, 205.507});
    r4_s0->AddLane(-2, 7.5, 7.5, {}, {});
    r4_s0->AddLane(-1, 4.0, 4.0, {}, {});
    r4_s0->AddLane( 1, 4.0, 4.0, {}, {});
    r4_s0->AddLane( 2, 7.5, 7.5, {}, {});

    map.CommitRoadInfo();
    //endregion

    //region Junction
    //Junc[0]-------------------------------------------------------------
    auto junc0 = map.AddJunction();

    map.AddConnection(junc0, 0, 1, 4, -1, 10.0, 10.0);
    map.AddConnection(junc0, 0, 2, 4, -2, 10.0, 10.0);
    map.AddConnection(junc0, 0, 3, 1, 1, 10.0, 10.0);

    map.AddConnection(junc0, 1, -1, 0, -1, 15.0, 15.0);
    map.AddConnection(junc0, 1, -1, 0, -2, 15.0, 15.0);
    map.AddConnection(junc0, 1, -2, 4, -1, 15.0, 15.0);
    map.AddConnection(junc0, 1, -2, 4, -2, 15.0, 15.0);

    map.AddConnection(junc0, 4, 1, 0, -1, 15.0, 15.0);
    map.AddConnection(junc0, 4, 1, 1,  1, 15.0, 15.0);
    map.AddConnection(junc0, 4, 2, 0, -2, 15.0, 15.0);

    //Junc[1]-------------------------------------------------------------
    auto junc1 = map.AddJunction();
    map.AddConnection(junc1, 1, 1, 2, 1, 15.0, 15.0);
    map.AddConnection(junc1, 2, -2, 1, -1, 15.0, 15.0);


    //Junc[2]-------------------------------------------------------------
    auto junc2 = map.AddJunction();
    map.AddConnection(junc2, 2, 1, 3, 1, 15.0, 15.0);
    map.AddConnection(junc2, 2, 1, 3, 2, 15.0, 15.0);

    map.AddConnection(junc2, 3, -3, 2, -1, 10.0, 10.0);

    //Junc[3]-------------------------------------------------------------
    auto junc3 = map.AddJunction();
    map.AddConnection(junc3, 3, 1, 4, 1, 15.0, 15.0);
    map.AddConnection(junc3, 3, 1, 4, 2, 15.0, 15.0);

    map.AddConnection(junc3, 4, -2, 3, -1, 10.0, 15.0);
    map.AddConnection(junc3, 4, -2, 3, -2, 10.0, 15.0);
    map.AddConnection(junc3, 4, -2, 3, -3, 10.0, 15.0);
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