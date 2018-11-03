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
    //Road[0/1]-------------------------------------------------------------
    auto r0 = map.AddRoad({-237.48, -108.11, 25.719});

    auto r0_s0 = r0->AddSection({-40.36, -13.162, 25.719});
    r0_s0->AddLane( 1,  4.00,  4.00, {}, {});
    r0_s0->AddLane( 2,  7.50,  7.50, {}, {});
    r0_s0->AddLane( 3, 10.80, 10.80, {}, {});

    auto r1 = map.AddRoad({-41.01, -13.67, 205.719});

    auto r1_s0 = r1->AddSection({-238.360, -108.96, 205.719});
    r1_s0->AddLane(1, 4.00, 4.00, {}, {});
    r1_s0->AddLane(2, 7.50, 7.50, {}, {});

//    r0->AddSignal({-55.123, 92.702}, 1, "SIG", "010");
//    r0->AddSignal({-55.123, 90.702}, 1, "SIG", "001");


    //Road[2/3]-----------------------------------------------------------
    auto r2 = map.AddRoad({-5.58, -27.31, -56.6636});

    auto r2_s0 = r2->AddSection({12.20, -54.34, -56.6636});
    r2_s0->AddLane( 1, 3.0, 3.0, {}, {1});

    auto r2_s1 = r2->AddSection({25.95, -72.92, -56.7125}, 15.0, 15.0);
    r2_s1->AddLane( 1, 3.0, 4.5, {1}, {1});


    auto r2_s2 = r2->AddSection({50.65, -110.54, -56.7125});
    r2_s2->AddLane( 1, 4.5, 4.5, {1}, {1, 2});

    auto r2_s3 = r2->AddSection({62.44, -126.17, -56.7125}, 10.0, 10.0);
    r2_s3->AddLane(1, 4.5, 2.9, {1}, {});
    r2_s3->AddLane(2, 4.5, 5.8, {1}, {});

    auto r2_s4 = r2->AddSection({76.79, -148.08, -56.7125});
    r2_s4->AddLane(1, 2.9, 2.9, {}, {});
    r2_s4->AddLane(2, 5.8, 5.8, {}, {});


    auto r3 = map.AddRoad({76.79, -148.08, 123.2875});

    auto r3_s0 = r3->AddSection({62.44, -126.17, 123.2875});
    r3_s0->AddLane(1, 3.0, 3.0, {}, {});


    auto r3_s1 = r3->AddSection({50.65, -110.54, 123.2875}, 10.0, 10.0);
    r3_s1->AddLane(1, 3.0, 4.5, {}, {});


    auto r3_s2 = r3->AddSection({25.95, -72.92, 123.2875});
    r3_s2->AddLane(1, 4.5, 4.5, {1}, {});

    auto r3_s3 = r3->AddSection({12.20, -54.34, 123.2875}, 15.0, 15.0);
    r3_s3->AddLane(1, 4.5, 2.9, {}, {});
    r3_s3->AddLane(2, 4.5, 5.8, {}, {});

    auto r3_s4 = r3->AddSection({-5.58, -27.31, 123.2875});
    r3_s4->AddLane(1, 2.9, 2.9, {}, {});
    r3_s4->AddLane(2, 5.8, 5.8, {}, {});

  //Road[4/5]-------------------------------------------------------------
    auto r4 = map.AddRoad({102.28, -156.36, 29.2014});

    auto r4_s0 = r4->AddSection({131.73, -139.90, 29.2014});
    r4_s0->AddLane( 1, 3.15, 3.15, {}, {1});

    auto r4_s1 = r4->AddSection({145.35, -131.00, 29.2014}, 10.0, 10.0);
    r4_s1->AddLane( 1, 3.15, 4.50, {1}, {1});

    auto r4_s2 = r4->AddSection({199.06, -101.11, 29.2014});
    r4_s2->AddLane( 1, 4.50, 4.50, {1}, {1, 2, 3});

    auto r4_s3 = r4->AddSection({226.87, -82.46, 29.2014}, 15.0, 15.0);
    r4_s3->AddLane( 1, 4.5, 3.0, {1}, {1});
    r4_s3->AddLane( 2, 4.5, 6.0, {2}, {2});
    r4_s3->AddLane( 3, 4.5, 9.0, {3}, {3});

    auto r4_s4 = r4->AddSection({252.64, -68.10, 29.2014});
    r4_s4->AddLane( 1, 3.0, 3.0, {1}, {});
    r4_s4->AddLane( 2, 6.0, 6.0, {2}, {});
    r4_s4->AddLane( 3, 9.0, 9.0, {3}, {});

    auto r5 = map.AddRoad({252.64, -68.10, 209.2014});

    auto r5_s0 = r5->AddSection({226.87, -82.46, 209.2014});
    r5_s0->AddLane(1, 3.0, 3.0, {}, {1});

    auto r5_s1 = r5->AddSection({199.06, -101.11, 209.2014}, 15.0, 15.0);
    r5_s1->AddLane(1, 3.0, 3.0, {1}, {1});

    auto r5_s2 = r5->AddSection({145.35, -131.00, 209.2014});
    r5_s2->AddLane(1, 3.0, 3.0, {1}, {1, 2});

    auto r5_s3 = r5->AddSection({131.73, -139.90, 209.2014}, 10.0, 10.0);
    r5_s3->AddLane(1, 3.0, 3.0, {1}, {1});
    r5_s3->AddLane(2, 3.0, 6.0, {2}, {2});

    auto r5_s4 = r5->AddSection({102.28, -156.36, 209.2014});
    r5_s4->AddLane(1, 3.0, 3.0, {1}, {1});
    r5_s4->AddLane(2, 6.0, 6.0, {2}, {2});

   //Road[6/7]-------------------------------------------------------------
    auto r6 = map.AddRoad({260.30, -35.27, 126.342});

    auto r6_s0 = r6->AddSection({185.09, 66.96, 126.342});
    r6_s0->AddLane( 1, 3.78, 3.78, {}, {});
    r6_s0->AddLane( 2, 7.26, 7.26, {}, {});
    r6_s0->AddLane( 3, 10.36, 10.36,  {}, {});

    auto r7 = map.AddRoad({185.38, 66.53, -53.6531});

    auto r7_s0 = r7->AddSection({260.70, -35.83, -53.6531});
    r7_s0->AddLane(1, 3.78, 3.78, {}, {});
    r7_s0->AddLane(2, 7.26, 7.26, {}, {});
    r7_s0->AddLane(3, 10.36, 10.36, {}, {});


    //Road[8/9]-------------------------------------------------------------
    auto r8 = map.AddRoad({4.89, 8.51, 25.7603});

    auto r8_s0 = r8->AddSection({144.54, 75.90, 25.7603});
    r8_s0->AddLane(1, 3.8, 3.8, {}, {});
    r8_s0->AddLane(2, 7.2, 7.2, {}, {});

    auto r9 = map.AddRoad({144.15, 75.65, -154.28});

    auto r9_s0 = r9->AddSection({4.48, 8.37, -154.28});
    r9_s0->AddLane( 1, 3.8, 3.8, {}, {});
    r9_s0->AddLane( 2, 7.2, 7.2, {}, {});

    map.CommitRoadInfo();

    //endregion

    //region Junction
    //Junc[0]-------------------------------------------------------------

    auto junc0 = map.AddJunction();

    map.AddConnection(junc0, 0, 1, 8, 1, 10.0, 10.0);
    map.AddConnection(junc0, 0, 2, 8, 2, 10.0, 10.0);
    map.AddConnection(junc0, 0, 3, 2, 1, 25.0, 15.0);


    map.AddConnection(junc0, 3, 1, 1, 1, 15.0, 15.0);
    map.AddConnection(junc0, 3, 1, 1, 2, 15.0, 15.0);
    map.AddConnection(junc0, 3, 2, 8, 1, 15.0, 15.0);
    map.AddConnection(junc0, 3, 2, 8, 2, 15.0, 15.0);

    map.AddConnection(junc0, 9, 1, 1, 1, 15.0, 15.0);
    map.AddConnection(junc0, 9, 1, 2, 1, 15.0, 15.0);
    map.AddConnection(junc0, 9, 2, 1, 2, 15.0, 15.0);


    //Junc[1]-------------------------------------------------------------
    auto junc1 = map.AddJunction();
    map.AddConnection(junc1, 2, 1, 4, 1, 15.0, 15.0);
    map.AddConnection(junc1, 5, 2, 3, 1, 15.0, 15.0);


    //Junc[2]-------------------------------------------------------------
    auto junc2 = map.AddJunction();
    map.AddConnection(junc2, 4, 1, 6, 1, 15.0, 15.0);
    map.AddConnection(junc2, 4, 1, 6, 2, 15.0, 15.0);

    map.AddConnection(junc2, 7, 3, 5, 1, 10.0, 10.0);

    //Junc[3]-------------------------------------------------------------
    auto junc3 = map.AddJunction();
    map.AddConnection(junc3, 6, 1, 9, 1, 15.0, 15.0);
    map.AddConnection(junc3, 6, 1, 9, 2, 15.0, 15.0);

    map.AddConnection(junc3, 8, 2, 7, 1, 10.0, 15.0);
    map.AddConnection(junc3, 8, 2, 7, 2, 10.0, 15.0);
    map.AddConnection(junc3, 8, 2, 7, 3, 10.0, 15.0);
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