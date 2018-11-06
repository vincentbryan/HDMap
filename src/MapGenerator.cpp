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

    r0->AddSignal(7.44, 5.99, 6.0, "SIG", "0101");
    r0->AddSignal(8.38, 3.99, 6.0, "SIG", "0111");

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
    r2_s3->AddLane(1, 4.5, 2.9, {1}, {1});
    r2_s3->AddLane(2, 4.5, 5.8, {1}, {2});

    auto r2_s4 = r2->AddSection({76.79, -148.08, -56.7125});
    r2_s4->AddLane(1, 2.9, 2.9, {1}, {});
    r2_s4->AddLane(2, 5.8, 5.8, {2}, {});


    auto r3 = map.AddRoad({76.79, -148.08, 123.2875});

    auto r3_s0 = r3->AddSection({62.44, -126.17, 123.2875});
    r3_s0->AddLane(1, 3.0, 3.0, {}, {1});


    auto r3_s1 = r3->AddSection({50.65, -110.54, 123.2875}, 10.0, 10.0);
    r3_s1->AddLane(1, 3.0, 4.5, {1}, {1});


    auto r3_s2 = r3->AddSection({25.95, -72.92, 123.2875});
    r3_s2->AddLane(1, 4.5, 4.5, {1}, {1, 2});

    auto r3_s3 = r3->AddSection({12.20, -54.34, 123.2875}, 15.0, 15.0);
    r3_s3->AddLane(1, 4.5, 2.9, {1}, {1});
    r3_s3->AddLane(2, 4.5, 5.8, {1}, {2});

    auto r3_s4 = r3->AddSection({-5.58, -27.31, 123.2875});
    r3_s4->AddLane(1, 2.9, 2.9, {1}, {});
    r3_s4->AddLane(2, 5.8, 5.8, {2}, {});

    r3->AddSignal(-23.99, 5.17, 6.0, "SIG", "1011");

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

    r4->AddSignal(314.52, -37.75, 6.0, "SIG", "1101");
    r4->AddSignal(315.51, -39.46, 6.0, "SIG", "0010");

    auto r5 = map.AddRoad({252.64, -68.10, 209.2014});

    auto r5_s0 = r5->AddSection({226.87, -82.46, 209.2014});
    r5_s0->AddLane(1, 3.0, 3.0, {}, {1});

    auto r5_s1 = r5->AddSection({199.06, -101.11, 209.2014}, 15.0, 15.0);
    r5_s1->AddLane(1, 3.0, 4.5, {1}, {1});

    auto r5_s2 = r5->AddSection({145.35, -131.00, 209.2014});
    r5_s2->AddLane(1, 4.5, 4.5, {1}, {1, 2});

    auto r5_s3 = r5->AddSection({131.73, -139.90, 209.2014}, 10.0, 10.0);
    r5_s3->AddLane(1, 4.5, 3.0, {1}, {1});
    r5_s3->AddLane(2, 4.5, 6.0, {1}, {2});

    auto r5_s4 = r5->AddSection({102.28, -156.36, 209.2014});
    r5_s4->AddLane(1, 3.0, 3.0, {1}, {});
    r5_s4->AddLane(2, 6.0, 6.0, {2}, {});

   //Road[6/7]-------------------------------------------------------------
    auto r6 = map.AddRoad({260.30, -35.27, 126.342});

    auto r6_s0 = r6->AddSection({185.09, 66.96, 126.342});
    r6_s0->AddLane( 1, 3.78, 3.78, {}, {});
    r6_s0->AddLane( 2, 7.26, 7.26, {}, {});
    r6_s0->AddLane( 3, 10.36, 10.36, {}, {});

    r6->AddSignal(154.85, 107.39, 6.0, "SIG", "1000");
    r6->AddSignal(156.42, 108.61, 6.0, "SIG", "0101");

    auto r7 = map.AddRoad({185.38, 66.53, -53.6531});

    auto r7_s0 = r7->AddSection({260.70, -35.83, -53.6531});
    r7_s0->AddLane(1, 3.78, 3.78, {}, {});
    r7_s0->AddLane(2, 7.26, 7.26, {}, {});
    r7_s0->AddLane(3, 10.36, 10.36, {}, {});

    r7->AddSignal(289.24, -85.89, 6.0, "SIG", "1101");
    r7->AddSignal(287.60, -87.08, 6.0, "SIG", "0010");

    //Road[8/9]-------------------------------------------------------------
    auto r8 = map.AddRoad({4.89, 8.51, 25.7603});

    auto r8_s0 = r8->AddSection({144.54, 75.90, 25.7603});
    r8_s0->AddLane(1, 3.8, 3.8, {}, {});
    r8_s0->AddLane(2, 7.2, 7.2, {}, {});

    r8->AddSignal(210.03, 110.64, 6.0, "SIG", "1000");
    r8->AddSignal(211.01, 108.91, 6.0, "SIG", "0101");

    auto r9 = map.AddRoad({144.15, 75.65, -154.28});

    auto r9_s0 = r9->AddSection({4.48, 8.37, -154.28});
    r9_s0->AddLane( 1, 3.8, 3.8, {}, {});
    r9_s0->AddLane( 2, 7.2, 7.2, {}, {});

    r9->AddSignal(-44.29, -11.26, 6.0, "SIG", "1000");
    r9->AddSignal(-45.48, -9.40,  6.0, "SIG", "1010");

    //Road[10/11]-------------------------------------------------------------
    auto r10 = map.AddRoad({191.44, 98.24, 27.71});

    auto r10_s0 = r10->AddSection({219.60, 113.03, 27.71});
    r10_s0->AddLane(1, 3.0, 3.0, {}, {1});

    auto r10_s1 = r10->AddSection({246.70, 128.85, 27.71}, 18.0, 18.0);
    r10_s1->AddLane(1, 3.0, 4.5, {1}, {1});

    auto r10_s2 = r10->AddSection({264.32, 138.16, 27.71});
    r10_s2->AddLane(1, 4.5, 4.5, {1}, {1});

    auto r10_s3 = r10->AddSection({385.41, 133.54, -34.4855}, 60, 40);
    r10_s3->AddLane(1, 4.5, 4.5, {1}, {1});

    auto r10_s4 = r10->AddSection({472.03, 74.04, -34.4855});
    r10_s4->AddLane(1, 4.5, 4.5, {1}, {});

    auto r11 = map.AddRoad({472.03, 74.04, 145.5145});

    auto r11_s0 = r11->AddSection({385.41, 133.54, 145.5145});
    r11_s0->AddLane(1, 4.5, 4.5, {}, {1});

    auto r11_s1 = r11->AddSection({264.32, 138.16, -152.291}, 40, 60);
    r11_s1->AddLane(1, 4.5, 4.5, {1}, {1});

    auto r11_s2 = r11->AddSection({246.70, 128.85, -152.291});
    r11_s2->AddLane(1, 4.5, 4.5, {1}, {1, 2});

    auto r11_s3 = r11->AddSection({219.60, 113.03, -152.291}, 18.0, 18.0);
    r11_s3->AddLane(1, 4.5, 3.0, {1}, {1});
    r11_s3->AddLane(2, 4.5, 6.0, {1}, {2});

    auto r11_s4 = r11->AddSection({191.44, 98.24, -152.291});
    r11_s4->AddLane(1, 3.0, 3.0, {1}, {});
    r11_s4->AddLane(2, 6.0, 6.0, {2}, {});

    r11->AddSignal(139.28, 77.52, 6.00, "SIG", "1000");
    r11->AddSignal(138.73, 79.22, 6.00, "SIG", "0101");

    //Road[12/13]-------------------------------------------------------------
    auto r12 = map.AddRoad({300.21, -48.38, 29.004});

    auto r12_s0 = r12->AddSection({326.40, -33.86, 29.004});
    r12_s0->AddLane(1, 3.0, 3.0, {}, {1});

    auto r12_s1 = r12->AddSection({357.13, -13.60, 29.004}, 15.0, 15.0);
    r12_s1->AddLane(1, 3.0, 4.5, {1}, {1});

    auto r12_s2 = r12->AddSection({419.84, 21.21, 29.004});
    r12_s2->AddLane(1, 4.5, 4.5, {1}, {1, 2});

    auto r12_s3 = r12->AddSection({444.29, 36.41, 29.004}, 15.0, 15.0);
    r12_s3->AddLane(1, 4.5, 3.0, {1}, {1});
    r12_s3->AddLane(2, 4.5, 6.0, {1}, {2});

    auto r12_s4 = r12->AddSection({467.82, 49.55, 29.004});
    r12_s4->AddLane(1, 3.0, 3.0, {1}, {});
    r12_s4->AddLane(2, 6.0, 6.0, {2}, {});

    auto r13 = map.AddRoad({467.82, 49.55, -150.996});

    auto r13_s0 = r13->AddSection({444.29, 36.41, -150.996});
    r13_s0->AddLane(1, 3.0, 3.0, {}, {1});

    auto r13_s1 = r13->AddSection({419.84, 21.21, -150.996}, 15.0, 15.0);
    r13_s1->AddLane(1, 3.0, 4.5, {1}, {1});

    auto r13_s2 = r13->AddSection({357.13, -13.60, -150.996});
    r13_s2->AddLane(1, 4.5, 4.5, {1}, {1, 2, 3});

    auto r13_s3 = r13->AddSection({326.40, -33.86, -150.996}, 15.0, 15.0);
    r13_s3->AddLane(1, 4.5, 3.0, {1}, {1});
    r13_s3->AddLane(2, 4.5, 6.0, {1}, {2});
    r13_s3->AddLane(3, 4.5, 9.0, {1}, {3});

    auto r13_s4 = r13->AddSection({300.21, -48.38, -150.996});
    r13_s4->AddLane(1, 3.0, 3.0, {1}, {});
    r13_s4->AddLane(2, 6.0, 6.0, {2}, {});
    r13_s4->AddLane(3, 9.0, 9.0, {3}, {});

    r13->AddSignal(248.71, -72.96, 6.0, "SIG", "1101");
    r13->AddSignal(247.75, -71.36, 6.0, "SIG", "0010");

    //Road[14/15]-------------------------------------------------------------

    auto r14 = map.AddRoad({378.93, -208.58, 124.063});

    auto r14_s0 = r14->AddSection({292.28, -80.42, 124.063});
    r14_s0->AddLane(1, 3.78, 3.78, {}, {});
    r14_s0->AddLane(2, 7.26, 7.26, {}, {});
    r14_s0->AddLane(3, 10.36, 10.36, {}, {});

    r14->AddSignal(262.74, -30.18, 6.0, "SIG", "1101");
    r14->AddSignal(264.63, -28.89, 6.0, "SIG", "0010");

    auto r15 = map.AddRoad({292.50, -81.01, -56.0177});

    auto r15_s0 = r15->AddSection({378.82, -209.07, -56.0177});
    r15_s0->AddLane(1, 3.8, 3.8, {}, {});
    r15_s0->AddLane(2, 7.26, 7.26, {}, {});
    r15_s0->AddLane(3, 10.36, 10.36, {}, {});


    //Road[16/17]-------------------------------------------------------------
    auto r16 = map.AddRoad({499.18, 51.05, -40.875});

    auto r16_s0 = r16->AddSection({511.37, 40.50, -40.875});
    r16_s0->AddLane(1, 3.0, 3.0, {}, {1});

    auto r16_s1 = r16->AddSection({546.70, 10.82, -40.875}, 15.0, 15.0);
    r16_s1->AddLane(1, 3.0, 4.5, {1}, {1});

    auto r16_s2 = r16->AddSection({636.66, -70.43, -40.875});
    r16_s2->AddLane(1, 4.5, 4.5, {1}, {1});

    auto r17 = map.AddRoad({636.66, -70.43, 139.125});

    auto r17_s0 = r17->AddSection({546.70, 10.82, 139.125});
    r17_s0->AddLane(1, 4.5, 4.5, {}, {1, 2});

    auto r17_s1 = r17->AddSection({511.37, 40.50, 139.125}, 15.0, 15.0);
    r17_s1->AddLane(1, 4.5, 3.0, {1}, {1});
    r17_s1->AddLane(2, 4.5, 6.0, {2}, {2});

    auto r17_s2 = r17->AddSection({499.18, 51.05, 139.125});
    r17_s2->AddLane(1, 3.0, 3.0, {1}, {});
    r17_s2->AddLane(2, 6.0, 6.0, {2}, {});


    //Road[18/19]-------------------------------------------------------------

    auto r18 = map.AddRoad({416.76, -220.00, 29.437});

    auto r18_s0 = r18->AddSection({443.66, -204.82, 29.437});
    r18_s0->AddLane(1, 3.0, 3.0, {}, {1});

    auto r18_s1 = r18->AddSection({470.15, -186.79, 29.437}, 15.0, 15.0);
    r18_s1->AddLane(1, 3.0, 4.5, {1}, {1});

    auto r18_s2 = r18->AddSection({582.31, -123.57, 29.437});
    r18_s2->AddLane(1, 4.5, 4.5, {1}, {1, 2});

    auto r18_s3 = r18->AddSection({607.80, -107.74, 29.437}, 10.0, 10.0);
    r18_s3->AddLane(1, 4.5, 3.0, {1}, {1});
    r18_s3->AddLane(2, 4.5, 6.0, {1}, {2});

    auto r18_s4 = r18->AddSection({631.80, -94.28, 29.437});
    r18_s4->AddLane(1, 3.0, 3.0, {1}, {});
    r18_s4->AddLane(2, 6.0, 6.0, {2}, {});

    auto r19 = map.AddRoad({631.80, -94.28, -150.563});

    auto r19_s0 = r19->AddSection({607.80, -107.74, -150.563});
    r19_s0->AddLane(1, 2.9, 2.9, {}, {1});

    auto r19_s1 = r19->AddSection({582.31, -123.57, -150.563}, 10.0, 10.0);
    r19_s1->AddLane(1, 2.9, 4.5, {1}, {1});

    auto r19_s2 = r19->AddSection({470.15, -186.79, -150.563});
    r19_s2->AddLane(1, 4.5, 4.5, {1}, {1, 2, 3});

    auto r19_s3 = r19->AddSection({443.66, -204.82, -150.563}, 15.0, 15.0);
    r19_s3->AddLane(1, 4.5, 3.0, {1}, {1});
    r19_s3->AddLane(2, 4.5, 6.0, {1}, {2});
    r19_s3->AddLane(3, 4.5, 9.0, {1}, {3});

    auto r19_s4 = r19->AddSection({416.76, -220.00, -150.563});
    r19_s4->AddLane(1, 3.0, 3.0, {1}, {});
    r19_s4->AddLane(2, 6.0, 6.0, {2}, {});
    r19_s4->AddLane(3, 9.0, 9.0, {3}, {});


    //Road[20/21]-------------------------------------------------------------
    auto r20 = map.AddRoad({216.71, -331.26, 29.337});

    auto r20_s0 = r20->AddSection({250.43, -312.21, 29.337});
    r20_s0->AddLane(1, 3.0, 3.0, {}, {1});

    auto r20_s1 = r20->AddSection({270.98, -298.99, 29.337}, 15.0, 15.0);
    r20_s1->AddLane(1, 3.0, 4.5, {1}, {1});

    auto r20_s2 = r20->AddSection({311.85, -276.02, 29.337});
    r20_s2->AddLane(1, 4.5, 4.5, {1}, {1, 2, 3});

    auto r20_s3 = r20->AddSection({336.87, -258.88, 29.337}, 15.0, 15.0);
    r20_s3->AddLane(1, 4.5, 3.0, {1}, {1});
    r20_s3->AddLane(2, 4.5, 6.0, {1}, {2});
    r20_s3->AddLane(3, 4.5, 9.0, {1}, {3});

    auto r20_s4 = r20->AddSection({368.83, -240.71, 29.337});
    r20_s4->AddLane(1, 3.0, 3.0, {1}, {});
    r20_s4->AddLane(2, 6.0, 6.0, {2}, {});
    r20_s4->AddLane(3, 9.0, 9.0, {3}, {});

    auto r21 = map.AddRoad({368.83, -240.71, -150.663});

    auto r21_s0 = r21->AddSection({336.87, -258.88, -150.663});
    r21_s0->AddLane(1, 3.0, 3.0, {}, {1});

    auto r21_s1 = r21->AddSection({311.85, -276.02, -150.663}, 15.0, 15.0);
    r21_s1->AddLane(1, 3.0, 4.5, {1}, {1});

    auto r21_s2 = r21->AddSection({270.98, -298.99, -150.663});
    r21_s2->AddLane(1, 4.5, 4.5, {1}, {1, 2});

    auto r21_s3 = r21->AddSection({250.43, -312.21, -150.663}, 15.0, 15.0);
    r21_s3->AddLane(1, 4.5, 3.0, {1}, {1});
    r21_s3->AddLane(2, 4.5, 6.0, {1}, {2});

    auto r21_s4 = r21->AddSection({216.71, -331.26, -150.663});
    r21_s4->AddLane(1, 3.0, 3.0, {1}, {});
    r21_s4->AddLane(2, 6.0, 6.0, {2}, {});


    //Road[22/23]-------------------------------------------------------------
    auto r22 = map.AddRoad({33.65, -432.96, 29.464});

    auto r22_s0 = r22->AddSection({127.70, -379.84, 29.464});
    r22_s0->AddLane(1, 4.5, 4.5, {}, {1, 2});

    auto r22_s1 = r22->AddSection({148.53, -366.65, 29.464}, 10.0, 10.0);
    r22_s1->AddLane(1, 4.5, 3.0, {1}, {1});
    r22_s1->AddLane(2, 4.5, 6.0, {1}, {2});

    auto r22_s2 = r22->AddSection({182.87, -347.25, 29.464});
    r22_s2->AddLane(1, 3.0, 3.0, {1}, {});
    r22_s2->AddLane(2, 6.0, 6.0, {2}, {});

    auto r23 = map.AddRoad({182.87, -347.25, -150.536});

    auto r23_s0 = r23->AddSection({148.53, -366.65, -150.536});
    r23_s0->AddLane(1, 3.0, 3.0, {}, {1});

    auto r23_s1 = r23->AddSection({127.70, -379.84, -150.536}, 10.0, 10.0);
    r23_s1->AddLane(1, 3.0, 4.5, {1}, {1});

    auto r23_s2 = r23->AddSection({33.65, -432.96, -150.536});
    r23_s2->AddLane(1, 4.5, 4.5, {1}, {});


    //Road[24/25]-------------------------------------------------------------
    auto r24 = map.AddRoad({95.06, -181.19, -56.9236});

    auto r24_s0 = r24->AddSection({112.02, -207.23, -56.9236});
    r24_s0->AddLane(1, 3.0, 3.0, {}, {1});

    auto r24_s1 = r24->AddSection({124.77, -224.02, -56.9236}, 10.0, 10.0);
    r24_s1->AddLane(1, 3.0, 4.5, {1}, {1});

    auto r24_s2 = r24->AddSection({160.47, -278.54, -56.9236});
    r24_s2->AddLane(1, 4.5, 4.5, {1}, {1, 2});

    auto r24_s3 = r24->AddSection({173.65, -296.34, -56.9236}, 15.0, 15.0);
    r24_s3->AddLane(1, 4.5, 3.0, {1}, {1});
    r24_s3->AddLane(2, 4.5, 6.0, {1}, {2});

    auto r24_s4 = r24->AddSection({190.95, -322.75, -56.9236});
    r24_s4->AddLane(1, 3.0, 3.0, {1}, {});
    r24_s4->AddLane(2, 6.0, 6.0, {2}, {});

    auto r25 = map.AddRoad({190.95, -322.75, 123.0764});

    auto r25_s0 = r25->AddSection({173.65, -296.34, 123.0764});
    r25_s0->AddLane(1, 3.0, 3.0, {}, {1});

    auto r25_s1 = r25->AddSection({160.47, -278.54, 123.0764}, 15.0, 15.0);
    r25_s1->AddLane(1, 3.0, 4.5, {1}, {1});

    auto r25_s2 = r25->AddSection({124.77, -224.02, 123.0764});
    r25_s2->AddLane(1, 4.5, 4.5, {1}, {1, 2});

    auto r25_s3 = r25->AddSection({112.02, -207.23, 123.0764}, 10.0, 10.0);
    r25_s3->AddLane(1, 4.5, 3.0, {1}, {1});
    r25_s3->AddLane(2, 4.5, 6.0, {1}, {2});

    auto r25_s4 = r25->AddSection({95.06, -181.19, 123.0764});
    r25_s4->AddLane(1, 3.0, 3.0, {1}, {});
    r25_s4->AddLane(2, 6.0, 6.0, {2}, {});

    //Road[26/27]-------------------------------------------------------------
    auto r26 = map.AddRoad({-99.93, -269.22, 29.1489});

    auto r26_s0 = r26->AddSection({-64.32, -249.36, 29.1489});
    r26_s0->AddLane(1, 3.0, 3.0, {}, {1});

    auto r26_s1 = r26->AddSection({-45.72, -237.33, 29.1489}, 15.0, 15.0);
    r26_s1->AddLane(1, 3.0, 4.5, {1}, {1});

    auto r26_s2 = r26->AddSection({21.28, -200.07, 29.1489});
    r26_s2->AddLane(1, 4.5, 4.5, {1}, {1, 2});

    auto r26_s3 = r26->AddSection({41.01, -187.68, 29.1489}, 15.0, 15.0);
    r26_s3->AddLane(1, 4.5, 3.0, {1}, {1});
    r26_s3->AddLane(2, 4.5, 6.0, {1}, {2});

    auto r26_s4 = r26->AddSection({71.35, -169.78, 29.1489});
    r26_s4->AddLane(1, 3.0, 3.0, {1}, {});
    r26_s4->AddLane(2, 6.0, 6.0, {2}, {});

    auto r27 = map.AddRoad({71.35, -169.78, -150.851});

    auto r27_s0 = r27->AddSection({41.01, -187.68, -150.851});
    r27_s0->AddLane(1, 3.0, 3.0, {}, {});

    auto r27_s1 = r27->AddSection({21.28, -200.07, -150.851}, 15.0, 15.0);
    r27_s1->AddLane(1, 3.0, 4.5, {}, {});

    auto r27_s2 = r27->AddSection({-45.72, -237.33, -150.851});
    r27_s2->AddLane(1, 4.5, 4.5, {}, {});

    auto r27_s3 = r27->AddSection({-64.32, -249.36, -150.851}, 15.0, 15.0);
    r27_s3->AddLane(1, 4.5, 3.0, {}, {});
    r27_s3->AddLane(2, 4.5, 6.0, {}, {});

    auto r27_s4 = r27->AddSection({-99.93, -269.22, -150.851});
    r27_s4->AddLane(1, 3.0, 3.0, {}, {});
    r27_s4->AddLane(2, 6.0, 6.0, {}, {});



    //湖山路
    //Road[28/29]-------------------------------------------------------------
    auto r28 = map.AddRoad({2.74, -435.17, 129.648});

    auto r28_s0 = r28->AddSection({-195.29, -196.20, 129.648});
    r28_s0->AddLane(1, 3.3, 3.3, {}, {1});
    r28_s0->AddLane(2, 6.6, 6.6, {}, {2});
    r28_s0->AddLane(3, 10.0, 10.0, {}, {3});

    auto r28_s1 = r28->AddSection({-237.58, -142.14, 128.035}, 10.0, 10.0);
    r28_s1->AddLane(1, 3.3, 3.3, {1}, {});
    r28_s1->AddLane(2, 6.6, 6.6, {2}, {});
    r28_s1->AddLane(3, 10.0, 10.0, {3}, {});

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
    map.AddConnection(junc1, 2, 2, 24, 1, 10.0, 10.0);
    map.AddConnection(junc1, 2, 2, 27, 1, 15.0, 15.0);

    map.AddConnection(junc1, 5, 1, 24, 1, 15.0, 15.0);
    map.AddConnection(junc1, 5, 2, 3, 1, 15.0, 15.0);
    map.AddConnection(junc1, 5, 2, 27, 1, 10.0, 10.0);

    map.AddConnection(junc1, 25, 1, 27, 1, 15.0, 15.0);
    map.AddConnection(junc1, 25, 2, 3, 1, 10.0, 10.0);
    map.AddConnection(junc1, 25, 2, 4, 1, 15.0, 15.0);

    map.AddConnection(junc1, 26, 1, 3, 1, 15.0, 15.0);
    map.AddConnection(junc1, 26, 2, 4, 1, 10.0, 10.0);
    map.AddConnection(junc1, 26, 2, 24, 1, 15.0, 15.0);

    //Junc[2]-------------------------------------------------------------
    auto junc2 = map.AddJunction();

    map.AddConnection(junc2, 4, 1, 6, 2, 15.0, 15.0);
    map.AddConnection(junc2, 4, 2, 12, 1, 10.0, 10.0);
    map.AddConnection(junc2, 4, 3, 15, 2, 15.0, 15.0);

    map.AddConnection(junc2, 14, 1, 5, 1, 15.0, 15.0);
    map.AddConnection(junc2, 14, 2, 6, 2, 10.0, 10.0);
    map.AddConnection(junc2, 14, 3, 12, 1, 15.0, 15.0);

    map.AddConnection(junc2, 13, 1, 15, 2, 15.0, 15.0);
    map.AddConnection(junc2, 13, 2, 5, 1, 10.0, 10.0);
    map.AddConnection(junc2, 13, 3, 6, 2, 15.0, 15.0);

    map.AddConnection(junc2, 7, 1, 12, 1, 15.0, 15.0);
    map.AddConnection(junc2, 7, 2, 15, 2, 10.0, 10.0);
    map.AddConnection(junc2, 7, 3, 5, 1, 15.0, 15.0);

    //Junc[3]-------------------------------------------------------------
    auto junc3 = map.AddJunction();
    map.AddConnection(junc3, 6, 1, 9, 1, 15.0, 15.0);
//    map.AddConnection(junc3, 6, 1, 9, 2, 15.0, 15.0);
    map.AddConnection(junc3, 6, 3, 10, 1, 15.0, 15.0);

//    map.AddConnection(junc3, 8, 2, 7, 1, 10.0, 15.0);
    map.AddConnection(junc3, 8, 2, 7, 2, 10.0, 15.0);
//    map.AddConnection(junc3, 8, 2, 7, 3, 10.0, 15.0);
    map.AddConnection(junc3, 8, 1, 10, 1, 10.0, 10.0);

    map.AddConnection(junc3, 11, 1, 7, 2, 15.0, 15.0);
    map.AddConnection(junc3, 11, 2, 9, 1, 10.0, 10.0);

    //Junc[4]-------------------------------------------------------------
    auto junc4 = map.AddJunction();
    map.AddConnection(junc4, 10, 1, 13, 1, 15.0, 15.0);
    map.AddConnection(junc4, 10, 1, 16, 1, 10.0, 10.0);

    map.AddConnection(junc4, 12, 1, 11, 1, 15.0, 15.0);
    map.AddConnection(junc4, 12, 2, 16, 1, 15.0, 15.0);

    map.AddConnection(junc4, 17, 1, 13, 1, 15.0, 15.0);
    map.AddConnection(junc4, 17, 2, 11, 1, 15.0, 15.0);

    //Junc[5]-------------------------------------------------------------
    auto junc5 = map.AddJunction();
    map.AddConnection(junc5, 16, 1, 19, 1, 15.0, 15.0);

    map.AddConnection(junc5, 18, 1, 17, 1, 15.0, 15.0);

    //Junc[6]-------------------------------------------------------------
    auto junc6 = map.AddJunction();
    map.AddConnection(junc6, 15, 3, 21, 1, 15.0, 15.0);
    map.AddConnection(junc6, 15, 1, 18, 1, 15.0, 15.0);

    map.AddConnection(junc6, 20, 1, 14, 2, 15.0, 15.0);
    map.AddConnection(junc6, 20, 2, 18, 1, 10.0, 10.0);

    map.AddConnection(junc6, 19, 2, 21, 1, 10.0, 10.0);
    map.AddConnection(junc6, 19, 3, 14, 2, 15.0, 15.0);

    //Junc[7]-------------------------------------------------------------
    auto junc7 = map.AddJunction();
    map.AddConnection(junc7, 24, 1, 20, 1, 15.0, 15.0);
    map.AddConnection(junc7, 24, 2, 23, 1, 15.0, 15.0);

    map.AddConnection(junc7, 22, 1, 25, 1, 15.0, 15.0);
    map.AddConnection(junc7, 22, 2, 20, 1, 10.0, 10.0);

    map.AddConnection(junc7, 21, 2, 25, 1, 15.0, 15.0);
    map.AddConnection(junc7, 21, 2, 23, 1, 10.0, 10.0);

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