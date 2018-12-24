//
// Created by vincent on 18-10-23.
//

#include <ros/ros.h>
#include <thread>
#include <visualization_msgs/MarkerArray.h>
#include "Type/Map.h"

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

    double offset = 0.0;
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<visualization_msgs::MarkerArray>("HDMap", 5000);
    shared_ptr<Sender> p_sender(new Sender(pub));

    Map map;
    map.SetSender(p_sender);

    //region Road
    //Road[0/1]-------------------------------------------------------------
    auto r0 = map.AddRoad({-229.17, -105.03, 25.719});

    auto r0_s0 = r0->AddSection({-39.96, -13.67, 25.719});
    r0_s0->AddLane( 1,  3.75,  3.75, {}, {});
    r0_s0->AddLane( 2,  7.37,  7.37, {}, {});
    r0_s0->AddLane( 3,  10.4,  10.4, {}, {});

    r0->AddSignal(7.53, 5.42, 5.8, Angle(25.719), "SIG", "0101");
    r0->AddSignal(8.47, 3.42, 5.8, Angle(25.719), "SIG", "0111");

    auto r1 = map.AddRoad({-40.31, -13.43, 205.719});

    auto r1_s0 = r1->AddSection({-229.35, -104.65, 205.719});
    r1_s0->AddLane(1, 3.9, 3.9, {}, {});
    r1_s0->AddLane(2, 7.37, 7.37, {}, {});

    //Road[2/3]-----------------------------------------------------------
    auto r2 = map.AddRoad({-5.58, -27.31, -56.6636});

    auto r2_s0 = r2->AddSection({12.20, -54.34, -56.6636});
    r2_s0->AddLane( 1, 2.8, 2.8, {}, {1});

    auto r2_s1 = r2->AddSection({25.95, -72.92, -56.7125}, 15.0, 15.0);
    r2_s1->AddLane( 1, 2.8, 4.6, {1}, {1});


    auto r2_s2 = r2->AddSection({50.65, -110.54, -56.7125});
    r2_s2->AddLane( 1, 4.6, 4.6, {1}, {1, 2});

    auto r2_s3 = r2->AddSection({62.44, -126.17, -56.7125}, 10.0, 10.0);
    r2_s3->AddLane(1, 4.6, 2.8, {1}, {1});
    r2_s3->AddLane(2, 4.6, 5.8, {1}, {2});

    auto r2_s4 = r2->AddSection({76.79, -148.08, -56.7125});
    r2_s4->AddLane(1, 2.8, 2.8, {1}, {});
    r2_s4->AddLane(2, 5.8, 5.8, {2}, {});


    auto r3 = map.AddRoad({76.79, -148.08, 123.2875});

    auto r3_s0 = r3->AddSection({62.44, -126.17, 123.2875});
    r3_s0->AddLane(1, 2.8, 2.8, {}, {1});


    auto r3_s1 = r3->AddSection({50.65, -110.54, 123.2875}, 10.0, 10.0);
    r3_s1->AddLane(1, 2.8, 4.6, {1}, {1});


    auto r3_s2 = r3->AddSection({25.95, -72.92, 123.2875});
    r3_s2->AddLane(1, 4.6, 4.6, {1}, {1, 2});

    auto r3_s3 = r3->AddSection({12.20, -54.34, 123.2875}, 15.0, 15.0);
    r3_s3->AddLane(1, 4.6, 2.8, {1}, {1});
    r3_s3->AddLane(2, 4.6, 5.8, {1}, {2});

    auto r3_s4 = r3->AddSection({-5.58, -27.31, 123.2875});
    r3_s4->AddLane(1, 2.8, 2.8, {1}, {});
    r3_s4->AddLane(2, 5.8, 5.8, {2}, {});

    r3->AddSignal(-23.99, 5.17, 4.0, Angle(123.2875), "SIG", "1011");

    //Road[4/5]-------------------------------------------------------------
    auto r4 = map.AddRoad({101.60, -157.67, 29.2014});

    auto r4_s0 = r4->AddSection({125.19, -144.53, 29.2014});
    r4_s0->AddLane( 1, 3.0, 3.0, {}, {1});

    auto r4_s1 = r4->AddSection({165.19, -120.44, 29.2014}, 10.0, 10.0);
    r4_s1->AddLane( 1, 3.0, 4.6, {1}, {1});

    auto r4_s2 = r4->AddSection({192.27, -105.54, 29.2014});
    r4_s2->AddLane( 1, 4.6, 4.6, {1}, {1, 2, 3});

    auto r4_s3 = r4->AddSection({237.14, -77.45, 29.2014}, 15.0, 15.0);
    r4_s3->AddLane( 1, 4.6, 2.9, {1}, {1});
    r4_s3->AddLane( 2, 4.6, 5.8, {1}, {2});
    r4_s3->AddLane( 3, 4.6, 8.5, {1}, {3});

    auto r4_s4 = r4->AddSection({253.27, -68.48, 29.2014});
    r4_s4->AddLane( 1, 2.9, 2.9, {1}, {});
    r4_s4->AddLane( 2, 5.8, 5.8, {2}, {});
    r4_s4->AddLane( 3, 8.5, 8.5, {3}, {});

    r4->AddSignal(314.374, -37.4881, 5.8, Angle(29.2014), "SIG", "1101");
    r4->AddSignal(315.364, -39.1981, 5.8, Angle(29.2014), "SIG", "0010");

    auto r5 = map.AddRoad({253.27, -68.48, 209.2014});

    auto r5_s0 = r5->AddSection({237.14, -77.45, 209.2014});
    r5_s0->AddLane(1, 2.8, 2.8, {}, {1});

    auto r5_s1 = r5->AddSection({192.27, -105.54, 209.2014}, 15.0, 15.0);
    r5_s1->AddLane(1, 2.8, 3.75, {1}, {1});

    auto r5_s2 = r5->AddSection({165.19, -120.44, 209.2014});
    r5_s2->AddLane(1, 3.75, 3.75, {1}, {1, 2});

    auto r5_s3 = r5->AddSection({125.19, -144.53, 209.2014}, 10.0, 10.0);
    r5_s3->AddLane(1, 3.75, 2.8, {1}, {1});
    r5_s3->AddLane(2, 3.75, 5.8, {1}, {2});

    auto r5_s4 = r5->AddSection({101.60, -157.67, 209.2014});
    r5_s4->AddLane(1, 2.8, 2.8, {1}, {});
    r5_s4->AddLane(2, 5.8, 5.8, {2}, {});

    //Road[6/7]-------------------------------------------------------------
    auto r6 = map.AddRoad({261.36, -35.95, 126.342});

    auto r6_s0 = r6->AddSection({186.00, 66.68, 126.342});
    r6_s0->AddLane( 1, 3.78, 3.78, {}, {});
    r6_s0->AddLane( 2, 7.26, 7.26, {}, {});
    r6_s0->AddLane( 3, 10.36, 10.36, {}, {});

    r6->AddSignal(155.98, 107.59, 5.8, Angle(126.342), "SIG", "1000");
    r6->AddSignal(157.69, 108.56, 5.8, Angle(126.342), "SIG", "0011");

    auto r7 = map.AddRoad({185.59, 66.65, -53.6531});

    auto r7_s0 = r7->AddSection({260.70, -35.83, -53.6531});
    r7_s0->AddLane(1, 3.78, 3.78, {}, {});
    r7_s0->AddLane(2, 7.26, 7.26, {}, {});
    r7_s0->AddLane(3, 10.36, 10.36, {}, {});

    r7->AddSignal(290.11, -85.98, 5.8, Angle(-53.6531), "SIG", "1101");
    r7->AddSignal(288.32, -87.21, 5.8, Angle(-53.6531), "SIG", "0010");

    //Road[8/9]-------------------------------------------------------------
    auto r8 = map.AddRoad({4.89, 7.9, 25.7603});

    auto r8_s0 = r8->AddSection({145.26, 75.50, 25.7603});
    r8_s0->AddLane(1, 3.8, 3.8, {}, {});
    r8_s0->AddLane(2, 7.2, 7.2, {}, {});

    r8->AddSignal(209.943, 110.82, 5.8, Angle(25.7603), "SIG", "1000");
    r8->AddSignal(210.923, 108.89, 5.8, Angle(25.7603), "SIG", "0111");

    auto r9 = map.AddRoad({144.98, 75.84, -154.28});

    auto r9_s0 = r9->AddSection({4.88, 8.29, -154.28});
    r9_s0->AddLane( 1, 3.8, 3.8, {}, {});
    r9_s0->AddLane( 2, 7.2, 7.2, {}, {});

    r9->AddSignal(-43.856, -12.1609, 5.8, Angle(-154.28), "SIG", "1000");
    r9->AddSignal(-45.046, -10.3009,  5.8, Angle(-154.28), "SIG", "0100");

    //Road[10/11]-------------------------------------------------------------
    auto r10 = map.AddRoad({191.04, 97.12, 27.71});

    auto r10_s0 = r10->AddSection({219.94, 112.28, 27.71});
    r10_s0->AddLane(1, 2.8, 2.8, {}, {1});

    auto r10_s1 = r10->AddSection({247.09, 128.20, 27.71}, 18.0, 18.0);
    r10_s1->AddLane(1, 2.8, 4.5, {1}, {1});

    auto r10_s2 = r10->AddSection({264.66, 137.46, 27.71});
    r10_s2->AddLane(1, 4.5, 4.5, {1}, {1});

    auto r10_s3 = r10->AddSection({385.39, 133.44, -34.4855}, 60, 35);
    r10_s3->AddLane(1, 4.5, 4.5, {1}, {1});

    auto r10_s4 = r10->AddSection({471.82, 73.75, -34.4855});
    r10_s4->AddLane(1, 4.5, 4.5, {1}, {});

    auto r11 = map.AddRoad({471.82, 73.75, 145.5145});

    auto r11_s0 = r11->AddSection({385.39, 133.545, 145.5145});
    r11_s0->AddLane(1, 4.5, 4.5, {}, {1});

    auto r11_s1 = r11->AddSection({264.66, 137.46, -152.291}, 35, 60);
    r11_s1->AddLane(1, 4.5, 4.0, {1}, {1});

    auto r11_s2 = r11->AddSection({247.09, 128.20, -152.291});
    r11_s2->AddLane(1, 4.0, 4.0, {1}, {1, 2});

    auto r11_s3 = r11->AddSection({219.94, 112.28, -152.291}, 18.0, 18.0);
    r11_s3->AddLane(1, 4.0, 2.8, {1}, {1});
    r11_s3->AddLane(2, 4.0, 5.8, {1}, {2});

    auto r11_s4 = r11->AddSection({191.04, 97.12, -152.291});
    r11_s4->AddLane(1, 2.8, 2.8, {1}, {});
    r11_s4->AddLane(2, 5.8, 5.8, {2}, {});

    r11->AddSignal(139.28, 77.52, 5.90, Angle(-152.291), "SIG", "1000");
    r11->AddSignal(138.73, 79.22, 5.90, Angle(-152.291), "SIG", "0101");

    //Road[12/13]-------------------------------------------------------------
    auto r12 = map.AddRoad({300.35, -48.52, 28.804});

    auto r12_s0 = r12->AddSection({325.72, -34.59, 28.804});
    r12_s0->AddLane(1, 2.8, 2.8, {}, {1});

    auto r12_s1 = r12->AddSection({357.58, -13.68, 28.804}, 15.0, 15.0);
    r12_s1->AddLane(1, 2.8, 3.75, {1}, {1});

    auto r12_s2 = r12->AddSection({413.34, 17.35, 28.804});
    r12_s2->AddLane(1, 3.75, 3.75, {1}, {1, 2});

    auto r12_s3 = r12->AddSection({444.85, 36.20, 28.804}, 15.0, 15.0);
    r12_s3->AddLane(1, 3.75, 2.8, {1}, {1});
    r12_s3->AddLane(2, 3.75, 5.8, {1}, {2});

    auto r12_s4 = r12->AddSection({468.08, 49.12, 28.804});
    r12_s4->AddLane(1, 2.8, 2.8, {1}, {});
    r12_s4->AddLane(2, 5.8, 5.8, {2}, {});

    auto r13 = map.AddRoad({468.08, 49.12, -150.996});

    auto r13_s0 = r13->AddSection({444.85, 36.20, -150.996});
    r13_s0->AddLane(1, 2.8, 2.8, {}, {1});

    auto r13_s1 = r13->AddSection({413.34, 17.35, -150.996}, 15.0, 15.0);
    r13_s1->AddLane(1, 2.8, 3.75, {1}, {1});

    auto r13_s2 = r13->AddSection({357.58, -13.68, -150.996});
    r13_s2->AddLane(1, 3.75, 3.75, {1}, {1, 2, 3});

    auto r13_s3 = r13->AddSection({325.72, -34.59, -150.996}, 15.0, 15.0);
    r13_s3->AddLane(1, 3.75, 2.8, {1}, {1});
    r13_s3->AddLane(2, 3.75, 5.8, {1}, {2});
    r13_s3->AddLane(3, 3.75, 8.5, {1}, {3});

    auto r13_s4 = r13->AddSection({300.35, -48.52, -150.996});
    r13_s4->AddLane(1, 2.8, 2.8, {1}, {});
    r13_s4->AddLane(2, 5.8, 5.8, {2}, {});
    r13_s4->AddLane(3, 8.5, 8.5, {3}, {});

    r13->AddSignal(249.52, -73.40, 5.8, Angle(-150.996), "SIG", "1101");
    r13->AddSignal(248.53, -71.63, 5.8, Angle(-150.996), "SIG", "0010");

    //Road[14/15]-------------------------------------------------------------

    auto r14 = map.AddRoad({379.61, -208.74, 124.063});

    auto r14_s0 = r14->AddSection({293.00, -80.72, 124.063});
    r14_s0->AddLane(1, 3.78, 3.78, {}, {});
    r14_s0->AddLane(2, 7.26, 7.26, {}, {});
    r14_s0->AddLane(3, 10.36, 10.36, {}, {});

    r14->AddSignal(263.43, -30.72, 5.8, Angle(124.063), "SIG", "1101");
    r14->AddSignal(265.23, -29.41, 5.8, Angle(124.063), "SIG", "0010");

    auto r15 = map.AddRoad({292.63, -80.90, -55.9177});

    auto r15_s0 = r15->AddSection({379.17, -208.84, -55.9177});
    r15_s0->AddLane(1, 3.7, 3.7, {}, {});
    r15_s0->AddLane(2, 7.3, 7.3, {}, {});
    r15_s0->AddLane(3, 10.5, 10.5, {}, {});


    //Road[16/17]-------------------------------------------------------------
    auto r16 = map.AddRoad({498.68, 50.94, -40.875});

    auto r16_s0 = r16->AddSection({511.37, 40.50, -40.875});
    r16_s0->AddLane(1, 2.8, 2.8, {}, {1});

    auto r16_s1 = r16->AddSection({545.20, 12.10, -40.875}, 15.0, 15.0);
    r16_s1->AddLane(1, 2.8, 4.5, {1}, {1});

    auto r16_s2 = r16->AddSection({636.42, -70.69, -40.875});
    r16_s2->AddLane(1, 4.5, 4.5, {1}, {1});

    auto r17 = map.AddRoad({636.42, -70.69, 139.125});

    auto r17_s0 = r17->AddSection({545.20, 12.10, 139.125});
    r17_s0->AddLane(1, 4.5, 4.5, {}, {1, 2});

    auto r17_s1 = r17->AddSection({511.37, 40.50, 139.125}, 15.0, 15.0);
    r17_s1->AddLane(1, 4.5, 2.8, {1}, {1});
    r17_s1->AddLane(2, 4.5, 5.8, {1}, {2});

    auto r17_s2 = r17->AddSection({498.68, 50.94, 139.125});
    r17_s2->AddLane(1, 2.8, 2.8, {1}, {});
    r17_s2->AddLane(2, 5.8, 5.8, {2}, {});


    //Road[18/19]-------------------------------------------------------------

    auto r18 = map.AddRoad({417.10, -220.23, 29.437});

    auto r18_s0 = r18->AddSection({443.66, -204.82, 29.437});
    r18_s0->AddLane(1, 2.8, 2.8, {}, {1});

    auto r18_s1 = r18->AddSection({470.57, -187.08, 29.437}, 15.9, 10.0);
    r18_s1->AddLane(1, 2.8, 4.0, {1}, {1});

    auto r18_s2 = r18->AddSection({582.31, -123.57, 29.437});
    r18_s2->AddLane(1, 4.0, 4.0, {1}, {1, 2});

    auto r18_s3 = r18->AddSection({607.80, -107.74, 29.437}, 10.0, 10.0);
    r18_s3->AddLane(1, 4.0, 2.8, {1}, {1});
    r18_s3->AddLane(2, 4.0, 5.8, {1}, {2});

    auto r18_s4 = r18->AddSection({632.79, -93.95, 29.437});
    r18_s4->AddLane(1, 2.8, 2.8, {1}, {});
    r18_s4->AddLane(2, 5.8, 5.8, {2}, {});

    auto r19 = map.AddRoad({632.79, -93.95, -150.563});

    auto r19_s0 = r19->AddSection({607.80, -107.74, -150.563});
    r19_s0->AddLane(1, 2.8, 2.8, {}, {1});

    auto r19_s1 = r19->AddSection({582.31, -123.57, -150.563}, 10.0, 10.0);
    r19_s1->AddLane(1, 2.8, 3.75, {1}, {1});

    auto r19_s2 = r19->AddSection({470.57, -187.08, -150.563});
    r19_s2->AddLane(1, 3.75, 3.75, {1}, {1, 2, 3});

    auto r19_s3 = r19->AddSection({443.66, -204.82, -150.563}, 10.0, 15.9);
    r19_s3->AddLane(1, 3.75, 2.75, {1}, {1});
    r19_s3->AddLane(2, 3.75, 5.6, {1}, {2});
    r19_s3->AddLane(3, 3.75, 8.5, {1}, {3});

    auto r19_s4 = r19->AddSection({417.10, -220.23, -150.563});
    r19_s4->AddLane(1, 2.75, 2.75, {1}, {});
    r19_s4->AddLane(2, 5.6, 5.6, {2}, {});
    r19_s4->AddLane(3, 8.5, 8.5, {3}, {});


    //Road[20/21]-------------------------------------------------------------
    auto r20 = map.AddRoad({217.07, -331.82, 29.337});

    auto r20_s0 = r20->AddSection({250.84, -312.84, 29.337});
    r20_s0->AddLane(1, 2.8, 2.7, {}, {1});

    auto r20_s1 = r20->AddSection({271.36, -299.60, 29.337}, 15.0, 15.0);
    r20_s1->AddLane(1, 2.7, 3.75, {1}, {1});

    auto r20_s2 = r20->AddSection({312.28, -276.52, 29.337});
    r20_s2->AddLane(1, 3.75, 3.75, {1}, {1, 2, 3});

    auto r20_s3 = r20->AddSection({337.63, -259.21, 29.337}, 15.0, 15.0);
    r20_s3->AddLane(1, 3.75, 2.8, {1}, {1});
    r20_s3->AddLane(2, 3.75, 5.8, {1}, {2});
    r20_s3->AddLane(3, 3.75, 8.5, {1}, {3});

    auto r20_s4 = r20->AddSection({369.39, -240.99, 29.337});
    r20_s4->AddLane(1, 2.8, 2.8, {1}, {});
    r20_s4->AddLane(2, 5.8, 5.8, {2}, {});
    r20_s4->AddLane(3, 8.5, 8.5, {3}, {});

    auto r21 = map.AddRoad({369.39, -240.99, -150.663});

    auto r21_s0 = r21->AddSection({337.63, -259.21, -150.663});
    r21_s0->AddLane(1, 2.8, 2.8, {}, {1});

    auto r21_s1 = r21->AddSection({312.28, -276.52, -150.663}, 15.0, 15.0);
    r21_s1->AddLane(1, 2.8, 3.75, {1}, {1});

    auto r21_s2 = r21->AddSection({271.36, -299.60, -150.663});
    r21_s2->AddLane(1, 3.75, 3.75, {1}, {1, 2});

    auto r21_s3 = r21->AddSection({250.84, -312.84, -150.663}, 15.0, 15.0);
    r21_s3->AddLane(1, 3.75, 2.8, {1}, {1});
    r21_s3->AddLane(2, 3.75, 5.8, {1}, {2});

    auto r21_s4 = r21->AddSection({217.07, -331.82, -150.663});
    r21_s4->AddLane(1, 2.8, 2.8, {1}, {});
    r21_s4->AddLane(2, 5.8, 5.8, {2}, {});


    //Road[22/23]-------------------------------------------------------------
    auto r22 = map.AddRoad({33.65, -432.96, 29.464});

    auto r22_s0 = r22->AddSection({127.70, -379.84, 29.464});
    r22_s0->AddLane(1, 3.75, 3.75, {}, {1, 2});

    auto r22_s1 = r22->AddSection({148.53, -366.65, 29.464}, 10.0, 10.0);
    r22_s1->AddLane(1, 3.75, 2.8, {1}, {1});
    r22_s1->AddLane(2, 3.75, 5.8, {1}, {2});

    auto r22_s2 = r22->AddSection({182.87, -347.25, 29.464});
    r22_s2->AddLane(1, 2.8, 2.8, {1}, {});
    r22_s2->AddLane(2, 5.8, 5.8, {2}, {});

    auto r23 = map.AddRoad({182.87, -347.25, -150.536});

    auto r23_s0 = r23->AddSection({148.53, -366.65, -150.536});
    r23_s0->AddLane(1, 2.8, 2.8, {}, {1});

    auto r23_s1 = r23->AddSection({127.70, -379.84, -150.536}, 10.0, 10.0);
    r23_s1->AddLane(1, 2.8, 3.75, {1}, {1});

    auto r23_s2 = r23->AddSection({33.65, -432.96, -150.536});
    r23_s2->AddLane(1, 3.75, 3.75, {1}, {});


    //Road[24/25]-------------------------------------------------------------
    auto r24 = map.AddRoad({95.26, -181.15, -56.9236});

    auto r24_s0 = r24->AddSection({111.79, -206.53, -56.9236});
    r24_s0->AddLane(1, 3.0, 3.0, {}, {1});

    auto r24_s1 = r24->AddSection({124.77, -224.02, -56.9236}, 10.0, 10.0);
    r24_s1->AddLane(1, 3.0, 4.5, {1}, {1});

    auto r24_s2 = r24->AddSection({159.60, -277.22, -56.9236});
    r24_s2->AddLane(1, 4.5, 4.5, {1}, {1, 2});

    auto r24_s3 = r24->AddSection({173.65, -296.34, -56.9236}, 15.0, 15.0);
    r24_s3->AddLane(1, 4.5, 2.8, {1}, {1});
    r24_s3->AddLane(2, 4.5, 5.8, {1}, {2});

    auto r24_s4 = r24->AddSection({191.13, -323.29, -56.9236});
    r24_s4->AddLane(1, 2.8, 2.8, {1}, {});
    r24_s4->AddLane(2, 5.8, 5.8, {2}, {});

    auto r25 = map.AddRoad({191.13, -323.29, 122.9764});

    auto r25_s0 = r25->AddSection({173.65, -296.34, 122.9764});
    r25_s0->AddLane(1, 2.8, 2.8, {}, {1});

    auto r25_s1 = r25->AddSection({159.60, -277.22, 122.9764}, 15.0, 15.0);
    r25_s1->AddLane(1, 2.8, 4.6, {1}, {1});

    auto r25_s2 = r25->AddSection({124.77, -224.02, 122.9764});
    r25_s2->AddLane(1, 4.6, 4.6, {1}, {1, 2});

    auto r25_s3 = r25->AddSection({111.79, -206.53, 122.9764}, 10.0, 10.0);
    r25_s3->AddLane(1, 4.6, 2.8, {1}, {1});
    r25_s3->AddLane(2, 4.6, 5.8, {1}, {2});

    auto r25_s4 = r25->AddSection({95.26, -181.15, 122.9764});
    r25_s4->AddLane(1, 2.8, 2.8, {1}, {});
    r25_s4->AddLane(2, 5.8, 5.8, {2}, {});


    //Road[26/27]-------------------------------------------------------------
    auto r26 = map.AddRoad({-99.93, -269.22, 29.1489});

    auto r26_s0 = r26->AddSection({-64.32, -249.36, 29.1489});
    r26_s0->AddLane(1, 2.8, 2.8, {}, {1});

    auto r26_s1 = r26->AddSection({-45.72, -237.33, 29.1489}, 15.0, 15.0);
    r26_s1->AddLane(1, 2.8, 3.75, {1}, {1});

    auto r26_s2 = r26->AddSection({21.28, -200.07, 29.1489});
    r26_s2->AddLane(1, 3.75, 3.75, {1}, {1, 2});

    auto r26_s3 = r26->AddSection({41.01, -187.68, 29.1489}, 15.0, 15.0);
    r26_s3->AddLane(1, 3.75, 2.8, {1}, {1});
    r26_s3->AddLane(2, 3.75, 5.8, {1}, {2});

    auto r26_s4 = r26->AddSection({71.35, -169.78, 29.1489});
    r26_s4->AddLane(1, 2.8, 2.8, {1}, {});
    r26_s4->AddLane(2, 5.8, 5.8, {2}, {});

    auto r27 = map.AddRoad({71.35, -169.78, -150.851});

    auto r27_s0 = r27->AddSection({41.01, -187.68, -150.851});
    r27_s0->AddLane(1, 2.8, 2.8, {}, {1});

    auto r27_s1 = r27->AddSection({21.28, -200.07, -150.851}, 15.0, 15.0);
    r27_s1->AddLane(1, 2.8, 3.75, {1}, {1});

    auto r27_s2 = r27->AddSection({-45.72, -237.33, -150.851});
    r27_s2->AddLane(1, 3.75, 3.75, {1}, {1, 2});

    auto r27_s3 = r27->AddSection({-64.32, -249.36, -150.851}, 15.0, 15.0);
    r27_s3->AddLane(1, 3.75, 2.8, {1}, {1});
    r27_s3->AddLane(2, 3.75, 5.8, {1}, {2});

    auto r27_s4 = r27->AddSection({-99.93, -269.22, -150.851});
    r27_s4->AddLane(1, 2.8, 2.8, {1}, {});
    r27_s4->AddLane(2, 5.8, 5.8, {2}, {});


    //Road[28/29]-------------------------------------------------------------
    auto r28 = map.AddRoad({209.43, -356.33, -56.7655});

    auto r28_s0 = r28->AddSection({230.51, -388.57, -56.7655});
    r28_s0->AddLane(1, 2.8, 2.8, {}, {1});

    auto r28_s1 = r28->AddSection({247.86, -412.75, -56.7655}, 15.0, 15.0);
    r28_s1->AddLane(1, 2.8, 3.75, {1}, {1});

    auto r28_s2 = r28->AddSection({290.47, -477.81, -56.7655});
    r28_s2->AddLane(1, 3.75, 3.75, {1}, {1, 2});

    auto r28_s3 = r28->AddSection({305.42, -497.99, -56.7655}, 15.0, 15.0);
    r28_s3->AddLane(1, 3.75, 2.8, {1}, {1});
    r28_s3->AddLane(2, 3.75, 5.8, {1}, {2});

    auto r28_s4 = r28->AddSection({324.92, -527.75, -56.7655});
    r28_s4->AddLane(1, 2.8, 2.8, {1}, {});
    r28_s4->AddLane(2, 5.8, 5.8, {2}, {});

    auto r29 = map.AddRoad({324.92, -527.75, 123.2345});

    auto r29_s0 = r29->AddSection({305.42, -497.99, 123.2345});
    r29_s0->AddLane(1, 2.8, 2.8, {}, {1});

    auto r29_s1 = r29->AddSection({290.47, -477.81, 123.2345}, 15.0, 15.0);
    r29_s1->AddLane(1, 2.8, 3.75, {1}, {1});

    auto r29_s2 = r29->AddSection({247.86, -412.75, 123.2345});
    r29_s2->AddLane(1, 3.75, 3.75, {1}, {1, 2});

    auto r29_s3 = r29->AddSection({230.51, -388.57, 123.2345}, 15.0, 15.0);
    r29_s3->AddLane(1, 3.75, 2.8, {1}, {1});
    r29_s3->AddLane(2, 3.75, 5.8, {1}, {2});

    auto r29_s4 = r29->AddSection({209.43, -356.33, 123.2345});
    r29_s4->AddLane(1, 2.8, 2.8, {1}, {});
    r29_s4->AddLane(2, 5.8, 5.8, {2}, {});

    //Road[30/31]-------------------------------------------------------------
    auto r30 = map.AddRoad({510.45, -408.00, 123.431});

    auto r30_s0 = r30->AddSection({407.71, -252.37, 123.431});
    r30_s0->AddLane(1, 3.78, 3.78, {}, {});
    r30_s0->AddLane(2, 7.26, 7.26, {}, {});
    r30_s0->AddLane(3, 10.36, 10.36, {}, {});

    auto r31 = map.AddRoad({407.48, -252.39, -56.7});

    auto r31_s0 = r31->AddSection({510.15, -408.69, -56.7});
    r31_s0->AddLane(1, 3.78, 3.78, {}, {});
    r31_s0->AddLane(2, 7.26, 7.26, {}, {});
    r31_s0->AddLane(3, 10.36, 10.36, {}, {});

    //Road[32/33]-------------------------------------------------------------
    auto r32 = map.AddRoad({-136.71, -266.72, 129.584});

    auto r32_s0 = r32->AddSection({-198.20, -192.35, 129.584});
    r32_s0->AddLane(1, 3.3, 3.3, {}, {1});
    r32_s0->AddLane(2, 6.6, 6.6, {}, {2});
    r32_s0->AddLane(3, 10.0, 10.0, {}, {3});

    auto r32_s1 = r32->AddSection({-237.77, -142.70, 128.94}, 10.0, 10.0);
    r32_s1->AddLane(1, 3.3, 3.3, {1}, {});
    r32_s1->AddLane(2, 6.6, 6.6, {2}, {});
    r32_s1->AddLane(3, 10.0, 10.0, {3}, {});

    auto r33 = map.AddRoad({-239.71, -146.33, -50.3793});

    auto r33_s0 = r33->AddSection({-205.17, -191.95, -50.3793}, 10.0, 10.0);
    r33_s0->AddLane(1, 3.3, 3.3, {}, {1});
    r33_s0->AddLane(2, 6.6, 6.6, {}, {2});
    r33_s0->AddLane(3, 10.0, 10.0, {}, {3});

    auto r33_s1 = r33->AddSection({-140.38, -270.21, -50.3793});
    r33_s1->AddLane(1, 3.3, 3.3, {1}, {});
    r33_s1->AddLane(2, 6.6, 6.6, {2}, {});
    r33_s1->AddLane(3, 10.0, 10.0, {3}, {});

    //Road[34,35,36]-------------------------------------------------------------
    auto r34 = map.AddRoad({156.66, -620.63, 129.682});

    auto r34_s0 = r34->AddSection({24.7, -461.61, 129.682});
    r34_s0->AddLane(1, 3.3, 3.3, {}, {});
    r34_s0->AddLane(2, 6.6, 6.6, {}, {});
    r34_s0->AddLane(3, 10.0, 10.0, {}, {});

    auto r35 = map.AddRoad({-4.99, -425.97, 129.653});
    auto r35_s0 = r35->AddSection({-79.86, -335.44, 129.653});
    r35_s0->AddLane(1, 3.3, 3.3, {}, {});
    r35_s0->AddLane(2, 6.6, 6.6, {}, {});
    r35_s0->AddLane(3, 10.0, 10.0, {}, {});


    auto r36 = map.AddRoad({-83.36, -338.57, -50.3428});

    auto r36_s0 = r36->AddSection({111.02, -572.593, -50.3428});
    r36_s0->AddLane(1, 3.3, 3.3, {}, {1});
    r36_s0->AddLane(2, 6.6, 6.6, {}, {2, 3});
    r36_s0->AddLane(3, 10.0, 10.0, {}, {4, 5});

    auto r36_s1 = r36->AddSection({133.48, -597.48, -51.3966}, 15.0, 15.0);
    r36_s1->AddLane(1, 3.3, 2.7, {1}, {1});
    r36_s1->AddLane(2, 6.6, 5.5, {2}, {2});
    r36_s1->AddLane(3, 6.6, 8.5, {2}, {3});
    r36_s1->AddLane(4, 10.0, 11.3, {3}, {4});
    r36_s1->AddLane(5, 10.0, 14.3, {3}, {5});

    auto r36_s2 = r36->AddSection({154.3, -622.84, -51.3966});
    r36_s2->AddLane(1, 2.7, 2.7, {1}, {});
    r36_s2->AddLane(2, 5.5, 5.5, {2}, {});
    r36_s2->AddLane(3, 8.5, 8.5, {3}, {});
    r36_s2->AddLane(4, 11.3, 11.3, {4}, {});
    r36_s2->AddLane(5, 14.3, 14.3, {5}, {});


    //Road[37/38]-------------------------------------------------------------
    auto r37 = map.AddRoad({199.86, -633.16, 29.6444});

    auto r37_s0 = r37->AddSection({237.20, -611.76, 29.6444});
    r37_s0->AddLane(1, 3.3, 3.3, {}, {1});
    r37_s0->AddLane(2, 6.6, 6.6, {}, {2});
    r37_s0->AddLane(3, 10.0, 10.0, {}, {3});

    auto r37_s1 = r37->AddSection({323.54, -560.25, 29.6444});
    r37_s1->AddLane(1, 3.3, 3.3, {1}, {});
    r37_s1->AddLane(2, 6.6, 6.6, {2}, {});
    r37_s1->AddLane(3, 10.0, 10.0, {3}, {});


    auto r38 = map.AddRoad({320.60, -556.34, -149.223});

    auto r38_s0 = r38->AddSection({253.96, -595.56, -149.223});
    r38_s0->AddLane(1, 3.3, 3.3, {}, {1});
    r38_s0->AddLane(2, 6.6, 6.6, {}, {2});
    r38_s0->AddLane(3, 10.0, 10.0, {}, {3, 4});

    auto r38_s1 = r38->AddSection({235.96, -608.58, -149.223}, 5.8, 5.8);
    r38_s1->AddLane(1, 3.3, 2.8, {1}, {1});
    r38_s1->AddLane(2, 6.6, 5.8, {2}, {2});
    r38_s1->AddLane(3, 10.0, 9.25, {3}, {3});
    r38_s1->AddLane(4, 10.0, 12.4, {3}, {4});

    auto r38_s2 = r38->AddSection({197.67, -630.61, -149.223});
    r38_s2->AddLane(1, 2.8, 2.8, {1}, {});
    r38_s2->AddLane(2, 5.8, 5.8, {2}, {});
    r38_s2->AddLane(3, 9.25, 9.25, {3}, {});
    r38_s2->AddLane(4, 12.4, 12.4, {4}, {});


    //Road[39/40]-------------------------------------------------------------
    auto r39 = map.AddRoad({354.66, -541.98, 30.6572});

    auto r39_s0 = r39->AddSection({421.05, -502.71, 30.6572});
    r39_s0->AddLane(1, 3.3, 3.3, {}, {1});
    r39_s0->AddLane(2, 6.6, 6.6, {}, {2});
    r39_s0->AddLane(3, 10.0, 10.0, {}, {3, 4});

    auto r39_s1 = r39->AddSection({467.31, -472.81, 30.6572}, 2.0, 30.0);
    r39_s1->AddLane(1, 3.3, 3.6, {1}, {1});
    r39_s1->AddLane(2, 6.6, 7.2, {2}, {2});
    r39_s1->AddLane(3, 10.0, 10.8, {3}, {3});
    r39_s1->AddLane(4, 10.0, 14.0, {3}, {4});

    auto r39_s2 = r39->AddSection({508.43, -448.53, 30.6572});
    r39_s2->AddLane(1, 3.6, 3.6, {1}, {});
    r39_s2->AddLane(2, 7.2, 7.2, {2}, {});
    r39_s2->AddLane(3, 10.8, 10.8, {3}, {});
    r39_s2->AddLane(4, 14.0, 14.0, {4}, {});

    auto r40 = map.AddRoad({506.51, -445.82, -149.314});

    auto r40_s0 = r40->AddSection({351.40, -538.16, -149.314});
    r40_s0->AddLane(1, 3.3, 3.3, {}, {});
    r40_s0->AddLane(2, 6.6, 6.6, {}, {});
    r40_s0->AddLane(3, 10.0, 10.0, {}, {});

    //Road[41]-------------------------------------------------------------
    ///越野路
    auto r41 = map.AddRoad({656.89, -108.49, 281.966});

    auto r41_s0 = r41->AddSection({645.97, -147.35, 221.641}, 18.0, 18.0);
    r41_s0->AddLane(1, 2.0, 2.0, {}, {1});

    auto r41_s1 = r41->AddSection({585.36, -153.42, 193.115}, 28.0, 18.8);
    r41_s1->AddLane(1, 2.0, 2.0, {1}, {1});

    auto r41_s2 = r41->AddSection({555.50, -198.39, 283.853}, 24.0, 12.9);
    r41_s2->AddLane(1, 2.0, 2.0, {1}, {1});

    auto r41_s3 = r41->AddSection({557.376, -225.416, 115.21+90.0}, 15.0, 11.0);
    r41_s3->AddLane(1, 2.0, 2.0, {1}, {1});

    auto r41_s4 = r41->AddSection({547.87, -224.35, 150.031}, 2.8, 2.8);
    r41_s4->AddLane(1, 2.0, 2.0, {1}, {1});

    auto r41_s5 = r41->AddSection({543.17, -239.72, 346.771}, 21.0, 17.0);
    r41_s5->AddLane(1, 2.0, 2.0, {1}, {1});

    auto r41_s6 = r41->AddSection({632.54, -258.03, -7.74});
    r41_s6->AddLane(1, 2.0, 2.0, {1}, {1});

    auto r41_s7 = r41->AddSection({635.97, -281.87, 182.23}, 29.5, 15.0);
    r41_s7->AddLane(1, 2.0, 2.0, {1}, {1});

    auto r41_s8 = r41->AddSection({591.60, -283.30, 94.05+90.0});
    r41_s8->AddLane(1, 2.0, 2.0, {1}, {1});

    auto r41_s9 = r41->AddSection({579.24, -295.00, 259.331}, 8.5, 8.5);
    r41_s9->AddLane(1, 2.0, 2.0, {1}, {1});

    auto r41_s10 = r41->AddSection({555.16, -306.33, 146.274}, 18.0, 17.0);
    r41_s10->AddLane(1, 2.0, 2.0, {1}, {1});

    auto r41_s11 = r41->AddSection({537.43, -312.23, 254.844}, 12.5, 12.5);
    r41_s11->AddLane(1, 2.0, 2.0, {1}, {1});

    auto r41_s12 = r41->AddSection({508.31, -319.25, 122.21}, 25.0, 16.5);
    r41_s12->AddLane(1, 2.0, 2.0, {1}, {1});

    auto r41_s13 = r41->AddSection({448.22, -227.68, 34.85+90.0});
    r41_s13->AddLane(1, 2.0, 2.0, {1}, {});

    map.CommitRoadInfo();
    //endregion

    //region Junction
    //Junc[0]-------------------------------------------------------------

    auto junc0 = map.AddJunction();

    map.AddRoadLink(junc0, 0, 8, "forward", {make_tuple(1, 1, 10.0, 10.0), make_tuple(2, 2, 10.0, 10.0)});
    map.AddRoadLink(junc0, 0, 2, "right", {make_tuple(3, 1, 20.0, 15.0)});
    map.AddRoadLink(junc0, 3, 1, "left", {make_tuple(1, 1, 15.0, 15.0), make_tuple(1, 2, 15.0, 15.0)});
    map.AddRoadLink(junc0, 3, 8, "right", {make_tuple(2, 1, 15.0, 15.0), make_tuple(2, 2, 15.0, 15.0)});
    map.AddRoadLink(junc0, 9, 1, "forward", {make_tuple(1, 1, 15.0, 15.0), make_tuple(2, 2, 15.0, 15.0)});
    map.AddRoadLink(junc0, 9, 2, "left", {make_tuple(1, 1, 15.0, 15.0)});


    //Junc[1]-------------------------------------------------------------
    auto junc1 = map.AddJunction();
    map.AddRoadLink(junc1, 2, 4, "left", {make_tuple(1, 1, 15.0, 15.0)});
    map.AddRoadLink(junc1, 2, 24, "forward", {make_tuple(2, 1, 10.0, 10.0)});
    map.AddRoadLink(junc1, 2, 27, "right", {make_tuple(2, 1, 10.0, 10.0)});

    map.AddRoadLink(junc1, 5, 24, "left", {make_tuple(1, 1, 15.0, 15.0)});
    map.AddRoadLink(junc1, 5, 27, "forward", {make_tuple(2, 1, 10.0, 10.0)});
    map.AddRoadLink(junc1, 5, 3, "right", {make_tuple(2, 1, 10.0, 10.0)});

    map.AddRoadLink(junc1, 25, 27, "left", {make_tuple(1, 1, 15.0, 15.0)});
    map.AddRoadLink(junc1, 25, 3, "forward", {make_tuple(2, 1, 10.0, 10.0)});
    map.AddRoadLink(junc1, 25, 4, "right", {make_tuple(2, 1, 10.0, 10.0)});

    map.AddRoadLink(junc1, 26, 3, "left", {make_tuple(1, 1, 15.0, 10.0)});
    map.AddRoadLink(junc1, 26, 4, "forward", {make_tuple(2, 1, 10.0, 10.0)});
    map.AddRoadLink(junc1, 26, 24, "right", {make_tuple(2, 1, 10.0, 10.0)});
    //Junc[2]-------------------------------------------------------------
    auto junc2 = map.AddJunction();

    map.AddRoadLink(junc2, 4, 6, "left", {make_tuple(1, 2, 15.0, 15.0)});
    map.AddRoadLink(junc2, 4, 12, "forward", {make_tuple(2, 1, 10.0, 10.0)});
    map.AddRoadLink(junc2, 4, 15, "right", {make_tuple(3, 2, 15.0, 15.0)});

    map.AddRoadLink(junc2, 14, 5, "left", {make_tuple(1, 1, 15.0, 15.0)});
    map.AddRoadLink(junc2, 14, 6, "forward", {make_tuple(2, 2, 10.0, 10.0)});
    map.AddRoadLink(junc2, 14, 12, "right", {make_tuple(3, 1, 15.0, 15.0)});

    map.AddRoadLink(junc2, 13, 15, "left", {make_tuple(1, 2, 15.0, 15.0)});
    map.AddRoadLink(junc2, 13, 5, "forward", {make_tuple(2, 1, 10.0, 10.0)});
    map.AddRoadLink(junc2, 13, 6, "right", {make_tuple(3, 2, 15.0, 15.0)});

    map.AddRoadLink(junc2, 7, 12, "left", {make_tuple(1, 1, 15.0, 15.0)});
    map.AddRoadLink(junc2, 7, 15, "forward", {make_tuple(2, 2, 10.0, 10.0)});
    map.AddRoadLink(junc2, 7, 5, "right", {make_tuple(3, 1, 15.0, 15.0)});

    //Junc[3]-------------------------------------------------------------
    auto junc3 = map.AddJunction();
    map.AddRoadLink(junc3, 6, 9, "left", {make_tuple(1, 1, 15.0, 15.0)});
    map.AddRoadLink(junc3, 6, 10, "right", {make_tuple(3, 1, 10.0, 15.0)});

    map.AddRoadLink(junc3, 8, 10, "forward", {make_tuple(1, 1, 10.0, 10.0)});
    map.AddRoadLink(junc3, 8, 7, "right", {make_tuple(2, 2, 15.0, 15.0)});

    map.AddRoadLink(junc3, 11, 7, "left", {make_tuple(1, 2, 15.0, 15.0)});
    map.AddRoadLink(junc3, 11, 9, "forward", {make_tuple(2, 1, 10.0, 10.0)});

    //Junc[4]-------------------------------------------------------------
    auto junc4 = map.AddJunction();
    map.AddRoadLink(junc4, 10, 16, "forward", {make_tuple(1, 1, 10.0, 10.0)});
    map.AddRoadLink(junc4, 10, 13, "right",{make_tuple(1, 1, 10.0, 15.0)});

    map.AddRoadLink(junc4, 12, 11, "left", {make_tuple(1, 1, 15.0, 15.0)});
    map.AddRoadLink(junc4, 12, 16, "right", {make_tuple(2, 1, 15.0, 15.0)});

    map.AddRoadLink(junc4, 17, 13, "left", {make_tuple(1, 1, 15.0, 15.0)});
    map.AddRoadLink(junc4, 17, 11, "forward", {make_tuple(2, 1, 10.0, 10.0)});

    //Junc[5]-------------------------------------------------------------
    auto junc5 = map.AddJunction();
    map.AddRoadLink(junc5, 16, 19, "right", {make_tuple(1, 1, 10.0, 10.0)});

    map.AddRoadLink(junc5, 18, 17, "left", {make_tuple(1, 1, 15.0, 15.0)});

    //Junc[6]-------------------------------------------------------------
    auto junc6 = map.AddJunction();

    map.AddRoadLink(junc6, 15, 18, "left", {make_tuple(1, 1, 15.0, 15.0)});
    map.AddRoadLink(junc6, 15, 31, "forward", {make_tuple(2, 2, 10.0, 10.0)});
    map.AddRoadLink(junc6, 15, 21, "right", {make_tuple(3, 1, 18.0, 15.0)});

    map.AddRoadLink(junc6, 20, 14, "left", {make_tuple(1, 2, 15.0, 15.0)});
    map.AddRoadLink(junc6, 20, 18, "forward", {make_tuple(2, 1, 10.0, 10.0)});
    map.AddRoadLink(junc6, 20, 31, "right", {make_tuple(3, 2, 15.0, 15.0)});

    map.AddRoadLink(junc6, 19, 31, "left", {make_tuple(1, 2, 15.0, 15.0)});
    map.AddRoadLink(junc6, 19, 21, "forward", {make_tuple(2, 1, 10.0, 10.0)});
    map.AddRoadLink(junc6, 19, 14, "right", {make_tuple(3, 2, 15.0, 15.0)});

    map.AddRoadLink(junc6, 30, 21, "left", {make_tuple(1, 1, 15.0, 15.0)});
    map.AddRoadLink(junc6, 30, 14, "forward", {make_tuple(2, 2, 10.0, 10.0)});
    map.AddRoadLink(junc6, 30, 18, "right", {make_tuple(3, 1, 15.0, 15.0)});

    //Junc[7]-------------------------------------------------------------
    auto junc7 = map.AddJunction();
    map.AddRoadLink(junc7, 24, 20, "left", {make_tuple(1, 1, 15.0, 15.0)});
    map.AddRoadLink(junc7, 24, 28, "forward", {make_tuple(2, 1, 10.0, 10.0)});
    map.AddRoadLink(junc7, 24, 23, "right", {make_tuple(2, 1, 15.0, 15.0)});

    map.AddRoadLink(junc7, 22, 25, "left", {make_tuple(1, 1, 15.0, 15.0)});
    map.AddRoadLink(junc7, 22, 20, "forward", {make_tuple(2, 1, 10.0, 10.0)});
    map.AddRoadLink(junc7, 22, 28, "right", {make_tuple(2, 1, 15.0, 15.0)});

    map.AddRoadLink(junc7, 21, 28, "left", {make_tuple(1, 1, 10.0, 10.0)});
    map.AddRoadLink(junc7, 21, 23, "forward", {make_tuple(2, 1, 10.0, 10.0)});
    map.AddRoadLink(junc7, 21, 25, "right", {make_tuple(2, 1, 15.0, 15.0)});

    map.AddRoadLink(junc7, 29, 23, "left", {make_tuple(1, 1, 15.0, 15.0)});
    map.AddRoadLink(junc7, 29, 25, "forward", {make_tuple(2, 1, 10.0, 10.0)});
    map.AddRoadLink(junc7, 29, 20, "right", {make_tuple(2, 1, 15.0, 15.0)});
    //Junc[8]-------------------------------------------------------------
    auto junc8 = map.AddJunction();
    map.AddRoadLink(junc8, 23, 35, "right", {make_tuple(1, 2, 15.0, 15.0)});

    map.AddRoadLink(junc8, 34, 35, "forward", {make_tuple(1, 1, 10.0, 10.0), make_tuple(2, 2, 10.0, 10.0)});

    map.AddRoadLink(junc8, 34, 22, "right", {make_tuple(3, 1, 15.0, 15.0)});

    //Junc[9]-------------------------------------------------------------
    auto junc9 = map.AddJunction();
    map.AddRoadLink(junc9, 35, 32, "forward", {make_tuple(1, 1, 10.0, 10.0), make_tuple(2, 2, 10.0, 10.0)});

    map.AddRoadLink(junc9, 33, 36, "forward", {make_tuple(1, 1, 10.0, 10.0)});

    map.AddRoadLink(junc9, 27, 32, "right", {make_tuple(2, 2, 15.0, 15.0)});

    map.AddRoadLink(junc9, 35, 26, "right", {make_tuple(3, 1, 60.0, 20.0)});


    //Junc[10]-------------------------------------------------------------
    auto junc10 = map.AddJunction();
    map.AddRoadLink(junc10, 32, 0, "right", {make_tuple(3, 2, 15.0, 15.0)});

    map.AddRoadLink(junc10, 1, 33, "left", {make_tuple(1, 2, 15.0, 15.0)});


    //Junc[11]-------------------------------------------------------------
    auto junc11 = map.AddJunction();

    map.AddRoadLink(junc11, 38, 34, "right", {make_tuple(4, 2, 15.0, 15.0)});

    map.AddRoadLink(junc11, 36, 37, "left", {make_tuple(1, 2, 15.0, 15.0)});


    //Junc[12]-------------------------------------------------------------
    auto junc12 = map.AddJunction();
    map.AddRoadLink(junc12, 37, 39, "forward", {make_tuple(1, 1, 10.0, 10.0), make_tuple(2, 2, 10.0, 10.0)});

    map.AddRoadLink(junc12, 40, 38, "forward", {make_tuple(1, 1, 10.0, 10.0), make_tuple(2, 2, 10.0, 10.0)});
    map.AddRoadLink(junc12, 40, 29, "right", {make_tuple(3, 1, 10.0, 10.0)});

    map.AddRoadLink(junc12, 28, 39, "left", {make_tuple(1, 1, 15.0, 15.0)});
    map.AddRoadLink(junc12, 28, 38, "right", {make_tuple(2, 2, 10.0, 10.0)});

    //Junc[13]-------------------------------------------------------------
    auto junc13 = map.AddJunction();
    map.AddRoadLink(junc13, 31, 40, "right", {make_tuple(1, 2, 15.0, 15.0)});
    map.AddRoadLink(junc13, 39, 30, "left", {make_tuple(1, 2, 15.0, 15.0)});

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