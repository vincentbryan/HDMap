//
// Created by vincent on 18-10-8.
//

#pragma once

#include <vector>
#include <memory>
#include "../Math/Curve.h"
#include "../Math/CubicFunction.h"
namespace hdmap
{
class Lane
{
public:
    enum LANE_TYPE
    {
        NONE, Driving, Stop, Shoulder, Biking, Sidewalk, Border,
        Restricted, Parking, Bidirectional,Median, Special1,
        Special2, Special3, RoadWorks, Tram, Rail, Entry, Exit,
        OffRamp, OnRamp
    };
    static double DEFAULT_WIDTH;
    //TODO lane:level
    //int level;

    CubicFunction width;
    unsigned land_id;
    LANE_TYPE type;

    std::vector<int> predecessors;
    std::vector<int> successors;

public:
    explicit Lane(int _lane_id = 0, CubicFunction _width = CubicFunction(), LANE_TYPE _type = LANE_TYPE ::Driving);
    ~Lane();

    void AddPredecessor(int idx)
    {
        predecessors.emplace_back(idx);
    }

    void AddSuccessors(int idx)
    {
        successors.emplace_back(idx);
    }
};


}

