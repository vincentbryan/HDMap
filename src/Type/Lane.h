//
// Created by vincent on 18-10-8.
//

#pragma once

#include <vector>
#include <memory>
#include "../Math/Curve.h"
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

    std::shared_ptr<Curve> pWidth;
    int land_id;
    LANE_TYPE type;

    std::vector<unsigned int> predecessors;
    std::vector<unsigned int> successors;

public:
    explicit Lane(int _lane_id = 0, std::shared_ptr<Curve> p_width = nullptr, LANE_TYPE _type = LANE_TYPE ::Driving);
    ~Lane();

    void AddPredecessor(unsigned int n)
    {
        predecessors.emplace_back(n);
    }

    void AddSuccessors(unsigned int n)
    {
        successors.emplace_back(n);
    }
};


}

