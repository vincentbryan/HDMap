//
// Created by vincent on 18-10-8.
//

#ifndef HDMAP_LANE_H
#define HDMAP_LANE_H

#include <vector>
#include "CubicPoly.h"
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

    int land_id;
    int level;
    LANE_TYPE type;

    std::vector<std::pair<int, int>>links;
    CubicPoly width;

public:
    explicit Lane(int _lane_id, int _level, LANE_TYPE _type = LANE_TYPE ::Driving, CubicPoly _width = CubicPoly());
    void AddLink(std::pair<int, int> _link);
};
}


#endif //HDMAP_LANE_H
