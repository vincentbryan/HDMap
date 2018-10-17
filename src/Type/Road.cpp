//
// Created by vincent on 18-10-8.
//

#include "Road.h"

using namespace hdmap;

unsigned int Road::ROAD_ID = 0;

Road::Road()
{
    iRoadId = ROAD_ID++;
    dLength = 0;
}

std::pair<int, double> Road::Distance(const Vector2d &v)
{
    double min_distance = 1000000;
    double min_sec_idx = 0;
    for(auto & x : mSections)
    {
        auto t = x.Distance(v);
        if(min_distance > t)
        {
            min_distance = t;
            min_sec_idx = x.iSectionId;
        }
    }
    return {min_sec_idx, min_distance};
}