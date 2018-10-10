//
// Created by vincent on 18-10-8.
//

#include "Road.h"

using namespace hdmap;

unsigned int Road::ROAD_ID = 0;

Road::Road()
{
    ROAD_ID++;
    mRoadId = ROAD_ID;
}
