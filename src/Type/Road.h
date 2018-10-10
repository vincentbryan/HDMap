//
// Created by vincent on 18-10-8.
//

#ifndef HDMAP_ROAD_H
#define HDMAP_ROAD_H


#include "LaneSection.h"
namespace hdmap
{
class Road
{
public:
    static unsigned int ROAD_ID;
    unsigned int mRoadId;
    double s;

    std::vector<LaneSection> mSections;

    explicit Road();

    void AddSection(const LaneSection & s)
    {
        mSections.emplace_back(s);
    }
};
}

#endif //HDMAP_ROAD_H
