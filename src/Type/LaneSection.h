//
// Created by vincent on 18-10-8.
//

#ifndef HDMAP_LANESECTION_H
#define HDMAP_LANESECTION_H

#include "Lane.h"
namespace hdmap
{
class LaneSection
{
public:
    double s;
    bool isSingleSide;

    std::vector<Lane> left;
    std::vector<Lane> center;
    std::vector<Lane> right;

    LaneSection(double _s, bool _isSingleSide, std::vector<Lane> _left, std::vector<Lane> _center, std::vector<Lane> _right);
};
}




#endif //HDMAP_LANESECTION_H
