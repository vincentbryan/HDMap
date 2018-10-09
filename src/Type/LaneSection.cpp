//
// Created by vincent on 18-10-8.
//

#include "LaneSection.h"

using namespace hdmap;

LaneSection::LaneSection(double _s,
                         bool _isSingleSide,
                         std::vector<Lane> _left,
                         std::vector<Lane> _center,
                         std::vector<Lane> _right):
    s(_s), isSingleSide(_isSingleSide),
    left(_left), center(_center), right(_right)
{}