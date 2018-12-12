//
// Created by vincent on 18-10-28.
//

#pragma once
#include <memory>

namespace hdmap
{
    class Road;

    class Junction;

    class LaneSection;

    class Signal;

    class Map;

    using RoadPtr = std::shared_ptr<Road>;
    using JuncPtr = std::shared_ptr<Junction>;
    using SecPtr  = std::shared_ptr<LaneSection>;
    using SigPtr  = std::shared_ptr<Signal>;
    using MapPtr  = std::shared_ptr<Map>;
}
