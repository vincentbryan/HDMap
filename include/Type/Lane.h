#pragma once

#include <vector>
#include <memory>
#include "Math/Curve.h"
#include "Math/CubicFunction.h"
#include "Interface/IXML.h"

namespace hdmap
{
class Lane : public IXML
{
public:
    enum LANE_TYPE { NONE, Driving };

    CubicFunction mOffset;
    unsigned mLandId;
    int mLaneIndex;
    LANE_TYPE mType;

    std::vector<int> mPredecessors;
    std::vector<int> mSuccessors;

public:
    explicit Lane(int _lane_id = 0, CubicFunction _offset = CubicFunction(), LANE_TYPE _type = LANE_TYPE ::Driving);
    ~Lane();

    void AddPredecessor(int idx);
    void AddSuccessors(int idx);

    boost::property_tree::ptree ToXML() override;
    void FromXML(const pt::ptree &p) override;
};
}

