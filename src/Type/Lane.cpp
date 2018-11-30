//
// Created by vincent on 18-10-8.
//

#include "Type/Lane.h"

using namespace hdmap;

Lane::Lane(int _lane_id, CubicFunction _offset, LANE_TYPE _type)
{
    mLandId = _lane_id;
    mLaneIndex = int(mLandId) % 10;
    mOffset = _offset;
    mType = _type;
}

Lane::~Lane() {}

void Lane::AddPredecessor(int idx)
{
    mPredecessors.emplace_back(idx);
}

void Lane::AddSuccessors(int idx)
{
    mSuccessors.emplace_back(idx);
}

boost::property_tree::ptree Lane::ToXML()
{
    pt::ptree p_lane;
    p_lane.add("<xmlattr>.id", mLandId);
    p_lane.add("<xmlattr>.idx", mLaneIndex);
    p_lane.add("offset.a", mOffset.a);
    p_lane.add("offset.b", mOffset.b);
    p_lane.add("offset.c", mOffset.c);
    p_lane.add("offset.d", mOffset.d);
    p_lane.add("offset.range", mOffset.range);

    for(auto & x : mPredecessors)
        p_lane.add("predecessors", x);

    for(auto & x : mSuccessors)
        p_lane.add("successors", x);

    return p_lane;
}

void Lane::FromXML(const pt::ptree &p)
{
    mLandId = p.get<int>("<xmlattr>.id");
    mLaneIndex = p.get<int>("<xmlattr>.idx");

    auto a = p.get<double>("offset.a");
    auto b = p.get<double>("offset.b");
    auto c = p.get<double>("offset.c");
    auto d = p.get<double>("offset.d");
    auto range = p.get<double>("offset.range");
    mOffset = CubicFunction(a, b, c, d, range);

    for(auto k : p.get_child(""))
    {
        if(k.first == "predecessors")
        {
            int n = atoi(k.second.data().c_str());
            mPredecessors.emplace_back(n);
        }
        if(k.first == "successors")
        {
            int n = atoi(k.second.data().c_str());
            mSuccessors.emplace_back(n);
        }
    }

}
