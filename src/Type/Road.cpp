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
std::vector<Pose> Road::Trajectory(int begin_lane_idx, int end_lane_idx)
{
    if(mSections.size() == 1)
    {
        return mSections.back().GetLanePoseByIndex(end_lane_idx);
    }

    std::vector<std::vector<int>> scheme;

    auto search = std::function<void(unsigned int, int, std::vector<int>)>();

    search = [&](unsigned int curr_sec_id, int curr_lane_idx, std::vector<int> v)
    {
        if(curr_sec_id + 1 == mSections.size()-1)
        {
            for(auto & m : mSections[curr_sec_id].mLanes[curr_lane_idx].successors)
            {
                if(m == end_lane_idx)
                {
                    v.emplace_back(curr_lane_idx);
                    v.emplace_back(end_lane_idx);
                    scheme.emplace_back(v);
                }
            }
        }
        else
        {
            for(auto & x : mSections[curr_sec_id].mLanes[curr_lane_idx].successors)
            {
                v.emplace_back(curr_lane_idx);
                search(curr_sec_id+1, x, v);
                v.pop_back();
            }
        }
    };

    std::vector<int> v;
    search(0, begin_lane_idx, v);

    for(auto & x : scheme)
    {
        for(auto & y : x)
            std::cout << y << " ";
        std::cout << std::endl;
    }

    //TODO
    auto r = scheme.front();

    std::vector<Pose> res;
    for(int i = 0; i < mSections.size(); i++)
    {
        auto p = mSections[i].GetLanePoseByIndex(r[i]);
        res.insert(res.end(), p.begin(), p.end());
    }
    return res;
}
