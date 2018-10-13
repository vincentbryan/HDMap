//
// Created by vincent on 18-10-9.
//

#include "HDMap.h"

using namespace hdmap;

HDMap::HDMap() : mStartPose(Pose()),
                 mEndPose(Pose()),
                 mPrevSection(LaneSection()),
                 mCurrSection(LaneSection())
{}

void HDMap::AddRoad()
{
    mRoads.emplace_back(Road());
}

void HDMap::StartSection(std::vector<std::pair<int, bool>> new_lane, std::vector<std::pair<int, int>>links)
{
    assert(mRoads.size() >= 1);
    auto section_id_next = CalcuSectionId(mRoads.size()-1, mRoads.back().mSections.size());
    mCurrSection.iSectionId = section_id_next;
    mCurrSection.s = mPrevSection.s + mPrevSection.pReferLine->Length();
    mCurrSection.mLanes.clear();

    vTempLane = new_lane;
    vTempLink = links;
}

void HDMap::EndSection(Pose p)
{
    //Update Pose
    mEndPose = mStartPose;
    mStartPose = p;

    mCurrSection.pReferLine.reset(new Line(mRoads.back().s, mEndPose, mStartPose));
    mCurrSection.Clear();

    for(auto x : vTempLane)
    {
        auto idx = std::get<0>(x);
        auto b = std::get<1>(x);
        auto id = CalcuLaneId(mCurrSection.iSectionId, idx);
        mCurrSection.AddLane(idx, id, b);
    }

    for(auto x : vTempLink)
    {
        int a = std::get<0>(x);
        int b = std::get<1>(x);
        mPrevSection.mLanes[a].AddPredecessor(b);
        mCurrSection.mLanes[b].AddSuccessors(a);
    }

    mRoads.back().mSections.emplace_back(mCurrSection);

    mPrevSection = mCurrSection;
}

unsigned int HDMap::CalcuSectionId(unsigned int road, unsigned int section)
{
    return road * 10 + section;
}

unsigned int HDMap::CalcuLaneId(unsigned int section, int lane)
{
    return section*10 + 5 + lane;
}

void HDMap::AddJunction()
{
    mJunctions.emplace_back(Junction());
}

void HDMap::AddConnection(unsigned int from_road, int from_lane_idx,
                          unsigned int to_road, int to_lane_idx)
{
    auto from_section = mRoads[from_road].mSections.back();
    auto to_section = mRoads[to_road].mSections.front();

    mJunctions.back().AddConnection(
         from_section.GetLaneByIndex(from_lane_idx),
         from_section.GetLanePoseByIndex(from_lane_idx).back(),
         to_section.GetLaneByIndex(to_lane_idx),
         to_section.GetLanePoseByIndex(to_lane_idx).front()
    );
}