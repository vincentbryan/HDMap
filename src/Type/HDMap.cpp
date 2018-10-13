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
    auto section_id_next = CalcuSectionId(mRoads.size(), mRoads.back().mSections.size()+1);
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
