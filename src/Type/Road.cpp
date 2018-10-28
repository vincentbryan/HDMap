//
// Created by vincent on 18-10-8.
//

#include "Road.h"

using namespace hdmap;

unsigned int Road::ROAD_ID = 0;

Road::Road() : mForwardRoad(1), mBackwardRoad(-1)
{
    iRoadId = ROAD_ID++;
    dLength = 0;
    iPrevJid = iNextJid = -1;
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


Pose Road::GetStartPose(int direction)
{
    if(mSections.empty())
        return Pose();

    if(direction > 0)
        return mSections.front().mReferLine.GetStartPose();
    else
        return mSections.back().mReferLine.GetEndPose();
}


Pose Road::GetEndPose(int direction)
{
    if(mSections.empty())
        return Pose();

    if(direction > 0)
        return mSections.back().mReferLine.GetEndPose();
    else
        return mSections.front().mReferLine.GetStartPose();
}


std::pair<unsigned int, int> Road::Locate(const Vector2d &v)
{
    double min_dist = 100000;
    int min_sec_idx = 0;

    for(int i = 0; i < mSections.size(); i++)
    {
        double t = Vector2d::SegmentDistance(mSections[i].GetReferPose().front().GetPosition(),
                                             mSections[i].GetReferPose().back().GetPosition(),
                                             v);
        if(t < min_dist)
        {
            min_dist = t;
            min_sec_idx = i;
        }
    }

    min_dist = 100000;
    double min_lane_idx = 0;
    for(auto x : mSections[min_sec_idx].mLanes)
    {
        auto p = mSections[min_sec_idx].GetLanePoseByIndex(x.first);
        double t = Vector2d::SegmentDistance(p.front().GetPosition(),
                                             p.back().GetPosition(),
                                             v);
        if(t < min_dist)
        {
            min_dist = t;
            min_lane_idx = x.first;
        }
    }

    return {min_sec_idx, min_lane_idx};
}


std::vector<std::vector<Pose>> Road::GetLanePosesByDirection(int direction)
{
    std::vector<std::vector<int>> scheme;

    auto search = std::function<void(unsigned int, int, std::vector<int>)>();
    search = [&](unsigned int curr_sec_idx, int curr_lane_idx, std::vector<int> v)
    {
        if(curr_sec_idx + 1 == mSections.size())
        {
            scheme.emplace_back(v);
        }
        else
        {
            for(auto x : mSections[curr_sec_idx].mLanes[curr_lane_idx].successors)
            {
                v.emplace_back(x);
                search(curr_sec_idx+1, x, v);
                v.pop_back();
            }
        }
    };

    for(auto x : mSections.front().mLanes)
    {
        if(x.first * direction > 0)
            search(0, x.first, {x.first});
    }

    std::vector<std::vector<Pose>> res;

    for(auto & x : scheme)
    {
        std::vector<Pose> p;
        if(x.front() > 0)
        {
            for(int i = 0; i < x.size(); ++i)
            {
                auto y = mSections[i].GetLanePoseByIndex(x[i]);
                p.insert(p.end(), y.begin(), y.end());
            }
        }
        else
        {
            for(int i = x.size()-1; i >= 0; i--)
            {
                auto y = mSections[i].GetLanePoseByIndex(x[i]);
                p.insert(p.end(), y.begin(), y.end());
            }
        }
        res.emplace_back(p);
    }

    return res;
}


void Road::Send(Sender &sender)
{
    std::string text = "Road[" + std::to_string(iRoadId) + "]: " + std::to_string(dLength);
    auto m = sender.GetText(text, GetStartPose(1).GetPosition());
    sender.array.markers.emplace_back(m);
    sender.Send();

    for(auto & s : mSections) s.Send(sender);
    for(auto & s : mSignals) s.Send(sender);
//    mForwardRoad.Send(sender);
//    mBackwardRoad.Send(sender);

}


std::shared_ptr<SubRoad> Road::GetSubRoadPtr(int dir)
{
    if(dir > 0)
        return std::make_shared<SubRoad>(mForwardRoad);
    else
        return std::make_shared<SubRoad>(mBackwardRoad);
}

void Road::InitSubRoad()
{
    mForwardRoad.Init(std::shared_ptr<Road>(this));
    mBackwardRoad.Init(std::shared_ptr<Road>(this));
}


void SubRoad::Send(hdmap::Sender &sender)
{
    auto ps = GetLanePose();
    for(auto const & p : ps)
    {
        sender.SendPoses(p, 0.7, 0.7, 0.7, 0.8, 0.0, 0.5);
    }

    for(auto & s : mSubRoadSignals)
    {
        s.Send(sender);
    }
}

Pose SubRoad::GetStartPose()
{
    if(direction > 0)
        return pBaseRoad->mSections.front().mReferLine.GetStartPose();
    else
        return pBaseRoad->mSections.back().mReferLine.GetEndPose();
}

std::vector<std::vector<Pose>> SubRoad::GetLanePose()
{
    return pBaseRoad->GetLanePosesByDirection(direction);
}

void SubRoad::Init(std::shared_ptr<Road> p_road)
{
    pBaseRoad = p_road;
    iRoadId = pBaseRoad->iRoadId;
    iPrevJid = direction > 0 ? pBaseRoad->GetPrevJid() : pBaseRoad->GetNextJid();
    iNextJid = direction > 0 ? pBaseRoad->GetNextJid() : pBaseRoad->GetPrevJid();

    for(auto & s : pBaseRoad->mSignals)
    {
        if(s.direction == direction)
        {
            this->mSubRoadSignals.emplace_back(s);
        }
    }

    for(auto & s : pBaseRoad->mSections)
    {
        this->mSubRoadSection.emplace_back(s.GetSubSection(direction));
    }

    if(direction < 0)
    {
        for(int i = 1; i < mSubRoadSection.size(); ++i)
        {
            mSubRoadSection[i].s = mSubRoadSection[i-1].s + mSubRoadSection[i-1].mReferLine.Length();
        }
    }
}

SubRoad::SubRoad(int direction) : direction(direction) {};
