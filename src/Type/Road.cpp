//
// Created by vincent on 18-10-8.
//

#include "Road.h"

using namespace hdmap;

unsigned int Road::ROAD_ID = 0;

Road::Road(Pose _start_pose)
{
    mRoadId = ROAD_ID++;
    mLength = 0;
    mPrevJid = mNextJid = -1;
    mDirection = 0;
    mStartPose = _start_pose;
}

std::vector<Pose> Road::Trajectory(int begin_lane_idx, int end_lane_idx)
{
    if(mSecPtrs.size() == 1)
    {
        return mSecPtrs.back()->GetLanePoseByIndex(end_lane_idx);
    }

    std::vector<std::vector<int>> scheme;

    auto search = std::function<void(unsigned int, int, std::vector<int>)>();

    search = [&](unsigned int curr_sec_id, int curr_lane_idx, std::vector<int> v)
    {
        if(curr_sec_id + 1 == mSecPtrs.size()-1)
        {
            for(auto & m : mSecPtrs[curr_sec_id]->mLanes[curr_lane_idx].mSuccessors)
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
            for(auto & x : mSecPtrs[curr_sec_id]->mLanes[curr_lane_idx].mSuccessors)
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
    for(int i = 0; i < mSecPtrs.size(); i++)
    {
        auto p = mSecPtrs[i]->GetLanePoseByIndex(r[i]);
        res.insert(res.end(), p.begin(), p.end());
    }
    return res;
}


Pose Road::GetStartPose(int direction)
{
    if(mSecPtrs.empty())
        return Pose();

    return  mSecPtrs.front()->mReferLine.GetStartPose();
}


Pose Road::GetEndPose(int direction)
{
    if(mSecPtrs.empty())
        return Pose();

    if(direction > 0)
        return mSecPtrs.back()->mReferLine.GetEndPose();
    else
        return mSecPtrs.front()->mReferLine.GetStartPose();
}


std::pair<unsigned int, int> Road::Locate(const Vector2d &v)
{
    double min_dist = 100000;
    int min_sec_idx = 0;

    for(int i = 0; i < mSecPtrs.size(); i++)
    {
        double t = Vector2d::SegmentDistance(mSecPtrs[i]->GetReferPose().front().GetPosition(),
                                             mSecPtrs[i]->GetReferPose().back().GetPosition(),
                                             v);
        if(t < min_dist)
        {
            min_dist = t;
            min_sec_idx = i;
        }
    }

    min_dist = 100000;
    double min_lane_idx = 0;
    for(auto x : mSecPtrs[min_sec_idx]->mLanes)
    {
        auto p = mSecPtrs[min_sec_idx]->GetLanePoseByIndex(x.first);
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
        if(curr_sec_idx + 1 == mSecPtrs.size())
        {
            scheme.emplace_back(v);
        }
        else
        {
            for(auto x : mSecPtrs[curr_sec_idx]->mLanes[curr_lane_idx].mSuccessors)
            {
                v.emplace_back(x);
                search(curr_sec_idx+1, x, v);
                v.pop_back();
            }
        }
    };

    for(auto x : mSecPtrs.front()->mLanes)
    {
        if(x.first * direction > 0)
            search(0, x.first, {x.first});
    }

    std::vector<std::vector<Pose>> res;

    for(auto & x : scheme)
    {
        std::vector<Pose> p;
        for(int i = 0; i < x.size(); ++i)
        {
            auto y = mSecPtrs[i]->GetLanePoseByIndex(x[i]);
            p.insert(p.end(), y.begin(), y.end());
        }
        res.emplace_back(p);
    }
    return res;
}


void Road::Send(Sender &sender)
{
    std::string text = "Road[" + std::to_string(mRoadId) + "]: " + std::to_string(mLength);

    auto p1 = GetStartPose(1);
    p1.Rotate(-90);
    p1.Translate(4.0, p1.GetAngle());

    auto p2 = GetEndPose(1);
    p2.Rotate(-90);
    p2.Translate(4.0, p2.GetAngle());

    auto m1 = sender.GetText(text, p1.GetPosition());
    auto m2 = sender.GetText(text, p2.GetPosition());

    sender.array.markers.emplace_back(m1);
    sender.array.markers.emplace_back(m2);
    sender.Send();

    for(auto & s : mSecPtrs) s->Send(sender);
    for(auto & s : mSigPtrs) s->Send(sender);
}

boost::property_tree::ptree Road::ToXML()
{
    pt::ptree p_road;

    p_road.add("<xmlattr>.id", mRoadId);
    p_road.add("<xmlattr>.direction", mDirection);
    p_road.add("<xmlattr>.length", mLength);
    p_road.add("<xmlattr>.prev_jid", mPrevJid);
    p_road.add("<xmlattr>.next_jid", mNextJid);

    for(auto & s : mSecPtrs) p_road.add_child("laneSection", s->ToXML());
    for(auto & s : mSigPtrs) p_road.add_child("signal", s->ToXML());

    return p_road;
}


SecPtr Road::AddSection(const Pose &_end_pose, double _ctrl_len1, double _ctrl_len2)
{
    unsigned int sec_id_ = mRoadId * 10 + mSecPtrs.size();
    double s_;
    Pose start_pose_ ;

    if(mSecPtrs.empty())
    {
        s_ = 0;
        start_pose_ = mStartPose;
    }
    else
    {
        s_ = mSecPtrs.back()->mStartS + mSecPtrs.back()->mReferLine.Length();
        start_pose_ = mSecPtrs.back()->mReferLine.GetEndPose();
    }

    auto refer_line_ = Bezier(start_pose_, _end_pose, _ctrl_len1, _ctrl_len2);
    SecPtr p(new LaneSection(sec_id_, s_, refer_line_));
    mSecPtrs.emplace_back(p);
    return p;
}

void Road::AddSignal(Vector2d v, int dir, std::string _type, std::string _info)
{
    mSigPtrs.emplace_back(new Signal(v, dir, _type, _info));
}

void Road::FromXML(const pt::ptree &p)
{
    mRoadId = p.get<int>("<xmlattr>.id");
    mLength = p.get<double>("<xmlattr>.length");
    mPrevJid = p.get<int>("<xmlattr>.prev_jid");
    mNextJid = p.get<int>("<xmlattr>.next_jid");

    for(auto s : p.get_child(""))
    {
        if(s.first == "laneSection")
        {
            SecPtr p_section(new LaneSection());
            p_section->FromXML(s.second);
            mSecPtrs.emplace_back(p_section);
        }

        if(s.first == "signal")
        {
            SigPtr p_signal(new Signal());
            p_signal->FromXML(s.second);
            mSigPtrs.emplace_back(p_signal);
        }
    }
}

double Road::Distance(const Vector2d & v)
{
    double min_dist = 100000;
    for(auto & s : mSecPtrs)
    {
        double t = s->Distance(v);
        if(t < min_dist)
            min_dist = t;
    }
    return min_dist;
}
