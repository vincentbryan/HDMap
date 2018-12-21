//
// Created by vincent on 18-10-8.
//

#include <random>
#include "Type/LaneSection.h"

using namespace hdmap;

LaneSection::LaneSection(unsigned int _section_id, double _s,
                         Bezier _refer_line,
                         CubicFunction _lane_offset)
{
    mSectionId = _section_id;
    mStartS = _s;
    mReferLine = _refer_line;
    mLaneOffset = _lane_offset;
}

void LaneSection::AddLane(int _lane_idx, double _start_width, double _end_width, std::vector<int> _pred, std::vector<int> _succ)
{
    if(_lane_idx > 0) mRightBoundary = std::max(mRightBoundary, _lane_idx);
    if(_lane_idx < 0) mLeftBoundary = std::min(mLeftBoundary, _lane_idx);

    int lane_id = mSectionId * 100 + _lane_idx;
    CubicFunction width(_start_width, mReferLine.Length(), _end_width);
    Lane lane(lane_id, width);
    lane.mPredecessors = std::move(_pred);
    lane.mSuccessors = std::move(_succ);

    mLanes.insert(std::pair<int, Lane>(_lane_idx, lane));
}

std::vector<Pose> LaneSection::GetReferPose()
{
    if(mAllLanePose.empty())
        GenerateAllPose(0.5);
    return mAllLanePose[0];
}

void LaneSection::GenerateAllPose(double ds)
{
    for(auto x : mLanes)
        mAllLanePose[x.first] = std::vector<Pose>();

    double _s = 0;
    double len = mReferLine.Length();

    while (true)
    {
        AppendPose(_s);
        _s += ds;
        if(_s >= len)
        {
            AppendPose(len);
            break;
        }
    }
}

void LaneSection::AppendPose(double s_)
{
    Pose refer_pose = mReferLine.GetPose(s_);
    mAllLanePose[0].emplace_back(refer_pose);
    double width = 0;
    int idx = 0;
    for(idx = 1; idx <= mRightBoundary; idx++)
    {

        Angle angle = refer_pose.GetAngle();
        angle.Rotate(-90.0);

        double w = mLanes[idx].mOffset.Value(s_);
        Pose t = refer_pose.GetTranslation(w, angle);
        mAllLanePose[idx].emplace_back(t);
    }
}

std::map<int, std::vector<Pose>> LaneSection::GetAllPose()
{
    if(mAllLanePose.empty())
        GenerateAllPose(0.5);
    return mAllLanePose;
}

void LaneSection::Send(Sender &sender)
{
    if(mAllLanePose.empty()) GenerateAllPose(1.0);

    for(auto & x : mAllLanePose)
    {
        if(x.first == 0)
        {
            auto refer_line = sender.GetLineStrip(x.second, 199.0/255, 166.0/255, 33.0/255, 1.0);
            sender.array.markers.push_back(refer_line);
        }
        else
        {
            std::mt19937 rg = std::mt19937(std::random_device()());
            double c = (rg()%255)/255.0;

            auto solid_line = sender.GetLineStrip(x.second, c, 0.7, 0.7, 1.0);
            sender.array.markers.push_back(solid_line);
        }
        auto lane_idx = sender.GetText(std::to_string(x.first), x.second[x.second.size()/2].GetPosition());
        sender.array.markers.push_back(lane_idx);
    }
    sender.Send();
}

boost::property_tree::ptree LaneSection::ToXML()
{
    pt::ptree p_sec;
    p_sec.add("<xmlattr>.id", mSectionId);
    p_sec.add("<xmlattr>.s", mStartS);
    p_sec.add("<xmlattr>.left_idx", mLeftBoundary);
    p_sec.add("<xmlattr>.right_idx", mRightBoundary);

    for(auto & p : mReferLine.GetParam())
        p_sec.add("referenceLine.param", p);

    for(auto & m : mLanes)
        p_sec.add_child("lane", m.second.ToXML());

    return p_sec;
}

std::vector<Pose> LaneSection::GetLanePoseByIndex(int _index)
{
    if(mAllLanePose.empty())
        GetAllPose();
    if (_index < 0) {
        _index = mRightBoundary + 1 + _index;
    }
    assert(_index >= 0 && _index <= mRightBoundary);

    return mAllLanePose[_index];
}


void LaneSection::FromXML(const pt::ptree &p)
{
    for(auto & sec_child : p.get_child(""))
    {
        if(sec_child.first == "<xmlattr>")
        {
            mSectionId = sec_child.second.get<int>("id");
            mStartS = sec_child.second.get<double>("s");
            mLeftBoundary = sec_child.second.get<int>("left_idx");
            mRightBoundary = sec_child.second.get<int>("right_idx");
        }

        if(sec_child.first == "referenceLine")
        {
            std::vector<double> res;
            for(auto & t : sec_child.second.get_child(""))
            {
                if(t.first == "param")
                    res.emplace_back(std::atof(t.second.data().c_str()));
            }
            assert(res.size() == 8);
            mReferLine = Bezier(res);
        }

        if(sec_child.first == "lane")
        {
            Lane lane;
            lane.FromXML(sec_child.second);
            mLanes.insert({lane.mLaneIndex, lane});
        }
    }
}


bool LaneSection::Cover(const Coor &v)
{
    if(mAllLanePose.empty()) GenerateAllPose(0.5);

    std::vector<Pose> vec;

    for(auto & x : mReferLine.GetPoses(0.5))
    {
        vec.push_back(x);
    }

    if(mRightBoundary > 0)
    {
        auto ps = mAllLanePose[mRightBoundary];
        std::reverse(ps.begin(), ps.end());
        for(auto & x : ps) vec.push_back(x);
    }

    return IGeometry::Cover(vec, {v});

}


