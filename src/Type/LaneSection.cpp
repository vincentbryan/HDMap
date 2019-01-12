#include <random>
#include <Type/LaneSection.h>

#include "Type/LaneSection.h"

using namespace hdmap;

LaneSection:: LaneSection(unsigned int _section_id, double _s,
                         Bezier _refer_line)
{
    ID = _section_id;
    mStartS = _s;
    mReferLine = _refer_line;
}

void LaneSection::AddLane(int _lane_idx, double _start_width, double _end_width, std::vector<int> _pred, std::vector<int> _succ)
{
    if(_lane_idx > 0) mRightBoundary = std::max(mRightBoundary, _lane_idx);
    if(_lane_idx < 0) mLeftBoundary = std::min(mLeftBoundary, _lane_idx);

    int lane_id = ID * 100 + _lane_idx;
    CubicFunction width(_start_width, mReferLine.Length(), _end_width);
    Lane lane(lane_id, width);
    lane.mPredecessors = std::move(_pred);
    lane.mSuccessors = std::move(_succ);

    mLanes.insert(std::pair<int, Lane>(_lane_idx, lane));
}

std::vector<Pose> LaneSection::GetReferenceLinePose()
{

    GetRegionPoses();

    return mAllLanePose[0];
}

void LaneSection::AppendPose(double s_)
{
    Pose refer_pose = mReferLine.GetPose(s_);
    mAllLanePose[0].emplace_back(refer_pose);

    for(int idx = 1; idx <= mRightBoundary; idx++)
    {

        Angle angle = refer_pose.GetAngle();
        angle.Rotate(-90.0);

        double w = mLanes[idx].mOffset.Value(s_);
        Pose t = refer_pose.GetTranslation(w, angle);
        mAllLanePose[idx].emplace_back(t);
    }
}

void LaneSection::OnSend(Sender &sender)
{
    if(mAllLanePose.empty()) GenerateRegionPoses();

    for(auto & x : mAllLanePose)
    {
        if(x.first == 0)
        {
            auto refer_line = sender.GetLineStrip(x.second, 199.0/255, 166.0/255, 33.0/255, 1.0);
            sender.array.markers.emplace_back(refer_line);
        }
        else
        {
            std::mt19937 rg = std::mt19937(std::random_device()());
            double c = (rg()%255)/255.0;

            auto solid_line = sender.GetLineStrip(x.second, c, 0.7, 0.7, 1.0);
            sender.array.markers.emplace_back(solid_line);
        }
        auto lane_idx = sender.GetText(std::to_string(x.first), x.second[x.second.size()/2].GetPosition());
        sender.array.markers.emplace_back(lane_idx);
    }

    // sender.Send();
}

boost::property_tree::ptree LaneSection::ToXML()
{
    pt::ptree p_sec;
    p_sec.add("<xmlattr>.id", ID);
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
        GenerateRegionPoses();

    if ( _index > mRightBoundary || _index < -mRightBoundary)
    {
        throw std::out_of_range("out of range");
    }

    // support selecting by negative index.
    if (_index < 0) {
        _index = mRightBoundary + 1 + _index;
    }

    return mAllLanePose[_index];
}


void LaneSection::FromXML(const pt::ptree &p)
{
    for(auto & sec_child : p.get_child(""))
    {
        if(sec_child.first == "<xmlattr>")
        {
            ID = sec_child.second.get<int>("id");
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

void LaneSection::GenerateRegionPoses()
{
    /// In lane section, all pose, include every lanes, will be generate
    /// But only reference line and the right most lane's pose will

    const double ds = 1.0;
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

    // generate region pose for cover checking and distance searching.
    if (mRightBoundary <= 0)
    {
        throw std::runtime_error("There is not any lanes added to this lane section yet.");
    }

    mRegionPoses.clear();
    for (auto & _pose : mReferLine.GetPoses(1.0))
    {
        mRegionPoses.emplace_back(_pose);
    }

    for (auto it = mAllLanePose[mRightBoundary].rbegin();
         it != mAllLanePose[mRightBoundary].rend(); ++it)
    {
        mRegionPoses.emplace_back(*it);
    }

}

