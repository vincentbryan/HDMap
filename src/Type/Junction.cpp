//
// Created by vincent on 18-10-13.
//

#include "Junction.h"

using namespace hdmap;

unsigned int Junction::JUNCTION_ID = 0;

Junction::Junction()
{
    mJunctionId = JUNCTION_ID++;
}

void Junction::AddConnection(unsigned int _from_road_id, int _from_lane_idx, Pose _from_lane_pose,
                             unsigned int _to_road_id, int _to_lane_idx, Pose _to_lane_pose,
                             double _ctrl_len1, double _ctrl_len2)
{
    std::pair<unsigned int, unsigned int> p(_from_road_id, _to_road_id);
    if(mRoadLinks.find(p) != mRoadLinks.end())
    {
        mRoadLinks[p].AddLaneLink(_from_lane_idx,
                                  _to_lane_idx,
                                  Bezier(_from_lane_pose, _to_lane_pose, _ctrl_len1, _ctrl_len2));
    }
    else
    {
        RoadLink road_link(_from_road_id, _to_road_id);
        road_link.AddLaneLink(_from_lane_idx,
                              _to_lane_idx,
                              Bezier(_from_lane_pose, _to_lane_pose, _ctrl_len1, _ctrl_len2));
        mRoadLinks[p] = road_link;
    }
}

std::vector<std::vector<Pose>> Junction::GetAllPose()
{
    if(mPoses.empty())
        _GenerateAllPose();

    return mPoses;
}

bool Junction::Check(std::pair<unsigned int, unsigned int> links)
{
    return mRoadLinks.find(links) != mRoadLinks.end();
}


std::pair<int, int> Junction::GetLink(std::pair<unsigned int, unsigned int> RoadPair)
{
    auto it = mRoadLinks.find(RoadPair);
    return  it->first;
}


std::vector<Pose> Junction::GetPose(unsigned int from_road_id,
                                    int from_lane_idx,
                                    unsigned int to_road_id,
                                    int to_lane_idx)
{
    return mRoadLinks[{from_road_id, to_road_id}].GetPose(from_lane_idx, to_lane_idx);
}

void Junction::Send(Sender &sender)
{
    if(mVertices.empty()) GenerateVertices();

    std::vector<Pose> ps;
    for(auto & x : mVertices)
    {
        ps.emplace_back(x, Angle(0));
    }
    ps.emplace_back(mVertices.front(), Angle(0));

    auto m = sender.GetLineStrip(ps, 76.0/255.0, 180.0/255.0, 231.0/255.0, 1.0, 0, 0.15);
    sender.array.markers.emplace_back(m);
    sender.Send();

    for(auto & x : mRoadLinks)
    {
        x.second.Send(sender);
    }
}

void Junction::_GenerateAllPose()
{
    for(auto x : mRoadLinks)
    {
        for(auto y : x.second.mLaneLinks)
        {
            mPoses.emplace_back(y.mReferLine.GetAllPose(0.1));
        }
    }
}

RoadLink Junction::GetRoadLink (int rid1, int rid2)
{
    auto it = mRoadLinks.find(std::pair<unsigned int, unsigned int>(rid1, rid2));
    return it->second;
}

boost::property_tree::ptree Junction::ToXML()
{
    pt::ptree p_junc;
    p_junc.add("<xmlattr>.id", mJunctionId);

    if(mVertices.empty())GenerateVertices();
    pt::ptree p_vec;
    pt::ptree p_v;
    for(auto & v : mVertices)
    {
        p_v.add("<xmlattr>.x", v.x);
        p_v.add("<xmlattr>.y", v.y);
        p_vec.add_child("vertex", p_v);
        p_v.clear();
    }
    p_junc.add_child("vertice", p_vec);

    for(auto & r : mRoadLinks)
    {
        p_junc.add_child("roadLink", r.second.ToXML());
    }

    return p_junc;
}

void Junction::FromXML(const pt::ptree &p)
{
    for(auto & n : p.get_child(""))
    {
        if(n.first == "<xmlattr>")
        {
            mJunctionId = n.second.get<int>("id");
        }

        if(n.first == "roadLink")
        {
            RoadLink r;
            r.FromXML(n.second);
            mRoadLinks[{r.mFromRoadId, r.mToRoadId}] = r;
        }

        if(n.first == "vertice")
        {
            pt::ptree p_vec = n.second;

            for(auto & t : p_vec.get_child(""))
            {
                if(t.first == "vertex")
                {
                    auto x = t.second.get<double>("<xmlattr>.x");
                    auto y = t.second.get<double>("<xmlattr>.y");
                    mVertices.emplace_back(x, y);
                }
            }
        }
    }
}

double Junction::Distance(const Vector2d &v)
{
    double min_dist = 1000000;
    for(auto & ps : GetAllPose())
    {
        for(auto & p : ps)
        {
            double t = sqrt((p.x - v.x)*(p.x - v.x) + (p.y - v.y)*(p.y - v.y));
            if(t < min_dist)
                min_dist = t;
        }
    }
    return min_dist;
}

void Junction::GenerateVertices()
{
    std::map<unsigned int, int> road_lane;  /// 每一条道路最右侧的lane_idx
    std::map<unsigned int, Pose> road_pose; /// 每一条道路最右侧的Pose

    /// 获取每一条道路最右侧的Pose
    for(auto & x : mRoadLinks)
    {
        auto from_rid = x.second.mFromRoadId;
        auto to_rid = x.second.mToRoadId;

        for(auto & y : x.second.mLaneLinks)
        {
            if(road_lane.find(from_rid) == road_lane.end() or
                road_lane[from_rid] < y.mFromLaneIndex)
            {
                road_lane[from_rid] = y.mFromLaneIndex;
                road_pose[from_rid] = y.mReferLine.GetStartPose();
            }

            if(road_lane.find(to_rid) == road_lane.end() or
                road_lane[to_rid] < y.mToLaneIndex)
            {
                road_lane[to_rid] = y.mToLaneIndex;
                road_pose[to_rid] = y.mReferLine.GetEndPose();
            }
        }
    }

    std::vector<Vector2d> vec;
    for(auto & x : road_pose) ///将最右侧的Pose再向右移动3.0米
    {
        auto p = x.second;
        p.Rotate(-90.0);
        p.Translate(3.0, p.GetAngle());
        vec.emplace_back(p.GetPosition());
    }


    auto test = [&vec, this](const Vector2d & s, const Vector2d & e) -> bool
    {
        for(auto & t : mVertices)
            if(t == e) return false;

        for(auto & t : vec)
        {
            if(t != s && t != e)
            {
                ///使用叉乘
                double m = (s.x-t.x)*(e.y-t.y) - (e.x-t.x)*(s.y-t.y);

                ///若是在同一直线,则判断t是否在s和e中间
                if(fabs(m) < 0.01)
                {
                    double c1 = (t.x - s.x) * (t.x - e.x);
                    double c2 = (t.y - s.y) * (t.y - e.y);
                    if(c1 < 0 and c2 < 0)
                        return false;
                }
                else if(m < 0)
                    return false;
            }
        }
        return true;
    };

    mVertices.emplace_back(vec.front());

    for(int cnt = 1; cnt < vec.size(); ++cnt)
    {
        for(auto x : vec)
        {
            if(test(mVertices.back(), x))
            {
                mVertices.emplace_back(x);
                break;
            }
        }
    }
}

bool Junction::Cover(const Vector2d &v)
{
    return IGeometry::Cover(mVertices, v);
}