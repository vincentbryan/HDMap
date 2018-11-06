//
// Created by vincent on 18-10-9.
//

#include <boost/property_tree/ptree.hpp>
#include <algorithm>
#include <LocalMapRequest.h>
#include "Map.h"
#include "common.h"

using namespace hdmap;
namespace pt = boost::property_tree;

Map::Map()
{}

void Map::SetSender(std::shared_ptr<Sender> _sender)
{
    pSender = _sender;
}


RoadPtr Map::AddRoad(const Pose &_start_pose)
{
    RoadPtr p(new Road(_start_pose));
    mRoadPtrs.emplace_back(p);
    return p;
}

JuncPtr Map::AddJunction()
{
    JuncPtr p(new Junction());
    mJuncPtrs.emplace_back(p);
    return p;
}

void Map::Send()
{
    for(auto & m : mRoadPtrs) m->Send(*pSender);
    for(auto & j : mJuncPtrs) j->Send(*pSender);
}


void Map::AddConnection(JuncPtr p, unsigned int from_road, int from_lane_idx,
                        unsigned int to_road, int to_lane_idx,
                        double _ctrl_len1, double _ctrl_len2)
{
    Pose start_pose, end_pose;

    if(from_lane_idx > 0)
    {
        auto p1 = mRoadPtrs[from_road]->mSecPtrs.back()->GetLanePoseByIndex(from_lane_idx).back();
        auto p2 = mRoadPtrs[from_road]->mSecPtrs.back()->GetLanePoseByIndex(from_lane_idx-1).back();
        start_pose = {0.5 * (p1.GetPosition() + p2.GetPosition()), p1.GetAngle()};
    }
    else
    {
        auto p1 = mRoadPtrs[from_road]->mSecPtrs.front()->GetLanePoseByIndex(from_lane_idx).front();
        auto p2 = mRoadPtrs[from_road]->mSecPtrs.front()->GetLanePoseByIndex(from_lane_idx+1).front();
        start_pose = {0.5 * (p1.GetPosition() + p2.GetPosition()), p1.GetAngle()};
        start_pose.Rotate(180);
    }


    if(to_lane_idx >  0)
    {
        auto p1 = mRoadPtrs[to_road]->mSecPtrs.front()->GetLanePoseByIndex(to_lane_idx).front();
        auto p2 = mRoadPtrs[to_road]->mSecPtrs.front()->GetLanePoseByIndex(to_lane_idx-1).front();
        end_pose = {0.5 * (p1.GetPosition() + p2.GetPosition()), p1.GetAngle()};
    }
    else
    {
        auto p1 = mRoadPtrs[to_road]->mSecPtrs.back()->GetLanePoseByIndex(to_lane_idx).back();
        auto p2 = mRoadPtrs[to_road]->mSecPtrs.back()->GetLanePoseByIndex(to_lane_idx+1).back();
        end_pose = {0.5 * (p1.GetPosition() + p2.GetPosition()), p1.GetAngle()};
        end_pose.Rotate(180);
    }


    p->AddConnection(
         from_road, from_lane_idx, start_pose,
         to_road, to_lane_idx, end_pose,
         _ctrl_len1, _ctrl_len2
    );
    
    if(from_lane_idx > 0)
        mRoadPtrs[from_road]->mNextJid = p->mJunctionId;
    else
        mRoadPtrs[from_road]->mPrevJid = p->mJunctionId;
    
    if(to_lane_idx > 0)
        mRoadPtrs[to_road]->mPrevJid = p->mJunctionId;
    else
        mRoadPtrs[to_road]->mNextJid = p->mJunctionId;
}

void Map::  CommitRoadInfo()
{
    for(auto & p : mRoadPtrs)
    {
        if(!p->mSecPtrs.empty())
            p->mLength = p->mSecPtrs.back()->mStartS + p->mSecPtrs.back()->mReferLine.Length();
    }
}

void Map::Save(const std::string &file_name)
{
    try
    {
        pt::write_xml(file_name, ToXML());
    }
    catch (std::exception &e)
    {
        std::cout << "Error: " << e.what() << std::endl;
    }
}

void Map::Load(const std::string &file_name)
{
    try
    {
        pt::ptree tree;
        pt::read_xml(file_name, tree);
        FromXML(tree);
    }
    catch (std::exception &e)
    {
        std::cout << "Error: " << e.what() << std::endl;
    }
}

std::vector<RoadPtr> Map::AdjacentRoadInfo(RoadPtr p_road)
{
    std::vector<RoadPtr> res;

    int jid = p_road->mNextJid;
    if(jid == -1) return res;

    for(auto const & m : mJuncPtrs[jid]->mRoadLinks)
    {
        if(m.first.first == p_road->mRoadId)
        {
            res.emplace_back(GetRoadPtrById(m.first.second));
        }
    }
    return res;
}

/*
void Map::Trajectory(std::vector<std::pair<unsigned int, int>> sequences)
{
    /*
    std::vector<Pose> res;

    for(int i = 0; i < sequences.size()-1; i++)
    {
        std::pair<int, int> k;
        for(auto & j : mJunctions)
        {
            if(j.Check({sequences[i].first, sequences[i+1].first}))
            {
                k = j.GetLink({sequences[i].first, sequences[i+1].first});
                break;
            }
        }

        auto road_traj = mRoads[sequences[i].first].Trajectory(sequences[i].second, k.first);
        res.insert(res.end(), road_traj.begin(), road_traj.end());


    }

    std::vector<Pose> res;
    for(auto x : sequences)
    {
        unsigned road_id = x.first;
        int lane_idx = x.second;

        for(auto & section : mRoads[road_id].mSections)
        {
            auto lane_pose = section.GetLanePoseByIndex(lane_idx);
            res.insert(res.end(), lane_pose.begin(), lane_pose.end());

            auto successors = section.mLanes[lane_idx].successors;
            if(!successors.empty())
                lane_idx = section.mLanes[lane_idx].successors.front();
        }
    }

}
*/

RoadPtr Map::Locate(const Vector2d &v)
{
    for(auto & x : mRoadPtrs)
    {
        if(x->Cover(v))
            return x;
    }
}

boost::property_tree::ptree Map::ToXML()
{
    pt::ptree p_map;

    for(auto & m : mRoadPtrs)
        p_map.add_child("hdmap.roads.road", m->ToXML());

    for(auto & m : mJuncPtrs)
        p_map.add_child("hdmap.junctions.junction", m->ToXML());

    return p_map;
}

void Map::FromXML(const pt::ptree &p)
{
    for(auto & r : p.get_child("hdmap.roads"))
    {
        RoadPtr pRoad(new Road());
        pRoad->FromXML(r.second);
        mRoadPtrs.emplace_back(pRoad);
    }
    Road::ROAD_ID = mRoadPtrs.back()->mRoadId + 1;

    for(auto & j : p.get_child("hdmap.junctions"))
    {
        JuncPtr pJunc(new Junction());
        pJunc->FromXML(j.second);
        mJuncPtrs.emplace_back(pJunc);
    }
}

RoadPtr Map::GetRoadPtrById(unsigned int _road_id)
{
    for(auto & x : mRoadPtrs)
    {
        if(x->mRoadId == _road_id)
            return x;
    }
    return hdmap::RoadPtr();
}

JuncPtr Map::GetJuncPtrById(unsigned int _junc_id)
{
    for(auto & x : mJuncPtrs)
    {
        if(x->mJunctionId == _junc_id)
            return x;
    }
    return hdmap::JuncPtr();
}

