//
// Created by vincent on 18-10-24.
//

#include <boost/property_tree/ptree.hpp>
#include "Tool/Planner.h"

using namespace hdmap;
namespace pt = boost::property_tree;

Planner::Planner(const Map & map, std::shared_ptr<Sender> _sender) :
    mHDMap(map),
    pSender(std::move(_sender))
{}

void Planner::GlobalPlanning()
{
//    pStart = mHDMap.Locate(mStartPoint);
//    pEnd = mHDMap.Locate(mEndPoint)
    mAllRouting.clear();
    mRouting.clear();

    std::vector<RoadPtr> v;
    v.emplace_back(pStart);
    DFS(pStart, v);
    Evaluate();
}

void Planner::DFS(RoadPtr p_curr, std::vector<RoadPtr> v)
{
    if(p_curr->mRoadId == pEnd->mRoadId)
    {
        mAllRouting.emplace_back(v);
    }
    else
    {
        for(auto & x : mHDMap.AdjacentRoadInfo(p_curr))
        {
            if(!is_visited[x->mRoadId])
            {
                is_visited[x->mRoadId] = true;
                v.emplace_back(x);
                DFS(x, v);
                v.pop_back();
                is_visited[x->mRoadId] = false;
            }
        }
    }
}

void Planner::Evaluate()
{
    if(mAllRouting.empty())
    {
        ROS_ERROR("Global planning failed, please reset the start point and end point...");
        return;
    }

    int len = 100000;
    int idx = -1;
    for(int i = 0; i < mAllRouting.size(); ++i)
    {
        if(mAllRouting[i].size() < len)
        {
            idx = i;
            len = mAllRouting[i].size();
        }

    }
    if(idx < 0 or idx >= mAllRouting.size())
    {
        ROS_ERROR("Global planning failed, please reset the start point and end point...");
        return;
    }

    mRouting = mAllRouting[idx];
}

void Planner::Send()
{
    auto m1 = pSender->GetCone(mStartPoint, 0.0, 1.0, 1.0, 1.0, 3.0);
    auto m2 = pSender->GetCone(mEndPoint, 1.0, 1.0, 0.0, 1.0, 3.0);
    pSender->array.markers.emplace_back(m1);
    pSender->array.markers.emplace_back(m2);
    pSender->Send();

    for(int i = 0; i < mRouting.size(); i++)
    {
        mRouting[i]->Send(*pSender);

        if(i + 1 < mRouting.size())
        {
            int jid = mRouting[i]->mNextJid;
            mHDMap.GetJuncPtrById(jid)->GetRoadLink(mRouting[i]->mRoadId, mRouting[i+1]->mRoadId).Send(*pSender);
        }
    }
}

void Planner::ToXML(std::string &str)
{
    try
    {
        pt::ptree tree;

        for(auto & x : mRouting)
            tree.add_child("hdmap.roads.road", x->ToXML());

        pt::ptree p_junc;
        for(int i = 0; i + 1< mRouting.size(); i++)
        {
            int jid = mRouting[i]->mNextJid;
            assert(jid != -1);
            auto junc = mHDMap.mJuncPtrs[jid];

            p_junc.add("<xmlattr>.id",junc->mJunctionId);

            if(junc->mVertices.empty())junc->GenerateVertices();

            pt::ptree p_vec;
            pt::ptree p_v;
            for(auto & v : junc->mVertices)
            {
                p_v.add("<xmlattr>.x", v.x);
                p_v.add("<xmlattr>.y", v.y);
                p_vec.add_child("vertex", p_v);
                p_v.clear();
            }
            p_junc.add_child("vertice", p_vec);

            auto road_link =  mHDMap.GetJuncPtrById(jid)->GetRoadLink(mRouting[i]->mRoadId, mRouting[i+1]->mRoadId);
            p_junc.add_child("roadLink", road_link.ToXML());
            tree.add_child("hdmap.junctions.junction", p_junc);
            p_junc.clear();
        }
        std::stringstream ss;
        pt::write_xml(ss, tree);
        str = ss.str();
    }
    catch (std::exception &e)
    {
        std::cout << "Error: " << e.what() << std::endl;
    }
}

bool Planner::OnRequest(HDMap::srv_route::Request &req, HDMap::srv_route::Response &res)
{
    pStart = mHDMap.GetRoadPtrById(req.start_rid);
    pEnd = mHDMap.GetRoadPtrById(req.end_rid);

    ROS_INFO("Request: road[%d] --> road[%d]", req.start_rid, req.end_rid);
    GlobalPlanning();

    if(mRouting.empty())
    {
        ROS_ERROR_STREAM("Result: Failed!!!");
        return false;
    }

    ROS_INFO_STREAM("Result:");
    for(auto & x : mRouting)
    {
        ROS_INFO_STREAM("\t" << x->mRoadId);
    }
    ToXML(res.route);
    Send();
    return true;
}
