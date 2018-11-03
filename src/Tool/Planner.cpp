//
// Created by vincent on 18-10-24.
//

#include <boost/property_tree/ptree.hpp>
#include "Planner.h"

using namespace hdmap;
namespace pt = boost::property_tree;

Planner::Planner(const Map & map, std::shared_ptr<Sender> _sender) :
    mHDMap(map),
    pSender(std::move(_sender))
{}

void Planner::GlobalPlanning()
{
    pStart = mHDMap.Locate(mStartPoint);
    pEnd = mHDMap.Locate(mEndPoint);

    ROS_INFO_STREAM("Start Point: " << pStart->mRoadId << " " << pStart->mDirection);
    ROS_INFO_STREAM("End   Point: " << pEnd->mRoadId << " " << pEnd->mDirection);

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
    mRouting = mAllRouting.front();
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
            mHDMap.GetJuncPtrById(jid)->GetSubRoadLink(mRouting[i]->mRoadId,
                                   mRouting[i]->mDirection,
                                   mRouting[i+1]->mRoadId,
                                   mRouting[i+1]->mDirection).Send(*pSender);
        }
    }
}

std::string Planner::ToXML(const std::string &file_name)
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
            auto sub_road_link =  mHDMap.mJuncPtrs[jid]->GetSubRoadLink(mRouting[i]->mRoadId,
                                                                        mRouting[i]->mDirection,
                                                                        mRouting[i+1]->mRoadId,
                                                                        mRouting[i+1]->mDirection);
            p_junc.add_child("roadLink", sub_road_link.ToXML());
            tree.add_child("hdmap.junctions.junction", p_junc);
            p_junc.clear();
        }
        pt::write_xml(file_name, tree);
    }
    catch (std::exception &e)
    {
        std::cout << "Error: " << e.what() << std::endl;
    }
}
