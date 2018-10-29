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

    ROS_INFO_STREAM("Start Point: " << pStart->iRoadId << " " << pStart->direction);
    ROS_INFO_STREAM("End   Point: " << pEnd->iRoadId << " " << pEnd->direction);

    std::vector<SubRoadPtr> v;
    v.emplace_back(pStart);
    DFS(pStart, v);
    Evaluate();

    mRecord.curr_idx = 0;
    mRecord.curr_rid = mRouting.front()->iRoadId;
    mRecord.curr_dir = mRouting.front()->direction;

    if(mRecord.curr_idx + 1 < mRouting.size())
    {
        mRecord.next_rid = mRouting[mRecord.curr_idx+1]->iRoadId;
        mRecord.next_dir = mRouting[mRecord.curr_idx+1]->direction;
        mRecord.curr_jid = mRouting[mRecord.curr_rid]->iNextJid;
    }
    else
    {
        mRecord.next_rid = -1;
        mRecord.curr_jid = -1;
    }

}


void Planner::DFS(Planner::SubRoadPtr p_curr, std::vector<Planner::SubRoadPtr> v)
{
    if(p_curr->iRoadId == pEnd->iRoadId and p_curr->direction == pEnd->direction)
    {
        mAllRouting.emplace_back(v);
    }
    else
    {
        for(auto & x : mHDMap.AdjacentRoadInfo(p_curr))
        {
            if(!is_visited[x->iRoadId])
            {
                is_visited[x->iRoadId] = true;
                v.emplace_back(x);
                DFS(x, v);
                v.pop_back();
                is_visited[x->iRoadId] = false;
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
            int jid = mRouting[i]->iNextJid;
            mHDMap.mJuncPtrs[jid]->GetSubRoadLink(mRouting[i]->iRoadId,
                                   mRouting[i]->direction,
                                   mRouting[i+1]->iRoadId,
                                   mRouting[i+1]->direction).Send(*pSender);
        }
    }
}

std::string Planner::ToXML(const std::string &file_name)
{
    try
    {
        pt::ptree tree;

        pt::ptree p_road;
        for(auto & x : mRouting)
        {
            p_road.add("<xmlattr>.id", x->iRoadId);
            p_road.add("<xmlattr>.direction", x->direction);
            p_road.add("<xmlattr>.length", x->pBaseRoad->dLength);
            p_road.add("<xmlattr>.prev_jid", x->iPrevJid);
            p_road.add("<xmlattr>.next_jid", x->iNextJid);

            pt::ptree p_sec;

            for(auto & sec : x->mSubRoadSecPtrs)
            {
                p_sec.add("<xmlattr>.id", sec->iSectionId);
                p_sec.add("<xmlattr>.s", sec->s);

                p_sec.add("<xmlattr>.left_idx", sec->most_left_lane_idx);
                p_sec.add("<xmlattr>.right_idx",sec->most_right_lane_idx);

                for(auto & p : sec->mReferLine.GetParam())
                    p_sec.add("refer_line.param", p);

                pt::ptree p_lane;
                for(auto & k : sec->mLanes)
                {
                    p_lane.add("<xmlattr>.idx", k.first);
                    p_lane.add("<xmlattr>.id", k.second.land_id);
                    p_lane.add("<xmlattr>.type", "Driving");
                    p_lane.add("offset.<xmlattr>.type", "cubic_function");
                    p_lane.add("offset.<xmlattr>.s", 0);
                    p_lane.add("offset.a", k.second.width.a);
                    p_lane.add("offset.b", k.second.width.b);
                    p_lane.add("offset.c", k.second.width.c);
                    p_lane.add("offset.d", k.second.width.d);

                    for(auto & s : k.second.predecessors)
                        p_lane.add("predecessors", s);

                    for(auto & s : k.second.successors)
                        p_lane.add("successors", s);

                    p_sec.add_child("lane", p_lane);
                    p_lane.clear();
                }
                p_road.add_child("lanesection", p_sec);
                p_sec.clear();
            }

            pt::ptree p_sig;
            for(auto & sig : x->mSubRoadSigPtrs)
            {
                p_sig.add("x", sig->position.x);
                p_sig.add("y", sig->position.y);
                p_sig.add("direction", sig->direction);
                p_sig.add("type", sig->type);
                p_sig.add("info", sig->info);
                p_road.add_child("signals", p_sig);
                p_sig.clear();
            }
            tree.add_child("hdmap.roads.road", p_road);
            p_road.clear();
        }


        pt::ptree p_junc;
        for(int i = 0; i + 1< mRouting.size(); i++)
        {
            int jid = mRouting[i]->iNextJid;
            assert(jid != -1);
            auto junc = mHDMap.mJuncPtrs[jid];

            p_junc.add("<xmlattr>.id",junc->iJunctionId);
            auto sub_road_link =  mHDMap.mJuncPtrs[jid]->GetSubRoadLink(mRouting[i]->iRoadId,
                                                                        mRouting[i]->direction,
                                                                        mRouting[i+1]->iRoadId,
                                                                        mRouting[i+1]->direction);

            pt::ptree p_conns;
            for(auto & k : sub_road_link.vLaneLinks)
            {
                p_conns.add("in", k.iFromIndex);
                p_conns.add("out", k.iToIndex);

                for(auto d : k.mReferLine.GetParam())
                    p_conns.add("param", d);

                p_junc.add_child("connection", p_conns);
                p_conns.clear();
            }

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
