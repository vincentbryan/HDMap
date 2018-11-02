//
// Created by vincent on 18-10-31.
//

#include <Location.h>
#include "Routing.h"
using namespace hdmap;

Routing::Routing(const hdmap::Map &map) :
    mRouting(map)
{}

void Routing::CallBack(const HDMap::Location &msg)
{
    Vector2d curr_pos = {msg.x, msg.y};

    if(!mRecord.is_init)
    {
        double min_dist = 1000000;
        int min_rid = -1;
        for(auto & r : mRouting.mRoadPtrs)
        {
            double t = r->Distance(curr_pos);
            if(t < min_dist)
            {
                min_dist = t;
                min_rid = r->mRoadId;
            }
        }

        for(int i = 0; i < mRouting.mRoadPtrs.size(); ++i)
        {
            if(mRouting.mRoadPtrs[i]->mRoadId == min_rid)
            {
                mRecord.curr_rid = mRouting.mRoadPtrs[i]->mRoadId;
                mRecord.curr_idx = i;
                mRecord.curr_jid = mRouting.mRoadPtrs[i]->mNextJid;

                if(i + 1 < mRouting.mRoadPtrs.size())
                {
                    mRecord.next_rid = mRouting.mRoadPtrs[i+1]->mRoadId;
                }
                mRecord.is_init = true;
                break;
            }
        }
        if(!mRecord.is_init)
        {
            ROS_ERROR("Current Position is out of the routing!!!");
            return;
        }
    }

    double d1 = 1000000;
    double d2 = 1000000;
    double d3 = 1000000;

    if(mRecord.curr_idx < mRouting.mRoadPtrs.size())
    {
        d1 = mRouting.mRoadPtrs[mRecord.curr_idx]->Distance(curr_pos);
    }
    if(mRecord.curr_jid != -1)
    {
        for(auto & j : mRouting.mJuncPtrs)
        {
            if(j->mJunctionId == mRecord.curr_jid)
            {
                d2 = j->Distance(curr_pos);
            }
        }
    }
    if(mRecord.curr_idx + 1 < mRouting.mRoadPtrs.size())
    {
        d3 = mRouting.mRoadPtrs[mRecord.curr_idx + 1]->Distance(curr_pos);
    }

    if(d1 < d2 && d1 < d3)
    {
        ROS_INFO_STREAM("In curr_road: " << mRecord.curr_rid << " [ " << d1 << " " << d2 << " " << d3 << "]");
        SendMap();
    }
    if(d2 < d1 && d2 < d3)
    {
        ROS_INFO_STREAM("In curr_junc: " << mRecord.curr_jid << " [ " << d1 << " " << d2 << " " << d3 << "]");
        SendMap();
    }
    if(d3 < d1 && d3 < d2)
    {
        ROS_ERROR("In next_road, trying to update record");
        if(mRecord.curr_idx + 1 < mRouting.mRoadPtrs.size())
        {
            mRecord.curr_idx++;
            mRecord.curr_rid = mRouting.mRoadPtrs[mRecord.curr_idx]->mRoadId;


            if(mRecord.curr_idx + 1 < mRouting.mRoadPtrs.size())
            {
                mRecord.curr_jid = mRouting.mRoadPtrs[mRecord.curr_idx]->mNextJid;
                mRecord.next_rid = mRouting.mRoadPtrs[mRecord.curr_idx+1]->mRoadId;
            }
            else
                mRecord.curr_jid = mRecord.next_rid = -1;
        }
        ROS_INFO_STREAM("Record updated");
        SendMap();
    }
}

void Routing::SendMap()
{
    pt::ptree p_map;
    if(mRecord.curr_rid != -1)
    {
        auto m = mRouting.GetRoadPtrById(mRecord.next_rid);
        if(m != nullptr)
            p_map.add_child("hdmap.roads.road", m->ToXML());
    }
    if(mRecord.next_rid != -1)
    {
        auto m = mRouting.GetRoadPtrById(mRecord.next_rid);
        if(m != nullptr)
            p_map.add_child("hdmap.roads.road", m->ToXML());
    }
    if(mRecord.curr_jid != -1)
    {
        auto m = mRouting.GetJuncPtrById(mRecord.curr_jid);
        if(m != nullptr)
            p_map.add_child("hdmap.junctions.junction", m->ToXML());
    }
//    std::stringstream ss;
//    pt::write_xml(ss, p_map);
//    std::string s = ss.str();
//    std::cout << s << std::endl;
}
