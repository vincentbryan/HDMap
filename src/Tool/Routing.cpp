//
// Created by vincent on 18-10-31.
//

#include <Location.h>
#include <msg_signal_list.h>
#include <msg_signal.h>
#include "Routing.h"
using namespace hdmap;

Routing::Routing(ros::NodeHandle & n, const hdmap::Map &map) :
    mRouting(map)
{
    mMapToTrafficLight = n.advertise<HDMap::msg_signal_list>("map_to_traffic_light", 1000);
    mSender = n.advertise<visualization_msgs::Marker>("Routing", 1000);
}

void Routing::CallBack(const nox_msgs::Location &msg)
{
    Vector2d curr_pos = {msg.x, msg.y};
    SendGPS(curr_pos);

    if(!mRecord.is_init)
    {
        for(int i = 0; i < mRouting.mRoadPtrs.size(); ++i)
        {
            if(mRouting.mRoadPtrs[i]->Cover(curr_pos))
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

    if(mRecord.curr_idx < mRouting.mRoadPtrs.size())
    {
        if(mRouting.mRoadPtrs[mRecord.curr_idx]->Cover(curr_pos))
        {
//            ROS_INFO_STREAM("In curr_road: " << mRecord.curr_rid);
            TrafficInfo(curr_pos);
//            SendMap();
            return;
        }
    }

    if(mRecord.curr_jid != -1)
    {
        for(auto & j : mRouting.mJuncPtrs)
        {
            if(j->mJunctionId == mRecord.curr_jid)
            {
                if(j->Cover(curr_pos))
                {
                    ROS_INFO_STREAM("In curr_junc: " << j->mJunctionId);
//                    SendMap();
                    return;
                }
            }
        }
    }

    if(mRecord.curr_idx + 1 < mRouting.mRoadPtrs.size())
    {
        if(mRouting.mRoadPtrs[mRecord.curr_idx+1]->Cover(curr_pos))
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
//            SendMap();
        }
    }

    ROS_ERROR_STREAM("curr_pos is out of local routing!!!");
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


void Routing::TrafficInfo(const Vector2d &v)
{
    if(mRecord.curr_rid != -1)
    {
        auto signals = mRouting.mRoadPtrs[mRecord.curr_idx]->GetSignals();

        int cnt = 0;
        for(auto & s : signals)
        {
            double t = sqrt((v.x-s->x)*(v.x-s->x) + (v.y-s->y)*(v.y-s->y));
            if(t < 100) cnt++;
        }

        HDMap::msg_signal_list ss;
        bool detected = false;
        if(!signals.empty() and cnt == signals.size())
        {
            detected = true;
            for(auto & s : signals)
            {
                HDMap::msg_signal signal;
                signal.x = s->x;
                signal.y = s->y;
                signal.z = s->z;
                signal.type = s->mType;
                signal.left = (s->mInfo[0] == '1');
                signal.forward = (s->mInfo[1] == '1');
                signal.right = (s->mInfo[2] == '1');

                if(s->mInfo[3] == '1')
                    signal.shape = "circle";
                else
                    signal.shape = "arrow";

                ss.signal_list.emplace_back(signal);
            }

        }
        mMapToTrafficLight.publish(ss);

        ROS_INFO_STREAM("curr_rid: " << mRecord.curr_rid << " signal: " << signals.size() << " detected: " << detected);
    }
}

void Routing::SendGPS(const Vector2d &v)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/hdmap";
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;

    marker.id = 1;
    marker.type = visualization_msgs::Marker::ARROW;

    double scale = 1.0;
    marker.scale.z = scale;
    marker.scale.x = scale;
    marker.scale.y = scale;

    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    geometry_msgs::Point start;
    geometry_msgs::Point end ;
    start.x = v.x;
    start.y = v.y;
    start.z = 0.25 + scale;

    end.x = v.x;
    end.y = v.y;
    end.z = 0.25;

    marker.points.emplace_back(start);
    marker.points.emplace_back(end);

    mSender.publish(marker);
}
