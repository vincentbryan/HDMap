//
// Created by vincent on 18-10-31.
//

#include <Location.h>
#include <HDMap/msg_signal_list.h>
#include <HDMap/msg_signal.h>
#include <std_msgs/String.h>
#include <HDMap/srv_route.h>
#include <HDMap/srv_routeRequest.h>
#include <HDMap/srv_routeResponse.h>
#include "Tool/Route.h"
using namespace hdmap;

Route::Route(ros::NodeHandle & n)
{
    ros::Publisher pub = n.advertise<visualization_msgs::MarkerArray>("HDMap", 1000);
    std::shared_ptr<Sender> p_sender(new Sender(pub));
    mHDMap.SetSender(p_sender);

    mClient = n.serviceClient<HDMap::srv_route>("map_service");

    mPubTrafficLight = n.advertise<HDMap::msg_signal_list>("map_to_traffic_light", 1000);
    mPubVIZ = n.advertise<visualization_msgs::Marker>("map_to_rviz", 1000);
    mPubPlanner = n.advertise<std_msgs::String>("map_to_planner", 1000);

    mSubGPS = n.subscribe("/gps/Localization", 1, &Route::LocationCallBack, this);
    mServer = n.advertiseService("map_command", &Route::OnCommandRequest, this);

    mCurrentPosition = Vector2d();
}

void Route::LocationCallBack(const nox_msgs::Location &msg)
{
    std::lock_guard<std::mutex> lock(mLock);
    mCurrentPosition = {msg.x, msg.y};
}

bool Route::OnCommandRequest(HDMap::srv_map_cmd::Request &req, HDMap::srv_map_cmd::Response &res)
{
    if(req.cmd == "start")
    {
        HDMap::srv_route srv;
        srv.request.start_rid = req.argv1;
        srv.request.end_rid = req.argv2;
        if(mClient.call(srv))
        {
            mHDMap.Clear();
            mRecord.Reset();

            std::stringstream ss;
            ss << srv.response.route;
            std::cout << ss.str() << std::endl;
            try
            {
                pt::ptree tree;
                pt::read_xml(ss, tree);
                mHDMap.FromXML(tree);

                for(auto & x : mHDMap.mRoadPtrs)
                    res.route.emplace_back(x->mRoadId);

                return true;
            }
            catch (std::exception & e)
            {
                std::cout << "Error: " << e.what() << std::endl;
                return false;
            }
        }
        else
        {
            ROS_ERROR_STREAM("Failed to received from MapService!!!");
            return false;
        }
    }
    else
    {
        mHDMap.Clear();
        mRecord.Reset();
        mCurrentPosition = {0, 0};
    }
    return true;
}

void Route::SendMap()
{
    pt::ptree p_map;
    if(mRecord.curr_rid != -1)
    {
        auto m = mHDMap.GetRoadPtrById(mRecord.curr_rid);
        if(m != nullptr)
            p_map.add_child("hdmap.roads.road", m->ToXML());
    }
    if(mRecord.next_rid != -1)
    {
        auto m = mHDMap.GetRoadPtrById(mRecord.next_rid);
        if(m != nullptr)
            p_map.add_child("hdmap.roads.road", m->ToXML());
    }
    if(mRecord.curr_jid != -1)
    {
        auto m = mHDMap.GetJuncPtrById(mRecord.curr_jid);
        if(m != nullptr)
            p_map.add_child("hdmap.junctions.junction", m->ToXML());
    }

    std::stringstream ss;
    pt::write_xml(ss, p_map);
    std_msgs::String s;
    s.data = ss.str();
    mPubPlanner.publish(s);
}

void Route::SendTrafficInfo(const Vector2d &v)
{
    if(mRecord.curr_rid != -1)
    {
        auto signals = mHDMap.mRoadPtrs[mRecord.curr_idx]->GetSignals();

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
                signal.theta = s->mDirection.Value();
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
        mPubTrafficLight.publish(ss);
        ROS_INFO("(%8.3f, %8.3f): In road[%d] with [%d] signals, detection state: [%d]", v.x, v.y, mRecord.curr_rid, signals.size(), detected);
    }
}

void Route::SendGPS(const Vector2d &v)
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

    mPubVIZ.publish(marker);
}

void Route::Process()
{
    mLock.lock();
    Vector2d curr_pos = mCurrentPosition;
    mLock.unlock();

    SendGPS(curr_pos);

    if(!mRecord.is_init)
    {
        if(mHDMap.mRoadPtrs.empty())
        {
            ROS_INFO("(%8.3f, %8.3f): No route, waiting for map command", curr_pos.x, curr_pos.y);
            return;
        }

        for(int i = 0; i < mHDMap.mRoadPtrs.size(); ++i)
        {
            if(mHDMap.mRoadPtrs[i]->Cover(curr_pos))
            {
                mRecord.curr_rid = mHDMap.mRoadPtrs[i]->mRoadId;
                mRecord.curr_idx = i;
                mRecord.curr_jid = mHDMap.mRoadPtrs[i]->mNextJid;

                if(i + 1 < mHDMap.mRoadPtrs.size())
                {
                    mRecord.next_rid = mHDMap.mRoadPtrs[i+1]->mRoadId;
                }
                mRecord.is_init = true;
                break;
            }
        }
        if(!mRecord.is_init)
        {
            ROS_ERROR("(%8.3f, %8.3f): Initial failed, please ensure that current position is in the routing!!!" , curr_pos.x, curr_pos.y);
            return;
        }
    }

    if(mRecord.curr_idx < mHDMap.mRoadPtrs.size())
    {
        if(mHDMap.mRoadPtrs[mRecord.curr_idx]->Cover(curr_pos))
        {
            SendTrafficInfo(curr_pos);
            SendMap();
            return;
        }
    }

    if(mRecord.curr_jid != -1)
    {
        for(auto & j : mHDMap.mJuncPtrs)
        {
            if(j->mJunctionId == mRecord.curr_jid)
            {
                if(j->Cover(curr_pos))
                {
                    ROS_INFO("(%8.3f, %8.3f): In junction[%d]", curr_pos.x, curr_pos.y, j->mJunctionId);
                    SendMap();
                    return;
                }
            }
        }
    }

    if(mRecord.curr_idx + 1 < mHDMap.mRoadPtrs.size())
    {
        if(mHDMap.mRoadPtrs[mRecord.curr_idx+1]->Cover(curr_pos))
        {
            ROS_INFO("(%8.3f, %8.3f): In next_road, trying to update record", curr_pos.x, curr_pos.y);
            if(mRecord.curr_idx + 1 < mHDMap.mRoadPtrs.size())
            {
                mRecord.curr_idx++;
                mRecord.curr_rid = mHDMap.mRoadPtrs[mRecord.curr_idx]->mRoadId;


                if(mRecord.curr_idx + 1 < mHDMap.mRoadPtrs.size())
                {
                    mRecord.curr_jid = mHDMap.mRoadPtrs[mRecord.curr_idx]->mNextJid;
                    mRecord.next_rid = mHDMap.mRoadPtrs[mRecord.curr_idx+1]->mRoadId;
                }
                else
                    mRecord.curr_jid = mRecord.next_rid = -1;
            }
            ROS_INFO("(%8.3f, %8.3f): Record updated", curr_pos.x, curr_pos.y);
            SendMap();
            return;
        }
    }
    ROS_ERROR("(%8.3f, %8.3f): Current position is out the routing!!!" , curr_pos.x, curr_pos.y);
}
