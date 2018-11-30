//
// Created by vincent on 18-10-31.
//

#include <Location.h>
#include <HDMap/msg_signal_list.h>
#include <HDMap/msg_route_region.h>
#include <std_msgs/String.h>
#include <HDMap/srv_route.h>
#include <HDMap/srv_map_data.h>
#include <unordered_set>
#include "Tool/Client.h"
using namespace hdmap;

Client::Client(ros::NodeHandle & n)
{
    ros::Publisher pub = n.advertise<visualization_msgs::MarkerArray>("HDMap", 1000);
    std::shared_ptr<Sender> p_sender(new Sender(pub));
    mCurPlanMap.SetSender(p_sender);

    mPlanClient = n.serviceClient<HDMap::srv_route>("map_plan_service");
    mDataClient = n.serviceClient<HDMap::srv_map_data>("map_data_service");

    mPubRouteRegion = n.advertise<HDMap::msg_route_region>("map_pub_route_region",1);
    mPubTrafficLight = n.advertise<HDMap::msg_signal_list>("map_to_traffic_light", 1);
    mPubVIZ = n.advertise<visualization_msgs::Marker>("map_to_rviz", 1000);
    mPubPlanner = n.advertise<std_msgs::String>("map_to_planner", 1);

    mSubGPS = n.subscribe("Localization", 1, &Client::LocationCallBack, this);
    mServer = n.advertiseService("map_command", &Client::OnCommandRequest, this);

    mCurrentPosition = Vector2d();
}

void Client::LocationCallBack(const nox_msgs::Location &msg)
{
    std::lock_guard<std::mutex> lock(mLock);
    mCurrentPosition = {msg.x, msg.y};
}

bool Client::OnCommandRequest(HDMap::srv_map_cmd::Request &req, HDMap::srv_map_cmd::Response &res)
{
    if(req.cmd == "start" || req.cmd == "seq")
    {
        HDMap::srv_route srv;
        srv.request.method = req.cmd;
        srv.request.argv = req.argv;
        if(mPlanClient.call(srv))
        {
            mCurPlanMap.Clear();
            mRecord.Reset();
            srv.request.method = req.cmd;
            srv.request.argv = req.argv;
            std::stringstream ss;
            ss << srv.response.route;
            std::cout << ss.str() << std::endl;
            try
            {
                pt::ptree tree;
                srv.request.method = req.cmd;
                srv.request.argv = req.argv;
                pt::read_xml(ss, tree);
                mCurPlanMap.FromXML(tree);

                for(auto & x : mCurPlanMap.mRoadPtrs)
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
        mCurPlanMap.Clear();
        mRecord.Reset();
        mCurrentPosition = {0, 0};
    }
    return true;
}

void Client::SendMap()
{
    pt::ptree p_map;
    if(mRecord.curr_rid != -1)
    {
        auto m = mCurPlanMap.GetRoadPtrById(mRecord.curr_rid);
        if(m != nullptr)
            p_map.add_child("hdmap.roads.road", m->ToXML());
    }
    if(mRecord.next_rid != -1)
    {
        auto m = mCurPlanMap.GetRoadPtrById(mRecord.next_rid);
        if(m != nullptr)
            p_map.add_child("hdmap.roads.road", m->ToXML());
    }
    if(mRecord.curr_jid != -1)
    {
        auto m = mCurPlanMap.GetJuncPtrById(mRecord.curr_jid);
        if(m != nullptr)
            p_map.add_child("hdmap.junctions.junction", m->ToXML());
    }

    std::stringstream ss;
    pt::write_xml(ss, p_map);
    std_msgs::String s;
    s.data = ss.str();
    mPubPlanner.publish(s);
}

void Client::SendTrafficInfo(const Vector2d &v)
{
    if(mRecord.curr_rid != -1)
    {
        auto signals = mCurPlanMap.mRoadPtrs[mRecord.curr_idx]->GetSignals();

        int cnt = 0;
        for(auto & s : signals)
        {
            double t = sqrt((v.x-s->x)*(v.x-s->x) + (v.y-s->y)*(v.y-s->y));
            if(t < 135) cnt++;
        }

        HDMap::msg_signal_list ss;

        ss.header.frame_id = "/hdmap";
        ss.header.stamp = ros::Time::now();

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

void Client::SendGPS(const Vector2d &v)
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

void Client::Process()
{
    mLock.lock();
    Vector2d curr_pos = mCurrentPosition;
    mLock.unlock();

    SendGPS(curr_pos);
    SendNearPolygonRegion(curr_pos);

    // Global initial location
    if(!mRecord.is_init)
    {
        if(mCurPlanMap.mRoadPtrs.empty())
        {
            ROS_INFO("(%8.3f, %8.3f): No route, waiting for map command", curr_pos.x, curr_pos.y);
            return;
        }

        for(int i = 0; i < mCurPlanMap.mRoadPtrs.size(); ++i)
        {
            if(mCurPlanMap.mRoadPtrs[i]->Cover(curr_pos))
            {
                mRecord.curr_rid = mCurPlanMap.mRoadPtrs[i]->mRoadId;
                mRecord.curr_idx = i;
                //mRecord.curr_jid = mCurPlanMap.mRoadPtrs[i]->mNextJid;
                if(mRecord.curr_rid+1 == mCurPlanMap.mRoadPtrs.size())
                {
                    mRecord.curr_jid = -1;
                }
                if(i + 1 < mCurPlanMap.mRoadPtrs.size())
                {
                    mRecord.next_rid = mCurPlanMap.mRoadPtrs[i+1]->mRoadId;
                }
                mRecord.is_init = true;
                break;
            }
        }
        if(!mRecord.is_init)
        {
            ROS_ERROR("(%8.3f, %8.3f): Initial failed, please ensure that current position is in a road" , curr_pos.x, curr_pos.y);
            return;
        }
    }

    // send the current road information
    if(mRecord.curr_idx < mCurPlanMap.mRoadPtrs.size())
    {
        if(mCurPlanMap.mRoadPtrs[mRecord.curr_idx]->Cover(curr_pos))
        {
            SendTrafficInfo(curr_pos);
            SendMap();
            return;
        }
    }

    // find the junction and send
    if(mRecord.curr_jid != -1)
    {
        for(auto & j : mCurPlanMap.mJuncPtrs)
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

    // find the next road and send
    if(mRecord.curr_idx + 1 < mCurPlanMap.mRoadPtrs.size())
    {
        if(mCurPlanMap.mRoadPtrs[mRecord.curr_idx+1]->Cover(curr_pos))
        {
            ROS_INFO("(%8.3f, %8.3f): In next_road, trying to update record", curr_pos.x, curr_pos.y);
            if(mRecord.curr_idx + 1 < mCurPlanMap.mRoadPtrs.size())
            {
                mRecord.curr_idx++;
                mRecord.curr_rid = mCurPlanMap.mRoadPtrs[mRecord.curr_idx]->mRoadId;


                if(mRecord.curr_idx + 1 < mCurPlanMap.mRoadPtrs.size())
                {
                    mRecord.curr_jid = mCurPlanMap.mRoadPtrs[mRecord.curr_idx]->mNextJid;
                    mRecord.next_rid = mCurPlanMap.mRoadPtrs[mRecord.curr_idx+1]->mRoadId;
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

void Client::SendNearPolygonRegion(const Vector2d &v, double radius) {
    assert(radius>=3 && radius<=500);
    
    static Vector2d last_v(-1,-1);
    static std::unordered_set<int> last_road_ids{};
    static std::unordered_set<int> last_junc_ids{};
    static int _subscribe_num;

    bool must_recalu = false;
    int subscribe_num = mPubRouteRegion.getNumSubscribers();
    if(subscribe_num>_subscribe_num)
    {
        must_recalu = true;
    }
    _subscribe_num = subscribe_num;

    std::unordered_set<int> cur_road_ids{};
    std::unordered_set<int> cur_junc_ids{};
    
    const double distance_tolerence = 1;
    if (!must_recalu && (last_v!=Vector2d(-1,-1) && Vector2d::Distance(v,last_v)<distance_tolerence))
    {
        ROS_INFO("Detected Roads and Junctions are same with last time, region message would not be sent.");
        return;
    }
    last_v = v;

    /// 寻找附近的road和junction
    std::vector<RoadPtr > near_roads;
    std::vector<JuncPtr > near_juncs;
    std::string roads_info_str,juncs_info_str;

    HDMap::srv_map_data srv;

    srv.request.type = "RoadByPos";
    srv.request.argv.emplace_back(v.x);
    srv.request.argv.emplace_back(v.y);
    srv.request.argv.emplace_back(radius);

    if( mDataClient.call(srv)) {
        try{
            pt::ptree tree;
            std::stringstream ss;
            ss << srv.response.res;
            pt::read_xml(ss, tree);
            for (auto &r: tree.get_child("hdmap.roads")) {
                auto rptr = mCurPlanMap.GetRoadPtrById(r.second.get<int>("<xmlattr>.id"));
                if (rptr != nullptr) {
                    near_roads.emplace_back(rptr);
                } else {
                    RoadPtr _tmp_road_ptr(new Road());
                    _tmp_road_ptr->FromXML(r.second);
                    near_roads.emplace_back(_tmp_road_ptr);
                }
                cur_road_ids.insert(near_roads.back()->mRoadId);
                roads_info_str += std::to_string(near_roads.back()->mRoadId)+" ";
            }
        }
        catch (...){
            ROS_INFO("Not receive any roads or something error happen.");
        }
    }
    srv.response.res="";
    srv.request.type = "JunctionByPos";
    if( mDataClient.call(srv))
    {
        try{
            pt::ptree tree;
            std::stringstream ss;
            ss << srv.response.res;
            pt::read_xml(ss,tree);
            for(auto & j: tree.get_child("hdmap.junctions"))
            {
                auto jptr = mCurPlanMap.GetJuncPtrById(j.second.get<int>("<xmlattr>.id"));
                if(jptr!= nullptr){
                    near_juncs.push_back(jptr);
                }
                else{
                    JuncPtr _tmp_junction_ptr(new Junction());
                    _tmp_junction_ptr->FromXML(j.second);
                    near_juncs.push_back(_tmp_junction_ptr);
                }
                cur_junc_ids.insert(near_juncs.back()->mJunctionId);
                juncs_info_str += std::to_string(near_juncs.back()->mJunctionId)+" ";
            }
        }
        catch (...){
            ROS_INFO("Not receive any junctions or something error happen.");
        }

    }


    if (juncs_info_str.empty()) juncs_info_str ="none ";
    if (roads_info_str.empty()) roads_info_str ="none ";
    ROS_INFO("(%8.3f, %8.3f) Region: Roads { %s} Junctions { %s}", mCurrentPosition.x,mCurrentPosition.y,roads_info_str.c_str(),juncs_info_str.c_str());

    if(!must_recalu && (last_junc_ids==cur_junc_ids && last_road_ids==cur_road_ids))
    {
        ROS_INFO("Detected Roads and Junctions are same with last time, region message would not be sent.");
        return;
    }
    last_junc_ids = cur_junc_ids;
    last_road_ids = cur_road_ids;

    // TODO: 双向道路中间分开的合并处理

    HDMap::msg_route_region mrr;
    mrr.header.frame_id = "hdmap";
    mrr.header.stamp = ros::Time::now();

    for(auto &rptr: near_roads)
    {
        geometry_msgs::Polygon pg;
        rptr->GenerateRegionVertics();
        for(auto &rv: rptr->mRegionVertices)
        {
            geometry_msgs::Point32 p;
            p.x = static_cast<float>(rv.x);
            p.y = static_cast<float>(rv.y);
            p.z = 0;
            pg.points.push_back(p);
        }
        mrr.polygons.emplace_back(pg);
    }
    for(auto &jptr: near_juncs)
    {
        geometry_msgs::Polygon pg;
        for(auto& jv: jptr->mRegionVertices)
        {
            geometry_msgs::Point32 p;
            p.x = static_cast<float>(jv.x);
            p.y = static_cast<float>(jv.y);
            p.z = 0;
            pg.points.push_back(p);
        }
        mrr.polygons.emplace_back(pg);
    }


    mPubRouteRegion.publish(mrr);

}
