#include <utility>

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
#include <Tool/Client.h>

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

    mCurrentPosition = Coor();
}

void Client::LocationCallBack(const nox_msgs::Location &msg)
{
    std::lock_guard<std::mutex> lock(mLock);
    mCurrentPosition = {msg.x, msg.y};
}

bool Client::OnCommandRequest(HDMap::srv_map_cmd::Request &req, HDMap::srv_map_cmd::Response &res)
{
    if(req.cmd == "start")
    {
        if(PlanByCommand(req.cmd, req.argv))
        {
            for(auto & x : mCurPlanMap.mRoadPtrs)
                res.route.emplace_back(x->ID);
            return true;
        }
        return false;
    }
    else if(req.cmd == "end")
    {
        mCurPlanMap.Clear();
        mRecord.Reset();
        mCurrentPosition = {0, 0};
    }
    else
    {
        ROS_ERROR("Not such Command: %s",req.cmd.c_str());
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

void Client::SendTrafficInfo(const Coor &v)
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
        ROS_INFO("(%8.3f, %8.3f): In road[%d] with [%d] signals, detection state: [%d]",
                 v.x, v.y, mRecord.curr_rid, signals.size(), detected);
    }
}

void Client::SendGPS(const Coor &v)
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
    Coor curr_pos = mCurrentPosition;
    mLock.unlock();

    SendGPS(curr_pos);
    SendNearPolygonRegion(curr_pos);

    // Check if current plan map covers curr_pos, if not re-plan
    if (!mRecord.is_init)
    {
        if(mCurPlanMap.mRoadPtrs.empty()) return;

        for(int i = 0; i < mCurPlanMap.mRoadPtrs.size() && !mRecord.is_init; ++i)
        {
            if(mCurPlanMap.mRoadPtrs[i]->Cover(curr_pos))
            {
                mRecord.curr_rid = mCurPlanMap.mRoadPtrs[i]->ID;
                mRecord.curr_idx = i;
                if(mRecord.curr_rid+1 == mCurPlanMap.mRoadPtrs.size())
                {
                    mRecord.curr_jid = -1;
                }
                else
                {
                    mRecord.curr_jid = mCurPlanMap.mRoadPtrs[i]->mNextJid;
                }
                if(i + 1 < mCurPlanMap.mRoadPtrs.size())
                {
                    mRecord.next_rid = mCurPlanMap.mRoadPtrs[i + 1]->ID;
                }
                mRecord.is_init = true;
            }
        }

        for(int i = 0; i < mCurPlanMap.mJuncPtrs.size() && !mRecord.is_init; ++i)
        {
            if(mCurPlanMap.mJuncPtrs[i]->Cover(curr_pos))
            {
                mRecord.curr_jid = mCurPlanMap.mJuncPtrs[i]->ID;
                for (int r = 0; r < mCurPlanMap.mRoadPtrs.size(); ++r)
                {
                    if (mCurPlanMap.mRoadPtrs[r]->GetNextJid() == mRecord.curr_jid)
                    {
                        mRecord.curr_idx = r;
                        mRecord.curr_rid = mCurPlanMap.mRoadPtrs[r]->ID;
                        mRecord.next_rid = mCurPlanMap.mRoadPtrs[r + 1]->ID;
                        break;
                    }
                }
                mRecord.is_init = true;
            }
        }

        if(!mRecord.is_init)
        {
            /// not int mCurPlanMap, ask re-plan
            ROS_INFO("(%8.3f, %8.3f) is not in current plan Map, try to re-plan", curr_pos.x, curr_pos.y);

            std::vector<RoadPtr> near_roads;
            std::vector<JuncPtr> near_juncs;
            GetNearRoadPtrs(near_roads,curr_pos, 20);
            GetNearJunctionPtrs(near_juncs,curr_pos, 20);

            if (!near_roads.empty())
            {
                ROS_ERROR("(%8.3f, %8.3f): Current position is out the routing and junction." ,
                          curr_pos.x, curr_pos.y);

                int _cur_road_id = near_roads.front()->ID;
                int _end_road_id = mCurPlanMap.mRoadPtrs.back()->ID;
                if(PlanByCommand("start", {_cur_road_id, _end_road_id}))
                {
                    ROS_INFO("(%8.3f, %8.3f): Re-Plan success, Road[%d] to Road[%d].",
                             curr_pos.x, curr_pos.y,_cur_road_id,_end_road_id);
                }
            }
        }
        return;
    }

    // check if record road or junction covers curr_pos
    bool _is_in_rr = mCurPlanMap.mRoadPtrs[mRecord.curr_idx]->Cover(curr_pos);

    bool _is_in_rj = false;
    if (!_is_in_rr && mRecord.curr_jid != -1)
        _is_in_rj = mCurPlanMap.GetJuncPtrById(mRecord.curr_jid)->Cover(curr_pos);

    if (!_is_in_rr && !_is_in_rj)
    {
        mRecord.is_init = false;
        ROS_ERROR("(%8.3f, %8.3f): Current position is out the routing and junction, "
                  "Re-Plan is ready to execute." , curr_pos.x, curr_pos.y);
    }
    else
    {
        if (_is_in_rr)
            ROS_INFO("(%8.3f, %8.3f): In Road[%d]",
                    curr_pos.x, curr_pos.y,mCurPlanMap.mRoadPtrs[mRecord.curr_idx]->ID);
        else
            ROS_INFO("(%8.3f, %8.3f): In Junction[%d]",
                    curr_pos.x, curr_pos.y,mCurPlanMap.mJuncPtrs[mRecord.curr_jid]->ID);

        SendMap();
        SendTrafficInfo(curr_pos);
        return;
    }
}

bool Client::PlanByCommand(const std::string& method, std::vector<int> argv)
{
    HDMap::srv_route srv;
    srv.request.method = method;
    srv.request.argv = std::move(argv);
    if(mPlanClient.call(srv))
    {
        mCurPlanMap.Clear();
        mRecord.Reset();
        srv.request.method = srv.request.method;
        srv.request.argv = srv.request.argv;
        std::stringstream ss;
        ss << srv.response.route;
        std::cout << ss.str() << std::endl;
        try
        {
            pt::ptree tree;
            pt::read_xml(ss, tree);
            mCurPlanMap.FromXML(tree);
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

void Client::SendNearPolygonRegion(const Coor &v, double radius) {
    assert(radius>=3 && radius<=500);

    static Coor last_v(-1, -1);
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
    if (!must_recalu && (last_v != Coor(-1, -1) && Coor::Distance(v, last_v) < distance_tolerence))
    {
        ROS_INFO("Detected Roads and Junctions are same with last time, region message would not be sent.");
        return;
    }
    last_v = v;

    /// 寻找附近的road和junction
    std::vector<RoadPtr > near_roads;
    std::vector<JuncPtr > near_juncs;
    std::string roads_info_str = GetNearRoadPtrs(near_roads,v,radius);
    std::string juncs_info_str = GetNearJunctionPtrs(near_juncs,v,radius);
    ROS_INFO("(%8.3f, %8.3f): Region: Roads { %s } Junctions { %s }",
             mCurrentPosition.x, mCurrentPosition.y, roads_info_str.c_str(), juncs_info_str.c_str());

    std::vector<IGeometry*> _geometries;
    for (const auto& _rptr: near_roads)
    {
        cur_road_ids.insert(_rptr->ID);
        _geometries.emplace_back(_rptr.get());
    }
    for (const auto& _jptr: near_juncs)
    {
        cur_junc_ids.insert(_jptr->ID);
        _geometries.emplace_back(_jptr.get());
    }

    if(!must_recalu && (last_junc_ids==cur_junc_ids && last_road_ids==cur_road_ids))
    {
        ROS_INFO("Detected Roads and Junctions are same with last time, region message would not be sent.");
        return;
    }
    last_junc_ids = cur_junc_ids;
    last_road_ids = cur_road_ids;


    HDMap::msg_route_region mrr;
    mrr.header.frame_id = "hdmap";
    mrr.header.stamp = ros::Time::now();

    for(const auto& geo: _geometries)
    {
        geometry_msgs::Polygon pg;
        for (const auto & _pose: geo->GetRegionPoses())
        {
            geometry_msgs::Point32 p;
            p.x = static_cast<float>(_pose.x);
            p.y = static_cast<float>(_pose.y);
            p.z = static_cast<float>(_pose.GetAngle().Value());
            pg.points.push_back(p);
        }
        mrr.polygons.emplace_back(pg);
    }

    mPubRouteRegion.publish(mrr);
}

std::string Client::GetNearRoadPtrs(std::vector<RoadPtr>& near_roads, const Coor &coor, double distance)
{
    std::string roads_info_str;

    HDMap::srv_map_data srv;
    srv.request.type = "RoadByPos";
    srv.request.argv.emplace_back(coor.x);
    srv.request.argv.emplace_back(coor.y);
    srv.request.argv.emplace_back(distance);

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
                roads_info_str += std::to_string(near_roads.back()->ID) + " ";
            }
        }
        catch (...){
            ROS_INFO("Not receive any roads or something error happen.");
        }
    }

    if (roads_info_str.empty()) roads_info_str ="none";

    return roads_info_str;
}

std::string Client::GetNearJunctionPtrs(std::vector<JuncPtr>& near_juncs, const Coor &coor, double distance)
{
    std::string juncs_info_str;

    HDMap::srv_map_data srv;
    srv.request.type = "JunctionByPos";
    srv.request.argv.emplace_back(coor.x);
    srv.request.argv.emplace_back(coor.y);
    srv.request.argv.emplace_back(distance);

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
                juncs_info_str += std::to_string(near_juncs.back()->ID) + " ";
            }
        }
        catch (...){
            ROS_INFO("Not receive any junctions or something error happen.");
        }
    }

    if (juncs_info_str.empty()) juncs_info_str ="none";

    return juncs_info_str;
}
