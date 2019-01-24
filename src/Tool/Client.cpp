//
// Created by vincent on 18-10-31.
//
#include <HDMap/msg_signal_list.h>
#include <HDMap/msg_route_region.h>
#include <HDMap/msg_route_info.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <HDMap/srv_route.h>
#include <HDMap/srv_map_data.h>
#include <unordered_set>
#include <tf/tf.h>

#include "Tool/Client.h"


using namespace hdmap;

Client::Client(ros::NodeHandle & n)
{
    ros::Publisher pub = n.advertise<visualization_msgs::MarkerArray>("/HDMap", 10);
    std::shared_ptr<Sender> p_sender(new Sender(pub));
    mCurPlanMap.SetSender(p_sender);

    mPlanClient = n.serviceClient<HDMap::srv_route>("/map_plan_service");
    mDataClient = n.serviceClient<HDMap::srv_map_data>("/map_data_service");

    mPubRouteRegion = n.advertise<HDMap::msg_route_region>("/map_pub_route_region",1);
    mPubTrafficLight = n.advertise<HDMap::msg_signal_list>("/map_to_traffic_light", 1);
    mPubVIZ = n.advertise<visualization_msgs::Marker>("/map_to_rviz", 10);
    mPubPlanner = n.advertise<std_msgs::String>("/map_to_planner", 1);
    mPubPointCloud = n.advertise<sensor_msgs::PointCloud2>("/map_pub_pointcloud", 5, true);
    mPubRouteInfo = n.advertise<HDMap::msg_route_info>("/map_pub_route_info", 5, true);

    mOdomSubscribe = n.subscribe("/odom", 1, &Client::OdometryCallBack, this);
    mLocaSubscribe = n.subscribe("/Location", 1, &Client::LocationCallBack, this);
    mServer = n.advertiseService("/map_command", &Client::OnCommandRequest, this);

    mCurrentPose = Pose();
}

void Client::OdometryCallBack(const nav_msgs::Odometry &msg)
{
    std::lock_guard<std::mutex> lock(mLock);
    const auto& _position = msg.pose.pose.position;
    geometry_msgs::Quaternion orientation = msg.pose.pose.orientation;
    tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
    double _yaw, _pitch, _roll;
    mat.getEulerYPR(_yaw, _pitch, _roll);
    mCurrentPose = {_position.x, _position.y, 90.0 + _yaw*180.0/TFSIMD_PI};
}


void Client::LocationCallBack(const nox_msgs::Location &msg)
{
    std::lock_guard<std::mutex> lock(mLock);
    mCurrentPose = {msg.x, msg.y, 90.0 + msg.yaw};
}


bool Client::OnCommandRequest(HDMap::srv_map_cmd::Request &req, HDMap::srv_map_cmd::Response &res)
{
    mRecord.Reset();
    if(req.cmd == "r2r" || req.cmd == "p2p")
    {
        mRecord.plan_method = req.cmd;
        // temporary adjust for that target pose's angle is zero

        if(PlanByCommand(req.cmd, req.argv))
        {
            res.suss = 1;

            if (req.cmd == "p2p")
            {
                mRecord.start_pose  = {req.argv[0], req.argv[1], req.argv[2]};
                mRecord.target_pose = {req.argv[3], req.argv[4], req.argv[5]};
            }

            return true;
        }
        else res.suss = 0;
        ROS_ERROR("OnCommandRequest: request fail, for more details please log in server.");
        return false;
    }
    else if(req.cmd == "clear")
    {
        mCurPlanMap.Clear();
        mCurrentPose = {0, 0, 0};
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
    if(mRecord.curr_idx != -1)
    {
        auto m = mCurPlanMap.RoadPtrs[mRecord.curr_idx];
        if(m != nullptr)
            p_map.add_child("hdmap.roads.road", m->ToXML());
    }
    if(mRecord.next_idx != -1)
    {
        auto m = mCurPlanMap.RoadPtrs[mRecord.next_idx];
        if(m != nullptr)
            p_map.add_child("hdmap.roads.road", m->ToXML());
    }
    if(mRecord.curr_jid != -1 && mRecord.curr_rid!= -1 && mRecord.next_idx !=-1)
    {
        auto m = mCurPlanMap.GetJuncPtrById(mRecord.curr_jid);
        if(m != nullptr)
        {
            pt::ptree p_junc;
            p_junc.add("<xmlattr>.id", m->ID);

            pt::ptree p_vec;
            pt::ptree p_v;

            for (auto &bezier: m->GetBoundaryCurves())
            {
                for (auto &p : bezier.GetParam())
                    p_v.add("param", p);
                p_vec.add_child("bezier", p_v);
                p_v.clear();
            }
            p_junc.add_child("regionBoundary", p_vec);

            auto road_link = m->GetRoadLink(mRecord.curr_rid, mCurPlanMap.RoadPtrs[mRecord.next_idx]->ID);
            p_junc.add_child("roadLink", road_link.ToXML());
            p_map.add_child("hdmap.junctions.junction", p_junc);
            p_junc.clear();
        }
    }

    if(mRecord.target_pose.x != 0.0 && mRecord.target_pose.y != 0.0)
    {
        pt::ptree p_junc;
        p_junc.add("param",mRecord.target_pose.x);
        p_junc.add("param",mRecord.target_pose.y);
        p_map.add_child("hdmap.route_info.target", p_junc);
    }

    std::stringstream ss;
    pt::write_xml(ss, p_map);
    std_msgs::String s;
    s.data = ss.str();
    mPubPlanner.publish(s);
}

void Client::SendTrafficInfo(const Coor &v, RoadPtr target_road)
{
    std::vector<SigPtr> signals;
    if (mRecord.curr_rid!=-1) signals = mCurPlanMap.RoadPtrs[mRecord.curr_idx]->GetSignals();
    if (target_road) signals = target_road->GetSignals();

    if(!signals.empty())
    {
        int cnt = 0;
        for(auto & s : signals)
        {
            double t = sqrt((v.x-s->x)*(v.x-s->x) + (v.y-s->y)*(v.y-s->y));
            if(t < 135) cnt++;
        }

        HDMap::msg_signal_list ss;

        ss.header.frame_id = mCurPlanMap.pSender->frame_id;
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
        ROS_INFO("(%8.3f, %8.3f): In road[%d] with [%ld] signals, detection state: [%d]",
                 v.x, v.y, mRecord.curr_rid, signals.size(), detected);
    }
}

void Client::SendGPS(const Coor &v)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = mCurPlanMap.pSender->frame_id;
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
    Pose curr_pose = mCurrentPose;
    mLock.unlock();

    SendGPS({curr_pose.x,curr_pose.y});
    SendNearPolygonRegion({curr_pose.x,curr_pose.y});
    SendPointCloud({curr_pose.x, curr_pose.y}, 50);

    // Check if current plan map covers curr_pose, if not re-plan
    if (!mRecord.is_init)
    {
        if(mCurPlanMap.RoadPtrs.empty()) return;

        for(int i = 0; i < mCurPlanMap.RoadPtrs.size() && !mRecord.is_init; ++i)
        {
            if(mCurPlanMap.RoadPtrs[i]->IsCover({curr_pose.x,curr_pose.y}))
            {
                mRecord.curr_rid = mCurPlanMap.RoadPtrs[i]->ID;
                mRecord.curr_idx = i;

                if(mRecord.curr_idx+1 == mCurPlanMap.RoadPtrs.size())
                {
                    mRecord.curr_jid = -1;
                }
                else
                {
                    mRecord.curr_jid = mCurPlanMap.RoadPtrs[i]->mNextJid;
                }

                if(i + 1 < mCurPlanMap.RoadPtrs.size())
                {
                    mRecord.next_idx = i + 1;
                }
                else
                {
                    mRecord.next_idx = -1;
                }
                mRecord.is_init = true;
            }
        }

        for(int i = 0; i < mCurPlanMap.JuncPtrs.size() && !mRecord.is_init; ++i)
        {
            if(mCurPlanMap.JuncPtrs[i]->IsCover({curr_pose.x,curr_pose.y}))
            {
                int _fitnest_road_r = -1;
                double _min_diff = 361;
                mRecord.curr_jid = mCurPlanMap.JuncPtrs[i]->ID;
                for (int r = 0; r < mCurPlanMap.RoadPtrs.size(); ++r)
                {
                    auto& _road_ptr = mCurPlanMap.RoadPtrs[r];

                    if (_road_ptr->mNextJid == mRecord.curr_jid)
                    {
                        Angle _diff_angle = _road_ptr->GetReferenceLinePoses().back().GetAngle()-curr_pose.GetAngle();

                        Angle::Wrap(_diff_angle, -180, 180);

                        double _diff_angle_value = abs(_diff_angle.Value());

                        if(_diff_angle_value < _min_diff)
                        {
                            _min_diff = _diff_angle_value;
                            _fitnest_road_r = r;
                        }
                    }
                }
                mRecord.curr_idx = _fitnest_road_r;
                mRecord.curr_rid = mCurPlanMap.RoadPtrs[_fitnest_road_r]->ID;
                if( _fitnest_road_r + 1 < mCurPlanMap.RoadPtrs.size())
                {
                    mRecord.next_idx = _fitnest_road_r+1;
                }
                else
                {
                    mRecord.next_idx = -1;
                }

                mRecord.is_init = true;
            }
        }

        if(!mRecord.is_init)
        {
            /// not int mCurPlanMap, ask re-plan
            RePlanRoute(curr_pose);
        }
        return;
    }


    const double _MIN_DIS = 0.5;
    bool _is_in_cur_road = false, _is_in_cur_junc = false, _is_in_next_road = false;
    if(mRecord.curr_idx!=-1)
        _is_in_cur_road = mCurPlanMap.RoadPtrs[mRecord.curr_idx]->GetDistanceFromCoor({curr_pose.x,curr_pose.y}) < _MIN_DIS;
    
    if (!_is_in_cur_road && mRecord.curr_jid != -1)
        _is_in_cur_junc = mCurPlanMap.GetJuncPtrById(mRecord.curr_jid)->IsCover({curr_pose.x,curr_pose.y});

    if(!_is_in_cur_road && !_is_in_cur_junc && mRecord.next_idx != -1)
    {
        _is_in_next_road = mCurPlanMap.RoadPtrs[mRecord.next_idx]->GetDistanceFromCoor({curr_pose.x,curr_pose.y}) < _MIN_DIS;
        if (_is_in_next_road)
        {
            mRecord.curr_rid = mCurPlanMap.RoadPtrs[mRecord.next_idx]->ID;
            mRecord.curr_idx = mRecord.next_idx;

            if(mRecord.curr_idx+1 == mCurPlanMap.RoadPtrs.size())
            {
                mRecord.curr_jid = -1;
            }
            else
            {
                mRecord.curr_jid = mCurPlanMap.RoadPtrs[mRecord.curr_idx]->mNextJid;
            }

            if(mRecord.curr_idx + 1 < mCurPlanMap.RoadPtrs.size())
            {
                mRecord.next_idx = mRecord.curr_idx + 1;
            }
            else
            {
                mRecord.next_idx = -1;
            }
        }
    }

    if (!_is_in_cur_road && !_is_in_cur_junc && !_is_in_next_road)
    {
        mRecord.is_init = false;
        ROS_ERROR("(%8.3f, %8.3f): Current position is out route, "
                  "local map search is ready to execute." , curr_pose.x, curr_pose.y);
    }
    else
    {
        if(_is_in_cur_junc)
        {
            ROS_INFO("(%8.3f, %8.3f): In Junction[%d]",
                     curr_pose.x, curr_pose.y,mCurPlanMap.GetJuncPtrById(mRecord.curr_jid)->ID);
        }
        else
        {
            ROS_INFO("(%8.3f, %8.3f): In Road[%d]",
                     curr_pose.x, curr_pose.y,mCurPlanMap.RoadPtrs[mRecord.curr_idx]->ID);
        }

        SendMap();
        SendTrafficInfo({curr_pose.x,curr_pose.y});
        return;
    }
}

bool Client::PlanByCommand(const std::string& method, std::vector<double> argv)
{
    HDMap::srv_route srv;
    srv.request.method = method;
    srv.request.argv = std::move(argv);
    if(mPlanClient.call(srv))
    {
        mCurPlanMap.Clear();
        std::stringstream ss;
        ss << srv.response.route;
        try
        {
            pt::ptree tree;
            pt::read_xml(ss, tree);
            mCurPlanMap.FromXML(tree);

            HDMap::msg_route_info _route_info;
            for(auto& rptr: mCurPlanMap.RoadPtrs)
            {
                _route_info.route_id.emplace_back(rptr->ID);
                const std::vector<hdmap::Pose>& _r_res = rptr->GetReferenceLinePoses();

                int _idx = 0;
                while(true)
                {
                    const Pose& _p = _r_res[_idx];
                    _route_info.route_pose.emplace_back(_p.x);
                    _route_info.route_pose.emplace_back(_p.y);
                    _route_info.route_pose.emplace_back(_p.GetAngle().Value());

                    _idx += 20;

                    if (_idx >= _r_res.size())
                    {
                        _route_info.route_pose.emplace_back(_r_res.back().x);
                        _route_info.route_pose.emplace_back(_r_res.back().y);
                        _route_info.route_pose.emplace_back(_r_res.back().GetAngle().Value());
                        break;
                    }
                }

            }
            mPubRouteInfo.publish(_route_info);

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

void Client::SendNearPolygonRegion(const Coor &v, double radius) 
{
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
             mCurrentPose.x, mCurrentPose.y, roads_info_str.c_str(), juncs_info_str.c_str());

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
    mrr.header.frame_id = mCurPlanMap.pSender->frame_id;
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

bool Client::RePlanRoute(const Pose& cur_pose)
{
    const Coor& cur = cur_pose.GetPosition();

    ROS_INFO("(%8.3f, %8.3f) is not in current local Map, try to re-plan", cur_pose.x, cur_pose.y);

    if(mRecord.plan_method == "undefined")
    {
        ROS_INFO("(%8.3f, %8.3f) Not define plan method, skip re plan", cur_pose.x, cur_pose.y);
        return false;
    }
    else if (mRecord.plan_method == "p2p"){
        if(PlanByCommand("p2p", {cur_pose.x, cur_pose.y,cur_pose.GetAngle().Value(),
                                 mRecord.target_pose.x, mRecord.target_pose.y, mRecord.target_pose.GetAngle().Value()}))
        {
            ROS_INFO("(%8.3f, %8.3f): Re-Plan success, (%3.2f, %3.2f, %3.2f) to (%3.2f, %3.2f, %3.2f).",
                     cur_pose.x, cur_pose.y,cur_pose.x, cur_pose.y,cur_pose.GetAngle().Value(),
                     mRecord.target_pose.x, mRecord.target_pose.y,  mRecord.target_pose.GetAngle().Value());
        }
        else return false;
    }
    else if (mRecord.plan_method == "r2r")
    {
        std::vector<RoadPtr> near_roads;
        std::vector<JuncPtr> near_juncs;

        GetNearRoadPtrs(near_roads, cur, 20);
        GetNearJunctionPtrs(near_juncs, cur, 2);

        if (!near_roads.empty())
        {
            if(!near_roads.front()->IsCover(cur))
            {
                if (!near_juncs.empty())
                {
                    if(!near_juncs.front()->IsCover(cur))
                    {
                        // current position is not in any junctions or roads.
                        ROS_ERROR("(%8.3f, %8.3f): Current position is out the routing and junction." ,
                                  cur.x, cur.y);
                        return false;
                    }
                    else
                    {
                        // current position is in junction, add to local map if it is not in.
                        mCurPlanMap.AddJunctionFromPtr (near_juncs[0]);
                        ROS_ERROR("(%8.3f, %8.3f): Current position is in junction[%d]." ,
                                  cur.x, cur.y, near_juncs.front()->ID);
                    }
                }
            }
            else
            {
                ROS_ERROR("(%8.3f, %8.3f): Current position is in road[%d]." ,
                          cur.x, cur.y, near_roads.front()->ID);
            }

            int _cur_road_id = near_roads.front()->ID;
            int _end_road_id = mCurPlanMap.RoadPtrs.back()->ID;
            if(PlanByCommand("r2r", {double(_cur_road_id), double(_end_road_id)}))
            {
                ROS_INFO("(%8.3f, %8.3f): Re-Plan success, Road[%d] to Road[%d].",
                         cur.x, cur.y,_cur_road_id,_end_road_id);
            }
            else
            {
                return false;
            }
        }
    }

    return true;
}

void Client::SendPointCloud(const Coor& v, const double& distance)
{

    int subscribe_num = mPubPointCloud.getNumSubscribers();
    if( subscribe_num == 0 || mLocalPointCloudPtr == nullptr )
    {
        return;
    }
    else
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr _pub_cloud;
        if (v.x == -1 && v.y == -1)
        {
            _pub_cloud = mLocalPointCloudPtr;
        }
        _pub_cloud->header.frame_id= mCurPlanMap.pSender->frame_id;
        pcl_conversions::toPCL(ros::Time::now(), _pub_cloud->header.stamp);

        pcl::PointXYZI searchPoint;
        searchPoint.x= static_cast<float>(v.x);
        searchPoint.y= static_cast<float>(v.y);

        std::vector<int> _indices;
        std::vector<float> _distance;
        mKdTree.radiusSearch(searchPoint, distance, _indices, _distance);

        pcl::copyPointCloud(*mLocalPointCloudPtr, _indices, *_pub_cloud);
        mPubPointCloud.publish(_pub_cloud);
    }
}

void Client::SetInputPointCloud(const std::string & path)
{
    if (mLocalPointCloudPtr == nullptr)
    {
        mLocalPointCloudPtr.reset(new pcl::PointCloud<pcl::PointXYZI>());
        mLocalPointCloudPtr->header.frame_id = mCurPlanMap.pSender->frame_id;
        pcl::io::loadPCDFile(path, *mLocalPointCloudPtr);
        if(!mLocalPointCloudPtr->empty())
        {
            mKdTree.setInputCloud(mLocalPointCloudPtr);
        }
    }
}
