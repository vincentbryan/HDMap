#include <utility>
#include <boost/property_tree/ptree.hpp>
#include <Tool/Planner.h>

using namespace hdmap;
namespace pt = boost::property_tree;

Planner::Planner(MapPtr _map, std::shared_ptr<Sender> _sender) :
    mMapPtr(_map),
    mSenderPtr(std::move(_sender))
{}

void Planner::BFS(RoadPtr p_start, RoadPtr p_end, std::vector<RoadPtr> &trace) {

    // find the shortest road

    std::queue<RoadPtr> road_queue;
    road_queue.push(p_start);
    RoadPtr road_pre[mMapPtr->GetRoadSize()];
    std::unordered_map<unsigned int,bool> _is_visit;
    std::fill(road_pre,road_pre+mMapPtr->GetRoadSize(), nullptr);

    while(!road_queue.empty()){
        RoadPtr r_curr = road_queue.front();
        road_queue.pop();

        if (r_curr->ID == p_end->ID) {
            break;
        }

        for(auto& x: mMapPtr->AdjacentRoadInfo(r_curr))
        {
            if (!_is_visit[x->ID] && !this->is_visited[x->ID])
            {
                road_pre[x->ID] = r_curr;
                _is_visit[x->ID] = true;
                road_queue.push(x);
            }
        }
    }
    if (road_pre[p_end->ID] == nullptr) {
        return;
    }
    // back trace
    trace.push_back(p_end);
    while (trace.back()->ID != p_start->ID) {
        trace.push_back(road_pre[trace.back()->ID]);
    }
    std::reverse(trace.begin(),trace.end());
}


void Planner::Send()
{
    auto m1 = mSenderPtr->GetCone(mStartPoint, 0.0, 1.0, 1.0, 1.0, 3.0);
    auto m2 = mSenderPtr->GetCone(mEndPoint, 1.0, 1.0, 0.0, 1.0, 3.0);
    mSenderPtr->array.markers.emplace_back(m1);
    mSenderPtr->array.markers.emplace_back(m2);
    mSenderPtr->Send();

    for(int i = 0; i < mRouting.size(); i++)
    {
        mRouting[i]->OnSend(*mSenderPtr);

        if(i + 1 < mRouting.size())
        {
            int jid = mRouting[i]->mNextJid;
            mMapPtr->GetJuncPtrById(jid)->GetRoadLink(mRouting[i]->ID, mRouting[i + 1]->ID).OnSend(*mSenderPtr);
        }
    }
}

void Planner::ToXML(std::string &str)
{
    try
    {
        pt::ptree tree;

        for(auto &  mRoadPtr: mRouting)
        {
            tree.add_child("hdmap.roads.road", mRoadPtr->ToXML());
            auto _JunPtr = mMapPtr->GetJuncPtrById(mRoadPtr->mNextJid);
            if (_JunPtr)
            {
                tree.add_child("hdmap.junctions.junction", _JunPtr->ToXML());
            }

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
    static std::set<std::string> _valid_cmd {"r2r","p2p"};

    std::map<unsigned int, bool>().swap(is_visited);

    if(_valid_cmd.count(req.method) == 0)
    {
        ROS_ERROR("[OnRequest] Invalid request method: %s", req.method.c_str());
        return false;
    }
    
    if(req.method == "r2r")
    {
        std::vector<int> _road_vec;
        for (const auto& _r: req.argv)
        {
            _road_vec.emplace_back(int(_r));
        }
        PlanRouteByRoadIDs(_road_vec);
    }
    else if (req.method == "p2p" )
    {
        if (req.argv.size()%3 == 0)
        {
            std::vector<Pose> _tmp_pose;
            for(int i = 0; i < req.argv.size(); i+=3)
            {
                _tmp_pose.emplace_back(req.argv[i], req.argv[i+1], req.argv[i+2]);
            }

            PlanRouteByPoses(_tmp_pose);
        }
        else ROS_ERROR_STREAM("[OnRequest] PlanRouteByPoese receive even number of data.");
    }

    if(mRouting.empty())
    {
        ROS_ERROR_STREAM("[OnRequest] Cannot find a property road, please check the map again.");
        return false;
    }

    ROS_INFO_STREAM("Result:");
    for(auto & x : mRouting)
    {
        ROS_INFO_STREAM("\t" << x->ID);
    }
    ToXML(res.route);
    Send();
    return true;
}

bool Planner::PlanRouteByPoses(std::vector<Pose> poses) {

    if( poses.size() == 2)
    {
        Coor _begin_coor = poses.front().GetPosition();

        mRouting.clear();

        // naive consideration.
        auto _end_road_ptr = mMapPtr->GetRoadPtrByDistance(poses.back().GetPosition(), 0, true).front();

        /// consider lane in start point.
        auto res = mMapPtr->GetLaneInfoByPose(poses.front());

        RoadPtr _cur_road_ptr    = std::get<0>(res);
        SecPtr  _sec_ptr         = std::get<1>(res);
        int     _start_lane_id   = std::get<2>(res);

        if ( _start_lane_id == -1)
        {
            ROS_INFO("PlanRouteByPoses: Cannot match location (%4.3f, %4.3f) to any road. Try to find junction",
                      _begin_coor.x,_begin_coor.y);

            // try to locate in junction and fit a road link.
            auto _res = mMapPtr->GetRoadLinkByPose(poses.front());

            JuncPtr   _cur_junc_ptr  = std::get<0>(_res);
            RoadLink& _cur_road_link = std::get<1>(_res);
            int      _cur_link_index = std::get<2>(_res);

            if (_cur_link_index == -1)
            {
                ROS_INFO("PlanRouteByPoses: Cannot match location (%4.3f, %4.3f) to any junction.",
                         _begin_coor.x,_begin_coor.y);
                return false;
            }

            // convert p2p problem to r2p
            return PlanRouteByRoadIDs(
                {
                    int(_cur_road_link.mFromRoadId),
                    int(_cur_road_link.mToRoadId),
                    int(_end_road_ptr->ID)
                });

        }

        assert(_cur_road_ptr!= nullptr);
        assert(_start_lane_id != -1);

        // select suit next road, just mark which is not to want to walk.
        JuncPtr _turn_junc_ptr = mMapPtr->GetJuncPtrById(_cur_road_ptr->mNextJid);
        if (_turn_junc_ptr)
        {
            auto _adjacent_roads = mMapPtr->AdjacentRoadInfo(_cur_road_ptr);
            for (auto& rl: _turn_junc_ptr->RoadLinks)
            {
                if ( _cur_road_ptr->ID == rl.first.first )
                {
                    bool _mark_done = false;
                    for(auto & _lane_link: rl.second.mLaneLinks)
                    {
                        if (_lane_link.mFromLaneIndex == _start_lane_id)
                        {
                            for (auto& rptr: _adjacent_roads)
                            {
                                if(rptr->ID != rl.second.mToRoadId)
                                {
                                    is_visited[rptr->ID] = true;
                                }
                            }
                            _mark_done = true;
                            break;
                        }
                    }
                    if(_mark_done) break;
                }
            }
        }

        return PlanRouteByRoadIDs({int(_cur_road_ptr->ID), int(_end_road_ptr->ID)});
    }
    else if(poses.size() > 2)
    {
        ROS_INFO("[OnRequest] PlanRouteByPoses: Not support for multiple target point yet.");
        return false;
    }
    else
    {
        ROS_ERROR("[OnRequest] PlanRouteByPoses: cors.size() < 2 is not valid.");
        return false;
    }
}

bool Planner::PlanRouteByRoadIDs(std::vector<int> ids) {
    if (ids.size()==2 && ids[0]==ids[1])
    {
        mRouting.clear();
        mRouting.push_back(mMapPtr->GetRoadPtrById(ids[0]));
        ROS_INFO("[OnRequest] Walk on the same Road %d method: PlanRouteByRoadIDs", ids.front());
    }
    else if(ids.size() >= 2 )
    {
        ROS_INFO("[OnRequest] road[%d] --> road[%d] method: PlanRouteByRoadIDs", ids.front(), ids.back());
        std::vector<RoadPtr>().swap(mRouting);
        for(int i=0;i<ids.size()-1;++i){
            std::vector<RoadPtr> trace;
            pStart = mMapPtr->GetRoadPtrById(ids[i]);
            pEnd   = mMapPtr->GetRoadPtrById(ids[i+1]);
            BFS(pStart,pEnd,trace);
            if (trace.empty()) break;
            else
            {
                if (!mRouting.empty()){
                    mRouting.pop_back();
                }
                for(auto&x:trace){
                    // is_visited[x->ID] = true;
                    mRouting.emplace_back(x);
                }
            }
        }
    }
    else
    {
        ROS_ERROR_STREAM("PlanRouteByRoadIDs: Invalid request");
        return false;
    }
    return true;
}
