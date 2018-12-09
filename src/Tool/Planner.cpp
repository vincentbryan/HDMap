#include <utility>

//
// Created by vincent on 18-10-24.
//

#include <boost/property_tree/ptree.hpp>
#include <Tool/Planner.h>
#include <queue>

using namespace hdmap;
namespace pt = boost::property_tree;

Planner::Planner(std::shared_ptr<Map> _map, std::shared_ptr<Sender> _sender) :
    mHDMapPtr(std::move(_map)),
    mSenderPtr(std::move(_sender))
{}

void Planner::PlanUsingSearch()
{
    // mAllRouting.clear();
    // mRouting.clear();
    // is_visited.clear();

    std::vector<RoadPtr>().swap(mRouting);
    std::vector<std::vector<RoadPtr>>().swap(mAllRouting);
    std::map<unsigned int, bool>().swap(is_visited);

    std::vector<RoadPtr> v;
    v.emplace_back(pStart);
    DFS(pStart, v);
    Evaluate();
}

void Planner::BFS(RoadPtr p_start, RoadPtr p_end, std::vector<RoadPtr> &trace) {

    // find the shortest road

    std::queue<RoadPtr> road_queue;
    road_queue.push(p_start);
    RoadPtr road_pre[mHDMapPtr->GetRoadSize()];
    std::unordered_map<unsigned int,bool> _is_visit;
    std::fill(road_pre,road_pre+mHDMapPtr->GetRoadSize(), nullptr);

    while(!road_queue.empty()){
        RoadPtr p_curr = road_queue.front();
        road_queue.pop();

        if (p_curr->mRoadId == p_end->mRoadId){
            break;
        }

        for(auto& x: mHDMapPtr->AdjacentRoadInfo(p_curr))
        {
            if (!_is_visit[x->mRoadId])
            {
                road_pre[x->mRoadId] = p_curr;
                _is_visit[x->mRoadId] = true;
                road_queue.push(x);
            }
        }
    }
    if (road_pre[p_end->mRoadId] == nullptr){
        return;
    }
    // back trace
    trace.push_back(p_end);
    while(trace.back()->mRoadId!=p_start->mRoadId){
        trace.push_back(road_pre[trace.back()->mRoadId]);
    }
    std::reverse(trace.begin(),trace.end());
}

void Planner::DFS(RoadPtr p_curr, std::vector<RoadPtr> v)
{
    if(p_curr->mRoadId == pEnd->mRoadId)
    {
        mAllRouting.emplace_back(v);
    }
    else
    {
        for(auto & x : mHDMapPtr->AdjacentRoadInfo(p_curr))
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
    auto m1 = mSenderPtr->GetCone(mStartPoint, 0.0, 1.0, 1.0, 1.0, 3.0);
    auto m2 = mSenderPtr->GetCone(mEndPoint, 1.0, 1.0, 0.0, 1.0, 3.0);
    mSenderPtr->array.markers.emplace_back(m1);
    mSenderPtr->array.markers.emplace_back(m2);
    mSenderPtr->Send();

    for(int i = 0; i < mRouting.size(); i++)
    {
        mRouting[i]->Send(*mSenderPtr);

        if(i + 1 < mRouting.size())
        {
            int jid = mRouting[i]->mNextJid;
            mHDMapPtr->GetJuncPtrById(jid)->GetRoadLink(mRouting[i]->mRoadId, mRouting[i+1]->mRoadId).Send(*mSenderPtr);
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
            auto junc = mHDMapPtr->mJuncPtrs[jid];
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

            pt::ptree p_region_vec;
            p_v.clear();
            for (auto &v : junc->mRegionPoses)
            {
                p_v.add("<xmlattr>.x", v.x);
                p_v.add("<xmlattr>.y", v.y);
                p_region_vec.add_child("vertex", p_v);
                p_v.clear();
            }

            p_junc.add_child("vertice", p_vec);
            p_junc.add_child("regionVertices", p_region_vec);

            auto road_link =  mHDMapPtr->GetJuncPtrById(jid)->GetRoadLink(mRouting[i]->mRoadId, mRouting[i+1]->mRoadId);
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
    if(req.method == "start")
    {
        if(req.argv.size()==2 && req.argv[0]==req.argv[1])
        {
            mRouting.push_back(mHDMapPtr->GetRoadPtrById(req.argv[0]));
            ROS_INFO("Request: Walk on the same Road %d method: %s", req.argv.front(), req.method.c_str());
        }
        else if(req.argv.size() >= 2 )
        {
            ROS_INFO("Request: road[%d] --> road[%d] method: %s", req.argv.front(), req.argv.back(), req.method.c_str());
            std::vector<RoadPtr>().swap(mRouting);
            std::map<unsigned int, bool>().swap(is_visited);
            for(int i=0;i<req.argv.size()-1;++i){
                std::vector<RoadPtr> trace;
                pStart = mHDMapPtr->GetRoadPtrById(req.argv[i]);
                pEnd   = mHDMapPtr->GetRoadPtrById(req.argv[i+1]);
                BFS(pStart,pEnd,trace);
                if (trace.empty()) break;
                else{
                    if (!mRouting.empty()){
                        mRouting.pop_back();
                    }
                    for(auto&x:trace){
                        is_visited[x->mRoadId] = true;
                        mRouting.emplace_back(x);
                    }
                }
            }

        }
        else
        {
            ROS_ERROR_STREAM("Invalid request: argv.size() != 2");
            return false;
        }
    }

    if(mRouting.empty())
    {
        ROS_ERROR_STREAM("Result: Cannot find a property road, please check the map again.");
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

