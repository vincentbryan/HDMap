#include <utility>

//
// Created by vincent on 18-10-24.
//

#include <boost/property_tree/ptree.hpp>
#include <Tool/Planner.h>

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

        if (p_curr->ID == p_end->ID) {
            break;
        }

        for(auto& x: mHDMapPtr->AdjacentRoadInfo(p_curr))
        {
            if (!_is_visit[x->ID])
            {
                road_pre[x->ID] = p_curr;
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

void Planner::DFS(RoadPtr p_curr, std::vector<RoadPtr> v)
{
    if (p_curr->ID == pEnd->ID)
    {
        mAllRouting.emplace_back(v);
    }
    else
    {
        for(auto & x : mHDMapPtr->AdjacentRoadInfo(p_curr))
        {
            if (!is_visited[x->ID])
            {
                is_visited[x->ID] = true;
                v.emplace_back(x);
                DFS(x, v);
                v.pop_back();
                is_visited[x->ID] = false;
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
            mHDMapPtr->GetJuncPtrById(jid)->GetRoadLink(mRouting[i]->ID, mRouting[i + 1]->ID).Send(*mSenderPtr);
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
            auto junc = mHDMapPtr->JuncPtrs[jid];
            p_junc.add("<xmlattr>.id", junc->ID);

            pt::ptree p_vec;
            pt::ptree p_v;

            for (auto &bezier: junc->GetBoundaryCurves())
            {
                for (auto &p : bezier.GetParam())
                    p_v.add("param", p);
                p_vec.add_child("bezier", p_v);
                p_v.clear();
            }
            p_junc.add_child("regionBoundary", p_vec);

            auto road_link = mHDMapPtr->GetJuncPtrById(jid)->GetRoadLink(mRouting[i]->ID, mRouting[i + 1]->ID);
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
            mRouting.clear();
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
                        is_visited[x->ID] = true;
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
        ROS_INFO_STREAM("\t" << x->ID);
    }
    ToXML(res.route);
    Send();
    return true;
}
