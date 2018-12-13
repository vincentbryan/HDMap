//
// Created by vincent on 18-10-24.
//

#ifndef HDMAP_PLANNER_H
#define HDMAP_PLANNER_H

#include "Type/Map.h"
#include <HDMap/srv_route.h>
#include <HDMap/srv_routeRequest.h>
#include <HDMap/srv_routeResponse.h>

namespace hdmap
{
class Planner
{
private:



    std::shared_ptr<Map> mHDMapPtr;
    Coor mStartPoint;
    Coor mEndPoint;
    RoadPtr pStart;
    RoadPtr pEnd;

    std::map<unsigned int, bool> is_visited;
    std::vector<RoadPtr> mRouting;
    std::vector<std::vector<RoadPtr>> mAllRouting;

    std::shared_ptr<Sender> mSenderPtr;

public:
    explicit Planner(std::shared_ptr<Map> _map, std::shared_ptr<Sender> _sender);
    void PlanUsingSearch();
    void Send();
    void ToXML(std::string &str);
    bool OnRequest(HDMap::srv_route::Request & req, HDMap::srv_route::Response & res);

private:
    void DFS(RoadPtr p_curr, std::vector<RoadPtr> v);
    void BFS(RoadPtr p_start, RoadPtr p_end, std::vector<RoadPtr> &trace);
    void Evaluate();
};
}
#endif //HDMAP_PLANNER_H
