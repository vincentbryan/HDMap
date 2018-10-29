//
// Created by vincent on 18-10-24.
//

#ifndef HDMAP_PLANNER_H
#define HDMAP_PLANNER_H

#include "../Type/Map.h"

namespace hdmap
{
class Planner
{
    using SubRoadPtr = std::shared_ptr<SubRoad>;
private:
    Map mHDMap;
    Vector2d mStartPoint;
    Vector2d mEndPoint;

    std::map<unsigned int, bool> is_visited;

    std::vector<SubRoadPtr> mRouting;
    std::vector<std::vector<SubRoadPtr>> mAllRouting;

    SubRoadPtr pStart;
    SubRoadPtr pEnd;

    struct
    {
        int curr_idx = -1;
        int curr_rid = -1;
        int curr_dir =  0;
        int curr_jid = -1;
        int next_rid = -1;
        int next_dir =  0;
    }mRecord;

    std::shared_ptr<Sender> pSender;

public:
    explicit Planner(const Map & map, std::shared_ptr<Sender> _sender);

    void SetStartPoint(const Vector2d & p)
    {
        mStartPoint = p;
    }
    void SetEndPoint(const Vector2d & p)
    {
        mEndPoint = p;
    }
    Vector2d GetStartPoint() const {return mStartPoint; }
    Vector2d GetEndPoint() const {return mEndPoint; }

    void GlobalPlanning();
    void Send();
    std::string ToXML(const std::string &file_name);

private:
    void DFS(SubRoadPtr p_curr, std::vector<SubRoadPtr> v);
    void Evaluate();
};
}



#endif //HDMAP_PLANNER_H
