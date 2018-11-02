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
private:
    Map mHDMap;
    Vector2d mStartPoint;
    Vector2d mEndPoint;
    RoadPtr pStart;
    RoadPtr pEnd;

    std::map<unsigned int, bool> is_visited;
    std::vector<RoadPtr> mRouting;
    std::vector<std::vector<RoadPtr>> mAllRouting;

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
    void DFS(RoadPtr p_curr, std::vector<RoadPtr> v);
    void Evaluate();
};
}
#endif //HDMAP_PLANNER_H
