//
// Created by vincent on 18-10-13.
//

#ifndef HDMAP_JUNCTION_H
#define HDMAP_JUNCTION_H

#include <utility>
#include <vector>
#include <map>
#include <memory>
#include "../Math/Curve.h"
#include "Lane.h"
#include "../Math/Bezier.h"

namespace hdmap
{
class Junction
{
public:
    struct Connection
    {
        unsigned int from_lane_id;
        unsigned int to_lane_id;
        Bezier refer_line;

        Connection(unsigned int id1, unsigned int id2, Bezier b)
        {
            from_lane_id = id1;
            to_lane_id = id2;
            refer_line = b;
        }
    };
public:
    static unsigned int JUNCTION_ID;
    unsigned int iJunctionId;

    std::vector<Connection> mConnection;
    std::vector<std::vector<Pose>> mPose;

    explicit Junction();

    void AddConnection(Lane from_lane, Pose start_pose, Lane to_lane, Pose end_pose, double _ctrl_len1, double _ctrl_len2);
    std::vector<std::vector<Pose>> GetAllPose();
};
}



#endif //HDMAP_JUNCTION_H
