//
// Created by vincent on 18-10-13.
//

#ifndef HDMAP_JUNCTION_H
#define HDMAP_JUNCTION_H

#include <utility>
#include <vector>
#include <map>
#include <memory>
#include "Curve.h"
#include "Lane.h"

namespace hdmap
{
class Junction
{
private:
    struct Connection
    {
        unsigned int from_lane_id;
        unsigned int to_lane_id;
        std::shared_ptr<Curve> curve;
    };
public:
    static unsigned int JUNCTION_ID;
    unsigned int iJunctionId;

    std::vector<Connection> mConnection;
    std::vector<std::vector<Pose>> mPose;

    explicit Junction();

    void AddConnection(Lane from_lane, Pose start_pose, Lane to_lane, Pose end_pose);
    std::vector<std::vector<Pose>> GetAllPose();
};
}



#endif //HDMAP_JUNCTION_H
