//
// Created by vincent on 18-10-13.
//

#include "Junction.h"
#include "../Math/Line.h"

using namespace hdmap;

unsigned int Junction::JUNCTION_ID = 0;

Junction::Junction()
{
    iJunctionId = JUNCTION_ID++;
}

void Junction::AddConnection(Lane from_lane, Pose start_pose, Lane to_lane, Pose end_pose, double _ctrl_len1, double _ctrl_len2)
{
    mConnection.emplace_back(
        Connection(from_lane.land_id, to_lane.land_id, Bezier(start_pose, end_pose, _ctrl_len1, _ctrl_len2))
    );
}

std::vector<std::vector<Pose>> Junction::GetAllPose()
{
    if(mPose.empty())
    {
        for(auto x : mConnection)
        {
            mPose.emplace_back(x.refer_line.GetAllPose(0.1));
        }
    }
    return mPose;
}