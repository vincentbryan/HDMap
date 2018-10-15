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

void Junction::AddConnection(Lane from_lane, Pose start_pose, Lane to_lane, Pose end_pose)
{
    Connection conn;
    conn.from_lane_id = from_lane.land_id;
    conn.to_lane_id = to_lane.land_id;

    //TODO
    conn.curve.reset(new Line(0, start_pose, end_pose));
    mConnection.emplace_back(conn);
}

std::vector<std::vector<Pose>> Junction::GetAllPose()
{
    if(mPose.empty())
    {
        for(auto x : mConnection)
        {
            mPose.emplace_back(x.curve->GeneratePose(0.1));
        }
    }

    return mPose;
}