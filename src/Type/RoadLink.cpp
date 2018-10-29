//
// Created by vincent on 18-10-16.
//

#include "RoadLink.h"
using namespace hdmap;

SubRoadLink RoadLink::operator()(int _form_dir, int _to_dir)
{
    SubRoadLink slk;
    slk.iFromRoadId = iFromRoadId;
    slk.iToRoadId = iToRoadId;

    for(auto const & x : vLaneLinks)
    {
        if(x.iFromIndex * _form_dir > 0 and x.iToIndex * _to_dir > 0)
            slk.vLaneLinks.emplace_back(x);
    }

    return slk;
}

void SubRoadLink::Send(Sender &sender)
{
    for(auto & x : vLaneLinks)
    {
        auto p = x.mReferLine.GetAllPose(0.1);
        sender.SendPoses(p, 1.0, 156.0/255, 30.0/255, 1.0, 0.0, 0.08);
    }
}
