//
// Created by vincent on 18-10-9.
//

#include "Type/Pose.h"
using namespace hdmap;

double Pose::AngleThresHold = 2.5;

bool Pose::InLine(const hdmap::Pose &p1, const hdmap::Pose &p2)
{
    if(fabs(p1.GetAngle().Value() - p2.GetAngle().Value()) < Pose::AngleThresHold)
    {
        hdmap::Angle angle(p2.GetPosition() - p1.GetPosition());

        if(fabs(angle.Value() - p1.GetAngle().Value()) < Pose::AngleThresHold &&
            fabs(angle.Value() - p2.GetAngle().Value()) < Pose::AngleThresHold)
        {
            return true;
        }
        else
            return false;
    }

    return false;
}
