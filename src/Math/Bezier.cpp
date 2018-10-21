//
// Created by vincent on 18-10-14.
//

#include "Bezier.h"
#include <cassert>

using namespace hdmap;

double Bezier::DEFAULT_LENGTH = 2.0;

Bezier::Bezier(Pose _start_pose, Pose _end_pose, double _ctrl_len1, double _ctrl_len2)
{
    start_pose = _start_pose;
    end_pose = _end_pose;
    p0 = start_pose.GetPosition();
    p3 = end_pose.GetPosition();
    p1 = p0 + _ctrl_len1 * start_pose.GetAngle().ToVector();
    p2 = p3 - _ctrl_len2 * end_pose.GetAngle().ToVector();
    ctrl_len1 = _ctrl_len1;
    ctrl_len2 = _ctrl_len2;
    length = 0;
    is_line = false;

    if(_start_pose.GetAngle() == _end_pose.GetAngle())
    {
        Angle a(_end_pose.GetPosition() - _start_pose.GetPosition());
        if(a == _start_pose.GetAngle())
        {
            is_line = true;
        }
    }

    if(is_line)
    {
        length = _start_pose.DistanceFrom(_end_pose);
    }
    else
    {
        double t = 0;
        double step = 0.01;
        t += step;
        Pose prev_pose = _GetPose(0);
        Pose curr_pose;
        while (t <= 1.0)
        {
            curr_pose = _GetPose(t);
            length += curr_pose.DistanceFrom(prev_pose);
            prev_pose = curr_pose;
            t += step;
        }
    }
}

Pose Bezier::_GetPose(double t)
{
    assert(0 <= t && t <= 1);
    Vector2d m1 = p0 + t * (p1 - p0);
    Vector2d m2 = p1 + t * (p2 - p1);
    Vector2d m3 = p2 + t * (p3 - p2);

    Vector2d mm1 = m1 + t * (m2 - m1);
    Vector2d mm2 = m2 + t * (m3 - m2);

    Vector2d m = mm1 + t * (mm2 - mm1);
    Angle a(mm2 - mm1);

    return {m, a};
}

double Bezier::Length()
{
    return length;
}

Pose Bezier::GetPose(double s)
{
    assert(length > 0);
    if(is_line)
        return start_pose.GetTranslation(s, start_pose.GetAngle());
    else
        return _GetPose(s / length);
}

std::vector<Pose> Bezier::GetAllPose(double ds)
{
    std::vector<Pose> res;
    double s = 0;
    while (s < length)
    {
        res.emplace_back(GetPose(s));
        s += ds;
        if(s >= length)
        {
            res.emplace_back(GetPose(length));
        }
    }

    return res;
}

Pose Bezier::GetStartPose()
{
    return GetPose(0);
}
Pose Bezier::GetEndPose()
{
    return GetPose(length);
}
