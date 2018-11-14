//
// Created by vincent on 18-10-9.
//

#ifndef HDMAP_POSE_H
#define HDMAP_POSE_H

#include <cmath>
#include "Vector2d.h"
#include "Angle.h"
namespace hdmap
{
struct Pose
{
    double x;
    double y;
    Angle direction;

    Pose()
    {
        x = 0;
        y = 0;
        direction = Angle(0);
    }

    Pose(double _x, double _y, double _theta = 0)
    {
        x = _x;
        y = _y;
        direction = Angle(_theta);
    }

    Pose(Vector2d v, Angle angle)
    {
        x = v.x;
        y = v.y;
        direction = angle;
    }

    double DistanceFrom(const Pose &another)
    {
        return sqrt((x - another.x) * (x - another.x) + (y - another.y) * (y - another.y));
    }


    void Rotate(double a)
    {
        direction.Rotate(a);
    }

    Pose GetRotation(double n)
    {
        Pose p(x, y, direction.Value());
        p.Rotate(n);
        return p;
    }

    ///angle is relative to x-y axis
    void Translate(double length, Angle angle)
    {
        Vector2d v = angle.ToVector();
        x += length * v.x;
        y += length * v.y;
    }

    Pose GetTranslation(double length, Angle angle) const
    {
        Vector2d v = angle.ToVector();
        return  {x + length * v.x, y + length * v.y, direction.Value()};
    }

    Vector2d GetPosition() const
    {
        return {x, y};
    }

    Angle GetAngle() const
    {
        return direction;
    }

    void SetPosition(Vector2d v)
    {
        x = v.x;
        y = v.y;
    }

    void SetAngleFromYaw(double _yaw)
    {
        direction.FromYaw(_yaw);
    }

    static bool InLine(const Pose & p1, const Pose & p2);
    static double AngleThresHold;
};
}

#endif //HDMAP_POSE_H
