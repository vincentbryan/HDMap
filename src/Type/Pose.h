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
    double yaw;

    Pose()
    {
        x = 0;
        y = 0;
        yaw = 0;
    }

    Pose(double _x, double _y, double _theta = 0)
    {
        x = _x;
        y = _y;
        yaw = _theta;
    }

    Pose(Vector2d v, Angle angle)
    {
        x = v.x;
        y = v.y;
        yaw = angle.ToYaw();
    }

    double DistanceFrom(const Pose &another)
    {
        return sqrt((x - another.x) * (x - another.x) + (y - another.y) * (y - another.y));
    }


    void Rotate(double a)
    {
        yaw += a;
    }

    Pose GetRotation(double n)
    {
        return {x, y, yaw+n};
    }

    ///angle is relative to x-y axis
    void Translate(double length, Angle angle)
    {
        Vector2d v = angle.ToVector();
        x += length * v.x;
        y += length * v.y;
    }

    Pose GetTranslation(double length, Angle angle)
    {
        Vector2d v = angle.ToVector();
        return  {x + length * v.x, y + length * v.y, yaw};
    }

    Vector2d GetPosition()
    {
        return {x, y};
    }

    Angle GetAngle()
    {
        Angle a;
        a.FromYaw(yaw);
        return a;
    }

    void SetPosition(Vector2d v)
    {
        x = v.x;
        y = v.y;
    }

    void SetYaw(Angle angle)
    {
        yaw = angle.ToYaw();
    }

    void SetYaw(double _yaw)
    {
        yaw = _yaw;
    }

};
}

#endif //HDMAP_POSE_H
