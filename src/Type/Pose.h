//
// Created by vincent on 18-10-9.
//

#ifndef HDMAP_POSE_H
#define HDMAP_POSE_H

#include <cmath>
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

    double DistanceFrom(const Pose &another)
    {
        return sqrt((x - another.x) * (x - another.x) + (y - another.y) * (y - another.y));
    }

    void Normalize()
    {
        double d = sqrt(x*x + y*y);
        x /= d;
        y /= d;
    }

    Pose GetNormalization()
    {
        Pose p = *this;
        p.Normalize();
        return p;
    }

    Pose operator * (double d)
    {
        Pose p;
        p.x = x * d;
        p.y = y * d;
        p.yaw = yaw;
        return p;
    }

    friend Pose operator * (double d, Pose& p)
    {
        return p * d;
    }

    void Rotate(double angle)
    {
        yaw += angle;
    }

    Pose GetRotation(double angle)
    {
        return {x, y, yaw+angle};
    }

    ///angle is relative to x-y axis
    void Translate(double length, double angle)
    {
        x += length * cos(angle / 180.0 * M_PI);
        y += length * sin(angle / 180.0 * M_PI);
    }

    Pose GetTranslation(double length, double angle)
    {
        return  {x + length * cos(angle / 180.0 * M_PI),
                 y + length * sin(angle / 180.0 * M_PI),
                 yaw};
    }


};
}

#endif //HDMAP_POSE_H
