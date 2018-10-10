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
    double theta;

    Pose()
    {
        x = 0;
        y = 0;
        theta = 0;
    }

    Pose(double _x, double _y, double _theta = 0)
    {
        x = _x;
        y = _y;
        theta = _theta;
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
        p.theta = theta;
        return p;
    }

    friend Pose operator * (double d, Pose& p)
    {
        return p * d;
    }

};
}

#endif //HDMAP_POSE_H
