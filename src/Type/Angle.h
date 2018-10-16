//
// Created by vincent on 18-10-14.
//

#ifndef HDMAP_ANGLE_H
#define HDMAP_ANGLE_H

#include <cmath>
#include <istream>
#include <iostream>
#include "Vector2d.h"

namespace hdmap
{
class Angle
{
private:
    double m;
    double Warp(double d)
    {
        while (d < 0) d += 360.0;
        while (d >= 360.0) d -= 360.0;
        return d;
    }

public:
    explicit Angle(double _m = 0) : m(_m){};
    explicit Angle(const Vector2d & v)
    {
        m = atan(v.y / v.x) / M_PI * 180.0;
    }

    double ToYaw()
    {
        return Warp(m-90);
    }

    void FromYaw(double y)
    {
        m = Warp(y + 90);
    }

    Vector2d ToVector()
    {
        return {cos(m / 180.0 * M_PI), sin(m / 180.0 * M_PI)};
    }

    void FromVector(const Vector2d & v)
    {
        m = atan(v.y / v.x) / M_PI * 180.0;
    }

    double GetAngle() const
    {
        return m;
    }

    void SetAngle(double angle)
    {
        m = angle;
    }

    ///Counterclockwise
    void Rotate(double degree)
    {
        m += degree;
    }

    friend std::istream & operator >> (std::istream & is, Angle & angle)
    {
        is >> angle.m;
    };

    friend std::ostream & operator << (std::ostream & os, const Angle & angle)
    {
        os << angle.m;
    }

    bool operator == (const Angle & a)
    {
        return m == a.GetAngle();
    }

    bool operator != (const Angle & a)
    {
        return m != a.GetAngle();
    }
};
}

#endif //HDMAP_ANGLE_H
