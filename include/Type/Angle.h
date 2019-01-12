//
// Created by vincent on 18-10-14.
//

#ifndef HDMAP_ANGLE_H
#define HDMAP_ANGLE_H

#include <cmath>
#include <istream>
#include <iostream>
#include "Coor.h"

namespace hdmap
{
class Angle
{
private:

    double m;

public:

    explicit Angle(double _m = 0) : m(_m){};

    explicit Angle(const Coor &v)
    {
        m = atan2(v.y, v.x) / M_PI * 180.0;
        m = (m);
    }

    double ToYaw()
    {
        m = m - 90;
        Wrap(*this, 0, 360);
        return m;
    }

    void FromYaw(double y)
    {
        m = y + 90;
        Wrap(*this);
    }

    Coor ToVector()
    {
        return {cos(m / 180.0 * M_PI), sin(m / 180.0 * M_PI)};
    }

    void FromVector(const Coor &v)
    {
        m = atan(v.y / v.x) / M_PI * 180.0;
    }

    double Value() const
    {
        return m;
    }

    void SetAngle(double angle)
    {
        m = angle;
    }

    /// Counter clockwise
    void Rotate(double degree)
    {
        m += degree;
    }

    friend std::istream & operator >> (std::istream & is, Angle & angle)
    {
        is >> angle.m;
        return is;
    };

    friend std::ostream & operator << (std::ostream & os, const Angle & angle)
    {
        os << angle.m;
        return os;
    }

    bool operator == (const Angle & a)
    {
        return m == a.Value();
    }

    bool operator != (const Angle & a)
    {
        return m != a.Value();
    }

    Angle operator - (const Angle& _angle) const
    {
        return Angle( this->m - _angle.m);
    }

    /// down and up accept degree.
    static void Wrap(Angle& angle, double down = 0.0, double up = 360.0)
    {
        while (angle.m < down) angle.m += 360.0;
        while (angle.m >= up) angle.m -= 360.0;
    }
};
}

#endif //HDMAP_ANGLE_H
