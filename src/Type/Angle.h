//
// Created by vincent on 18-10-14.
//

#ifndef HDMAP_ANGLE_H
#define HDMAP_ANGLE_H

#include <cmath>
#include "Vector2d.h"
namespace hdmap
{
class Angle
{
private:
    double m;

public:
    explicit Angle(double _m = 0) : m(_m){};
    explicit Angle(const Vector2d & v)
    {
        m = atan(v.y / v.x) / M_PI * 180.0;
    }
    double ToYaw()
    {
        //TODO to validate
        return fmod(m-90, 360);
    }

    void FromYaw(double y)
    {
        m = fmod(y+90, 360);
    }

    Vector2d ToVector()
    {
        return {cos(m / 180.0 * M_PI), sin(m / 180.0 * M_PI)};
    }

    void FromVector(const Vector2d & v)
    {
        m = atan(v.y / v.x) / M_PI * 180.0;
    }

    double GetAngle()
    {
        return m;
    }

    void SetAngle(double angle)
    {
        m = angle;
    }

    void Rotate(double degree)
    {
        m += degree;
    }
};
}

#endif //HDMAP_ANGLE_H
