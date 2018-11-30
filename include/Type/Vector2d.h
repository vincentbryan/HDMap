//
// Created by vincent on 18-10-14.
//

#ifndef HDMAP_VECTOR2D_H
#define HDMAP_VECTOR2D_H

#include <cmath>
namespace hdmap
{
class Vector2d
{
public:
    double x;
    double y;

public:
    Vector2d(double _x = 0, double _y = 0) : x(_x), y(_y){};

    Vector2d(double angle)
    {
        x = cos(angle);
        y = sin(angle);
    }

    Vector2d operator + (const Vector2d & v) const
    {
        return {x+v.x, y+v.y};
    }

    Vector2d& operator +=(const Vector2d & v)
    {
        x+=v.x;
        y+=v.y;
        return *this;
    }

    Vector2d& operator *=(const double & v)
    {
        x*=v;
        y*=v;
        return *this;
    }

    Vector2d& operator /=(const double & v)
    {
        (*this)*=(1/v);
        return *this;
    }

    Vector2d operator - (const Vector2d & v) const
    {
        return {x-v.x, y-v.y};
    }

    Vector2d operator * (double d) const
    {
        return {d*x, d*y};
    }

    Vector2d operator / (double d) const
    {
        if(d == 0.0) return {0, 0};
        else return {x/d, y/d};
    }

    friend Vector2d operator * (double d, const Vector2d & v)
    {
        return v * d;
    }
    bool operator == (const Vector2d & v)
    {
        return fabs(x - v.x) < 0.000001 && fabs(y - v.y) < 0.000001;
    }

    bool operator != (const Vector2d & v)
    {
        return fabs(x - v.x) >= 0.000001 || fabs(y - v.y) >= 0.000001;
    }

    Vector2d& operator = (const Vector2d& v)
    {
        x = v.x;
        y = v.y;
        return *this;
    }

    double cross(const Vector2d & v)
    {
        return x * v.x + y * v.y;
    }

    double norm()
    {
        return std::sqrt(x*x + y*y);
    }
    
    static double Distance(const Vector2d& a, const Vector2d& b);

    static double SegmentDistance(const Vector2d & start, const Vector2d & end, const Vector2d & target);
};
}



#endif //HDMAP_VECTOR2D_H
