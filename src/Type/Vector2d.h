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

    Vector2d operator + (const Vector2d & v)
    {
        return {x+v.x, y+v.y};
    }

    Vector2d operator - (const Vector2d & v)
    {
        return {x-v.x, y-v.y};
    }

    Vector2d operator * (double d) const
    {
        return {d*x, d*y};
    }

    friend Vector2d operator * (double d, const Vector2d & v)
    {
        return v * d;
    }
    bool operator == (Vector2d & v)
    {
        return x == v.x && y == v.y;
    }

    bool operator != (Vector2d & v)
    {
        return x != v.x || y != v.y;
    }

    Vector2d& operator = (const Vector2d& v)
    {
        x = v.x;
        y = v.y;
        return *this;
    }
};
}



#endif //HDMAP_VECTOR2D_H