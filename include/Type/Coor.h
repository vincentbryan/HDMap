#ifndef HDMAP_VECTOR2D_H
#define HDMAP_VECTOR2D_H

#include <cmath>
namespace hdmap
{
    class Coor
{
public:
    double x;
    double y;

public:
        Coor(double _x = 0, double _y = 0) : x(_x), y(_y) {};

        Coor(double angle)
    {
        x = cos(angle);
        y = sin(angle);
    }

        Coor operator+(const Coor &v) const
    {
        return {x+v.x, y+v.y};
    }

        Coor &operator+=(const Coor &v)
    {
        x+=v.x;
        y+=v.y;
        return *this;
    }

        Coor &operator*=(const double &v)
    {
        x*=v;
        y*=v;
        return *this;
    }

        Coor &operator/=(const double &v)
    {
        (*this)*=(1/v);
        return *this;
    }

        Coor operator-(const Coor &v) const
    {
        return {x-v.x, y-v.y};
    }

        Coor operator*(double d) const
    {
        return {d*x, d*y};
    }

        Coor operator/(double d) const
    {
        if(d == 0.0) return {0, 0};
        else return {x/d, y/d};
    }

        friend Coor operator*(double d, const Coor &v)
    {
        return v * d;
    }

        bool operator==(const Coor &v)
    {
        return fabs(x - v.x) < 0.000001 && fabs(y - v.y) < 0.000001;
    }

        bool operator!=(const Coor &v)
    {
        return fabs(x - v.x) >= 0.000001 || fabs(y - v.y) >= 0.000001;
    }

        Coor &operator=(const Coor &v)
    {
        x = v.x;
        y = v.y;
        return *this;
    }

        double cross(const Coor &v)
    {
        return x * v.x + y * v.y;
    }

    double norm()
    {
        return std::sqrt(x*x + y*y);
    }

        static double Distance(const Coor &a, const Coor &b);

        static double SegmentDistance(const Coor &start, const Coor &end, const Coor &target);
};
}



#endif //HDMAP_VECTOR2D_H
