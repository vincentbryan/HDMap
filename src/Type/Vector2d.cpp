//
// Created by vincent on 18-10-14.
//

#include <Type/Vector2d.h>

double hdmap::Vector2d::SegmentDistance(const hdmap::Vector2d &start,
                                        const hdmap::Vector2d &end,
                                        const hdmap::Vector2d &target)
{
    Vector2d start_to_end = end - start;
    Vector2d start_to_target = target - start;

    double r = start_to_end.cross(start_to_target) / (start_to_end.norm() * start_to_end.norm());

    if(r <= 0)
    {
        return start_to_target.norm();
    }
    else if(r >= 1)
    {
        return (target - end).norm();
    }
    else
    {
        r *= start_to_end.norm();
        return sqrt(start_to_target.norm() * start_to_target.norm() - r*r);
    }
}

double hdmap::Vector2d::Distance(const hdmap::Vector2d &a, const hdmap::Vector2d &b) {
    return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));   
    return 0;
}
