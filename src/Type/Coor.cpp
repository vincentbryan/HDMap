#include <Type/Coor.h>

double hdmap::Coor::SegmentDistance(const hdmap::Coor &start,
                                    const hdmap::Coor &end,
                                    const hdmap::Coor &target) {
    Coor start_to_end = end - start;
    Coor start_to_target = target - start;

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

double hdmap::Coor::Distance(const hdmap::Coor &a, const hdmap::Coor &b) {
    return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));   
    return 0;
}
