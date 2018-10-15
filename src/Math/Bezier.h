//
// Created by vincent on 18-10-14.
//

#ifndef HDMAP_BEZIER_H
#define HDMAP_BEZIER_H

#include <vector>
#include "../Type/Pose.h"
namespace hdmap
{
class Bezier
{
private:
    Pose start_pose;
    Pose end_pose;
    Vector2d p0, p1, p2, p3;
    double length;

    Pose _GetPose(double t);

public:
    Bezier(Pose _start_pose, Pose _end_pose);
    double Length();
    Pose GetPose(double s);
    std::vector<Pose> GetAllPose(double ds);
};

}

#endif //HDMAP_BEZIER_H
