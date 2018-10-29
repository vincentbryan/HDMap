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
public:
    Pose start_pose;
    Pose end_pose;
    Vector2d p0, p1, p2, p3;
    double length;
    bool is_line;
    double ctrl_len1;
    double ctrl_len2;
    Pose _GetPose(double t);

public:
    explicit Bezier(Pose _start_pose = {0, 0, 0}, Pose _end_pose = {0, 0, 0}, double _ctrl_len1 = 5.0, double _ctrl_len2 = 5.0);
    double Length();
    Pose GetPose(double s);
    Pose GetStartPose();
    Pose GetEndPose();
    std::vector<Pose> GetAllPose(double ds);
    std::vector<double> GetParam();
    static double DEFAULT_LENGTH;
};
}

#endif //HDMAP_BEZIER_H
