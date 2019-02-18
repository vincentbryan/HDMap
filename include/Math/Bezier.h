#ifndef HDMAP_BEZIER_H
#define HDMAP_BEZIER_H

#include <vector>
#include <cassert>
#include "Type/Pose.h"

namespace hdmap
{
class Bezier
{
private:
    Pose start_pose;
    Pose end_pose;
    Coor p0, p1, p2, p3;
    double length = 0.0;
    bool is_line = false;
    double ctrl_len1 = 0.0;
    double ctrl_len2 = 0.0;

    Coor a, b, c, d;

    Pose _GetPose(double t);

public:
    explicit Bezier(Pose _start_pose = {0, 0, 0}, Pose _end_pose = {0, 0, 0}, double _ctrl_len1 = 5.0, double _ctrl_len2 = 5.0);
    explicit Bezier(std::vector<double> v);
    double Length();
    Pose GetPose(double s);
    Pose GetStartPose();
    Pose GetEndPose();

    std::vector<Pose> GetPoses(double ds, double start = 0.0, double end = 1.0);
    std::vector<double> GetParam();
    static double DEFAULT_LENGTH;
};
}

#endif //HDMAP_BEZIER_H
