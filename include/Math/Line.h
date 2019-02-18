#ifndef HDMAP_LINE_H
#define HDMAP_LINE_H

#include "Curve.h"
#include "Type/Pose.h"

namespace hdmap
{
class Line : public Curve
{
public:
    double start;
    double length;
    Pose start_pose;
    Pose end_pose;
    Angle direction;

    Line(double _start, Pose _start_pose, Pose _end_pose);

    double Length() override;
    std::vector<Pose> GeneratePose(double step) override;
    Pose GetPose(double ds) override;
};
}

#endif //HDMAP_LINE_H
