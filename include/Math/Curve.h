#ifndef HDMAP_CURVE_H
#define HDMAP_CURVE_H

#include <vector>
#include "Type/Pose.h"
namespace hdmap
{
class Curve
{
public:
    virtual double Length() = 0;
    virtual std::vector<Pose> GeneratePose(double step) = 0;
    virtual Pose GetPose(double ds) = 0;
};
}


#endif //HDMAP_CURVE_H
