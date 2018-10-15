//
// Created by vincent on 18-10-15.
//

#ifndef HDMAP_CUBICFUNCTION_H
#define HDMAP_CUBICFUNCTION_H

namespace hdmap
{
class CubicFunction
{
private:
    double a, b, c, d;
    double range;

public:
    CubicFunction(double dx1, double _range, double dx2);
    double Value(double x);
};
}



#endif //HDMAP_CUBICFUNCTION_H
