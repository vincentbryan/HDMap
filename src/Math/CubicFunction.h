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
    explicit CubicFunction(double dx1 = 1.0, double _range = 1.0, double dx2 = 1.0);
    double Value(double x);
};
}



#endif //HDMAP_CUBICFUNCTION_H
