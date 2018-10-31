//
// Created by vincent on 18-10-15.
//

#ifndef HDMAP_CUBICFUNCTION_H
#define HDMAP_CUBICFUNCTION_H

namespace hdmap
{
class CubicFunction
{
public:
    double a, b, c, d;
    double range;

public:
    double x0, y0, x1, y1;

public:
    explicit CubicFunction(double y0 = 0.0, double _range = 1.0, double y1 = 0.0);
    explicit CubicFunction(double _a, double _b, double _c, double _d, double _range);
    double Value(double x);
};
}



#endif //HDMAP_CUBICFUNCTION_H
