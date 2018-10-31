//
// Created by vincent on 18-10-15.
//

#include "CubicFunction.h"

using namespace hdmap;

CubicFunction::CubicFunction(double y0, double _range, double y1)
{
    double dx0 = 0;
    double dx1 = 0;

    a = 2*y0 - 2*y1 + dx0 + dx1;
    b = -3*y0 + 3*y1 - 2*dx0 - dx1;
    c = dx0;
    d = y0;

    range = _range;

    this->x0 = 0;
    this->y0 = y0;
    this->x1 = _range;
    this->y1 = y1;
}

double CubicFunction::Value(double x)
{
    double t = x/range;
    return a*(t*t*t) + b*(t*t) + c*t + d;
}

CubicFunction::CubicFunction(double _a, double _b, double _c, double _d, double _range)
{
    a = _a;
    b = _b;
    c = _c;
    d = _d;
    range = _range;
}
