//
// Created by vincent on 18-10-8.
//

#ifndef HDMAP_WIDTH_H
#define HDMAP_WIDTH_H

namespace hdmap
{
class CubicPoly
{
public:
    double offset;
    double a, b, c, d;
    CubicPoly(double _offset, double _a, double _b, double _c, double _d);
    CubicPoly();
};
}



#endif //HDMAP_WIDTH_H
