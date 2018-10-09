//
// Created by vincent on 18-10-8.
//

#include <ldap.h>
#include "CubicPoly.h"

using namespace hdmap;

CubicPoly::CubicPoly(double _offset, double _a, double _b, double _c, double _d)
{
    offset = _offset;
    a = _a;
    b = _b;
    c = _c;
    d = _d;
}

CubicPoly::CubicPoly()
{
   offset = a = b = c = d = 0;
}