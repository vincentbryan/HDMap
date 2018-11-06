//
// Created by vincent on 18-11-3.
//

#ifndef HDMAP_IGEOMETRY_H
#define HDMAP_IGEOMETRY_H
#include "../Type/Vector2d.h"

namespace hdmap
{
class IGeometry
{
public:
    virtual bool Cover(const Vector2d & v) = 0;

protected:
    bool Cover(std::vector<Vector2d> _vertices, const Vector2d &v)
    {
        bool res = false;
        if(_vertices.size() < 3) return res;

        _vertices.emplace_back(_vertices.front());
        for(int i = 0; i + 1 < _vertices.size(); ++i)
        {
            double slope = 0.0;
            if(_vertices[i+1].x - _vertices[i].x != 0)
            {
                slope = (_vertices[i+1].y - _vertices[i].y) / (_vertices[i+1].x - _vertices[i].x);
            }
            else
            {
                if(_vertices[i].x == v.x and (_vertices[i].y - v.y) * (_vertices[i+1].y - v.y) < 0)
                    return true;
            }

            double t1 = (v.x - _vertices[i].x) * (v.x - _vertices[i+1].x);
            double t2 = v.y - (slope * (v.x - _vertices[i].x) + _vertices[i].y);
            if(t1 < 0 and t2 < 0) res = !res;
        }
        return res;
    }
};

}

#endif //HDMAP_IGEOMETRY_H
