//
// Created by vincent on 18-11-3.
//

#ifndef HDMAP_IGEOMETRY_H
#define HDMAP_IGEOMETRY_H

#include "Type/Pose.h"
#include <vector>
#include <array>
#include <algorithm>

namespace hdmap
{
class IGeometry
{
public:

    virtual bool Cover(const Coor &v) = 0;

    virtual std::vector<Pose> GetRegionPoses() = 0;

protected:

    virtual void GenerateRegionPoses() = 0;

    bool Cover(std::vector<Pose> _vertices, const std::vector<Coor> &vp)
    {

        if (_vertices.size() < 3) return false;

        std::vector<bool> res(vp.size(), false);

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
                for (const auto &v:vp) {
                    if (_vertices[i].x == v.x and (_vertices[i].y - v.y) * (_vertices[i + 1].y - v.y) < 0)
                        return true;
                }
            }
            for (int j = 0; j < vp.size(); ++j) {
                auto &v = vp[j];
                double t1 = (v.x - _vertices[i].x) * (v.x - _vertices[i + 1].x);
                double t2 = v.y - (slope * (v.x - _vertices[i].x) + _vertices[i].y);
                if (t1 < 0 and t2 < 0) res[j] = !res[j];
            }
        }
        return std::all_of(res.begin(), res.end(), [](bool v) { return v; });
    }
};

}

#endif //HDMAP_IGEOMETRY_H
