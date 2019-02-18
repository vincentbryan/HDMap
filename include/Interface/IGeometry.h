// IGeometry 提供了描述路网几何特征的接口和方法

#ifndef HDMAP_IGEOMETRY_H
#define HDMAP_IGEOMETRY_H

#include <vector>
#include <array>
#include <algorithm>
#include <iostream>
#include <exception>

#include "Common/kdtree.hpp"
#include "Type/Pose.h"

namespace hdmap
{
class IGeometry
{
public:
    /**
     * 获取组成多边形边缘的顶点姿态
     * @return 边缘顶点姿态向量
     */
    const std::vector<Pose>& GetRegionPoses()
    {
        if (mRegionPoses.empty()) GenerateRegionPoses();

        if (mRegionPoses.empty())
        {
            throw std::runtime_error("Cannot not Generate Pose.");
        }

        return mRegionPoses;
    }

    /**
     * 用于判断一个顶点是否处于多边形内部
     * @param v 需要进行判断的顶点
     * @return 如果传入的顶点处于多边形内返回 true，否则 false
     */
    bool IsCover(const Coor& v)
    {
        return Cover(GetRegionPoses(), {v});
    }

    /**
     * 计算一个顶点到多边形的距离，存在误差
     * @param v 需要进行计算的顶点
     * @return 顶点到多边形的距离
     */
    double GetDistanceFromCoor(const Coor& v)
    {
        if (IsCover(v)) return 0;
        std::vector<int> indices;
        std::vector<double> distances;
        mKdtree.NearestSearch({v.x, v.y}, indices, distances, 1);
        return distances[0];
    }
    
    /**
     * 类的全局方法，用于判断是否一系列点vp都处在一系列点vertices所围起来的多边形内
     * @param vertices 组成多边形的顶点集合
     * @param vp 需要判断的一系列顶点
     * @return 如果vp中所有的点都在多边形内部，返回true，否则false
     */
    static bool Cover(std::vector<Pose> vertices, const std::vector<Coor> &vp)
    {

        if (vertices.size() < 3) return false;

        std::vector<bool> res(vp.size(), false);

        vertices.emplace_back(vertices.front());
        for(int i = 0; i + 1 < vertices.size(); ++i)
        {
            double slope = 0.0;
            if(vertices[i+1].x - vertices[i].x != 0)
            {
                slope = (vertices[i+1].y - vertices[i].y) / (vertices[i+1].x - vertices[i].x);
            }
            else
            {
                for (const auto &v:vp) {
                    if (vertices[i].x == v.x and (vertices[i].y - v.y) * (vertices[i + 1].y - v.y) < 0)
                        return true;
                }
            }
            for (int j = 0; j < vp.size(); ++j) {
                auto &v = vp[j];
                double t1 = (v.x - vertices[i].x) * (v.x - vertices[i + 1].x);
                double t2 = v.y - (slope * (v.x - vertices[i].x) + vertices[i].y);
                if (t1 < 0 and t2 < 0) res[j] = !res[j];
            }
        }
        return std::all_of(res.begin(), res.end(), [](bool v) { return v; });
    }

protected:

    static constexpr double CURVE_DS = 1.0;

    virtual void GenerateRegionPoses() = 0;

    kt::kdtree<double> mKdtree;

    std::vector<std::vector<double>> mKdtreeData;

    std::vector<Pose> mRegionPoses;

};

//const double IGeometry::CURVE_DS = 1.0;

}

#endif //HDMAP_IGEOMETRY_H
