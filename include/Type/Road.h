#ifndef HDMAP_ROAD_H
#define HDMAP_ROAD_H

#include "LaneSection.h"
#include "Signal.h"
#include "Common/pointer_typedef.h"
#include "Common/kdtree.hpp"
#include <algorithm>

namespace hdmap
{
class Road : public IView, public IXML, public IGeometry
{

public:
    /**
     * 当前节点的最大Road的ID
     */
    static unsigned int ROAD_ID;

    /**
     * 当前Road的ID
     */
    unsigned int ID;

    /**
     * 当前路的长度信息
     */
    double Lenght;

    /**
     * 当前路的最大速度信息
     */
    double Max_Speed;

    /**
     * 当前路的最小速度信息
     */
    double Min_Speed;

    /**
     * 当前路的方向信息
     */
    int Direction;

private:

    void GenerateRegionPoses() override;

public:

    std::vector<SecPtr> mSecPtrs;

    std::vector<SigPtr> mSigPtrs;

    int mPrevJid;

    int mNextJid;

    Pose mStartPose;

    Pose mEndPose;

    /**
     * 创建一个 Road 对象
     * @param _start_pose 设置Road的起始姿态
     */
    explicit Road(Pose _start_pose = Pose());

    /**
     * 添加一个路段到当前路，路段起始姿态为上一路段的结束姿态，使用贝塞尔曲线描述路段
     * @param _end_pose 路段的结束姿态
     * @param _ctrl_len1 路段起始点控制长度
     * @param _ctrl_len2 路段结束点控制长度
     * @return 生成路段的指针
     */
    SecPtr AddSection(const Pose & _end_pose, double _ctrl_len1 = 1.0, double _ctrl_len2 = 1.0);

    /**
     * 添加一个红绿灯到当前路
     * @param _x 红绿灯所在的位置的x值
     * @param _y 红绿灯所在的位置的y值
     * @param _z 红绿灯所在的位置的z值
     * @param dir 红绿灯面板面向的方向旋转180度后的值，单位为角度
     * @param _type "SIG"类型为信号灯
     * @param _info 当 _type="SIG" 时，info为四个字符，分别代表是否左转、直行、右转，灯的类型是否为圆形
     *              例如 _info="0101" 表示灯可以直行，并且灯为圆形
     */
    void AddSignal(double _x, double _y, double _z, Angle dir, std::string _type, std::string _info);

    /**
     * 为当前道路添加速度限制信息
     * @param min_speed 当前路的最低速
     * @param max_speed 当前路的最高速
     */
    void AddSpeedInfo(const double& min_speed, const double& max_speed);

    /**
     * 获取信号灯的信息
     * @return 返回一个在当前道路上所有信号灯信息的集合
     */
    std::vector<SigPtr> GetSignals();

    /**
     * 获取当前路的引导线的姿态点集
     * @return 当前路的引导线的姿态点集
     */
    std::vector<Pose> GetReferenceLinePoses();

    /**
     * 获取当前路的最右边缘点集合
     * @return 当前路的最右边缘点集合
     */
    std::vector<Pose> GetRightmostLinePoses();

    void OnSend(Sender &sender) override;

    boost::property_tree::ptree ToXML() override;

    void FromXML(const pt::ptree &p) override;
};
}

#endif //HDMAP_ROAD_H
