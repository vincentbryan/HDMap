//
// Created by vincent on 18-10-9.
//

#include <boost/property_tree/ptree.hpp>
#include <algorithm>
#include <LocalMapRequest.h>
#include "Map.h"
#include "common.h"

using namespace hdmap;
namespace pt = boost::property_tree;

Map::Map()
{}

void Map::SetSender(std::shared_ptr<Sender> _sender)
{
    pSender = _sender;
}


RoadPtr Map::AddRoad(const Pose &_start_pose)
{
    RoadPtr p(new Road(_start_pose));
    mRoadPtrs.emplace_back(p);
    return p;
}

JuncPtr Map::AddJunction()
{
    JuncPtr p(new Junction());
    mJuncPtrs.emplace_back(p);
    return p;
}

void Map::Send()
{
    for(auto & m : mRoadPtrs) m->Send(*pSender);
    for(auto & j : mJuncPtrs) j->Send(*pSender);
}


void Map::AddConnection(JuncPtr p, unsigned int from_road, int from_lane_idx,
                        unsigned int to_road, int to_lane_idx,
                        double _ctrl_len1, double _ctrl_len2)
{
    Pose start_pose, end_pose;

    if(from_lane_idx > 0)
    {
        auto p1 = mRoadPtrs[from_road]->mSecPtrs.back()->GetLanePoseByIndex(from_lane_idx).back();
        auto p2 = mRoadPtrs[from_road]->mSecPtrs.back()->GetLanePoseByIndex(from_lane_idx-1).back();
        start_pose = {0.5 * (p1.GetPosition() + p2.GetPosition()), p1.GetAngle()};
    }
    else
    {
        auto p1 = mRoadPtrs[from_road]->mSecPtrs.front()->GetLanePoseByIndex(from_lane_idx).front();
        auto p2 = mRoadPtrs[from_road]->mSecPtrs.front()->GetLanePoseByIndex(from_lane_idx+1).front();
        start_pose = {0.5 * (p1.GetPosition() + p2.GetPosition()), p1.GetAngle()};
        start_pose.Rotate(180);
    }


    if(to_lane_idx >  0)
    {
        auto p1 = mRoadPtrs[to_road]->mSecPtrs.front()->GetLanePoseByIndex(to_lane_idx).front();
        auto p2 = mRoadPtrs[to_road]->mSecPtrs.front()->GetLanePoseByIndex(to_lane_idx-1).front();
        end_pose = {0.5 * (p1.GetPosition() + p2.GetPosition()), p1.GetAngle()};
    }
    else
    {
        auto p1 = mRoadPtrs[to_road]->mSecPtrs.back()->GetLanePoseByIndex(to_lane_idx).back();
        auto p2 = mRoadPtrs[to_road]->mSecPtrs.back()->GetLanePoseByIndex(to_lane_idx+1).back();
        end_pose = {0.5 * (p1.GetPosition() + p2.GetPosition()), p1.GetAngle()};
        end_pose.Rotate(180);
    }


    p->AddConnection(
         from_road, from_lane_idx, start_pose,
         to_road, to_lane_idx, end_pose,
         _ctrl_len1, _ctrl_len2
    );
    
    if(from_lane_idx > 0)
        mRoadPtrs[from_road]->mNextJid = p->mJunctionId;
    else
        mRoadPtrs[from_road]->mPrevJid = p->mJunctionId;
    
    if(to_lane_idx > 0)
        mRoadPtrs[to_road]->mPrevJid = p->mJunctionId;
    else
        mRoadPtrs[to_road]->mNextJid = p->mJunctionId;
}

void Map::CommitRoadInfo()
{
    for(auto & p : mRoadPtrs)
    {
        if(!p->mSecPtrs.empty())
            p->mLength = p->mSecPtrs.back()->mStartS + p->mSecPtrs.back()->mReferLine.Length();
    }
}

void Map::Save(const std::string &file_name)
{
    try
    {
        pt::write_xml(file_name, ToXML());
    }
    catch (std::exception &e)
    {
        std::cout << "Error: " << e.what() << std::endl;
    }
}

void Map::Load(const std::string &file_name)
{
    try
    {
        pt::ptree tree;
        pt::read_xml(file_name, tree);
        FromXML(tree);
    }
    catch (std::exception &e)
    {
        std::cout << "Error: " << e.what() << std::endl;
    }
}

std::vector<std::shared_ptr<SubRoad>> Map::AdjacentRoadInfo(std::shared_ptr<SubRoad> pSubRoad)
{
    std::vector<std::shared_ptr<SubRoad>> res;

    int jid = pSubRoad->mNextJid;
    if(jid == -1) return res;

    for(auto const & m : mJuncPtrs[jid]->mRoadLinks)
    {
        if(m.first.first == pSubRoad->mRoadId)
        {
            int dir = m.second.mLaneLinks.front().mToLaneIndex > 0 ? 1 : -1;

            res.emplace_back(mRoadPtrs[m.first.second]->GetSubRoadPtr(dir));
        }
    }
    return res;
}

/*
void Map::Summary()
{
    std::cout << "Summary: = = = = = = = = = = = = = = =\n";
    std::cout << "Road: " << mRoads.size() << std::endl;
    std::cout << "Junc: " << mJunctions.size() << std::endl;
    for(auto r : mRoads)
    {
        std::cout << "- - - - - - - - - - - - - - - - - - - -\n";
        std::cout << "road[" << r.iRoadId << "] length:" << r.dLength << std::endl;

        for(auto section : r.mSections)
        {
            std::cout << "\tsec[" << section.iSectionId << "] s:" << section.s << std::endl;

            for(auto lane : section.mLanes)
            {
                std::cout << "\t\tlane[" << lane.second.land_id << "] idx:" << lane.first << std::endl;

                std::cout << "\t\t\tprev: ";
                for(auto link : lane.second.predecessors)
                {
                    std::cout << "[" << link << "] ";
                }
                std::cout << std::endl;

                std::cout << "\t\t\tnext: ";
                for(auto link : lane.second.successors)
                {
                    std::cout << "[" << link << "] ";
                }
                std::cout << std::endl;
            }
        }
    }

    for(auto j : mJunctions)
    {
        std::cout << "- - - - - - - - - - - - - - - - - - - -\n";
        std::cout << "junc[" << j.iJunctionId << "]" << std::endl;
        for(auto c : j.mRoadLinks)
        {
            std::cout << "\troad[" << std::get<0>(c.first) << "] --> road[" << std::get<1>(c.first) << "]\n";

            for(auto k : c.second.vLaneLinks)
            {
                std::cout << "\t\t[" << k.iFromIndex << "] --> [" << k.iToIndex << "]\n";
            }
            std::cout << std::endl;
        }
    }
}
*/

/*
void Map::Trajectory(std::vector<std::pair<unsigned int, int>> sequences)
{
    /*
    std::vector<Pose> res;

    for(int i = 0; i < sequences.size()-1; i++)
    {
        std::pair<int, int> k;
        for(auto & j : mJunctions)
        {
            if(j.Check({sequences[i].first, sequences[i+1].first}))
            {
                k = j.GetLink({sequences[i].first, sequences[i+1].first});
                break;
            }
        }

        auto road_traj = mRoads[sequences[i].first].Trajectory(sequences[i].second, k.first);
        res.insert(res.end(), road_traj.begin(), road_traj.end());


    }

    std::vector<Pose> res;
    for(auto x : sequences)
    {
        unsigned road_id = x.first;
        int lane_idx = x.second;

        for(auto & section : mRoads[road_id].mSections)
        {
            auto lane_pose = section.GetLanePoseByIndex(lane_idx);
            res.insert(res.end(), lane_pose.begin(), lane_pose.end());

            auto successors = section.mLanes[lane_idx].successors;
            if(!successors.empty())
                lane_idx = section.mLanes[lane_idx].successors.front();
        }
    }

}
*/

/*
bool Map::OnRequest(HDMap::LocalMap::Request &request, HDMap::LocalMap::Response &response)
{

    //region 发送LocalMap
    auto func = [&]()
    {
        if(mRecord.curr_rid != -1)
        {
            auto ps = mRoads[mRecord.curr_rid].GetLanePosesByDirection(mRecord.curr_road_dir);

            for(auto & p : ps)
            {
                HDMap::Pose2DArray array;
                for(auto & x : p)
                {
                    geometry_msgs::Pose2D pose2D;
                    pose2D.x = x.GetPosition().x;
                    pose2D.y = x.GetPosition().y;
                    pose2D.theta = x.GetAngle().ToYaw();
                    array.lane.emplace_back(pose2D);
                }
                response.curr_road.lanes.emplace_back(array);
            }
        }

        if(mRecord.jid != -1)
        {
            for(auto & j : mJunctions[mRecord.jid].mRoadLinks)
            {
                if(j.first.first == mRecord.curr_rid and j.first.second == mRecord.next_rid)
                {
                    for(auto & i : j.second.vLaneLinks)
                    {
                        response.junction.in.emplace_back(i.iFromIndex);
                        response.junction.out.emplace_back(i.iToIndex);
                    }

                    auto ps = j.second.GetAllPose();
                    for(auto & p : ps)
                    {
                        HDMap::Pose2DArray array;
                        for(auto & x : p)
                        {
                            geometry_msgs::Pose2D pose2D;
                            pose2D.x = x.GetPosition().x;
                            pose2D.y = x.GetPosition().y;
                            pose2D.theta = x.GetAngle().ToYaw();
                            array.lane.emplace_back(pose2D);
                        }
                        response.junction.conns.emplace_back(array);
                    }
                }
            }
        }

        if(mRecord.next_rid != -1)
        {
            auto ps = mRoads[mRecord.next_rid].GetLanePosesByDirection(mRecord.next_road_dir);

            for(auto & p : ps)
            {
                HDMap::Pose2DArray array;
                for(auto & x : p)
                {
                    geometry_msgs::Pose2D pose2D;
                    pose2D.x = x.GetPosition().x;
                    pose2D.y = x.GetPosition().y;
                    pose2D.theta = x.GetAngle().ToYaw();
                    array.lane.emplace_back(pose2D);
                }
                response.next_road.lanes.emplace_back(array);
            }
        }
        return true;
    };
    //endregion

    ROS_INFO("Map::OnRequest: from local map [%f\t%f]", request.x, request.y);

    if(!request.has_local_map)
    {
        response.need_update = true;
        return func();
    }

    double t1 = 100000;
    if(mRecord.curr_rid != -1)
    {
        t1 = Vector2d::SegmentDistance(mRoads[mRecord.curr_rid].GetStartPose(mRecord.curr_road_dir).GetPosition(),
                                       mRoads[mRecord.curr_rid].GetEndPose(mRecord.curr_road_dir).GetPosition(),
                                       {request.x, request.y});
    }
    double t2 = 100000;
    if(mRecord.jid != -1)
    {
        t2 = Vector2d::SegmentDistance(mRoads[mRecord.curr_rid].GetEndPose(mRecord.curr_road_dir).GetPosition(),
                                       mRoads[mRecord.next_rid].GetStartPose(mRecord.next_road_dir).GetPosition(),
                                       {request.x, request.y});
    }
    double t3 = 100000;
    if(mRecord.next_rid != -1)
    {
        t3 = Vector2d::SegmentDistance(mRoads[mRecord.next_rid].GetStartPose(mRecord.next_road_dir).GetPosition(),
                                       mRoads[mRecord.next_rid].GetEndPose(mRecord.next_road_dir).GetPosition(),
                                       {request.x, request.y});
    }

    if(t1 < t2 and t1 < t3)
    {
        ROS_INFO("Map::OnRequest: in curr_road, no need to update local map");
        response.need_update = false;
    }
    if(t2 < t1 and t2 < t3)
    {
        ROS_INFO("Map::OnRequest: in junction, no need to update local map");
        response.need_update = false;
    }
    if(t3 < t1 and t3 < t2)
    {
        ROS_INFO("Map::OnRequest: in next_road, attempt to update local map");
        response.need_update = true;
    }

    if(std::min(std::min(t1, t2), t3) > 50.0)
    {
        ROS_ERROR("Current position is too far away from the local map!!!");
    }

    if(response.need_update)
    {
        mRecord.curr_routing_idx++;

        if(mRecord.curr_routing_idx < mBestRouting.size())
        {
            mRecord.curr_rid = mBestRouting[mRecord.curr_routing_idx].first;
            mRecord.curr_road_dir = mBestRouting[mRecord.curr_routing_idx].second;
        }
        else
        {
            ROS_ERROR("Reaching the end road of the routing");
            mRecord.curr_rid = -1;
        }

        if(mRecord.curr_routing_idx + 1 < mBestRouting.size())
        {
            mRecord.next_rid = mBestRouting[mRecord.curr_routing_idx+1].first;
            mRecord.next_road_dir = mBestRouting[mRecord.curr_routing_idx+1].second;
            mRecord.jid = mRoads[mRecord.curr_rid].AdjacentJid(mRecord.curr_road_dir);
        }
        else
        {
            mRecord.next_rid = -1;
            mRecord.jid = -1;
        }
    }

    return func();
}
*/

std::shared_ptr<SubRoad> Map::Locate(const Vector2d &v)
{
    double min_dist = 100000;
    unsigned int road_id = 0;

    for(auto & x : mRoadPtrs)
    {
        double t = Vector2d::SegmentDistance(x->GetStartPose(1).GetPosition(),
                                             x->GetEndPose(1).GetPosition(),
                                             v);
        if( t < min_dist)
        {
            min_dist = t;
            road_id = x->mRoadId;
        }
    }

    auto p1 = mRoadPtrs[road_id]->Locate(v);
    return mRoadPtrs[road_id]->GetSubRoadPtr(p1.second);
}

boost::property_tree::ptree Map::ToXML()
{
    pt::ptree p_map;

    for(auto & m : mRoadPtrs)
        p_map.add_child("hdmap.roads.road", m->ToXML());

    for(auto & m : mJuncPtrs)
        p_map.add_child("hdmap.junctions.junction", m->ToXML());

    return p_map;
}

void Map::FromXML(const pt::ptree &p)
{
    for(auto & r : p.get_child("hdmap.roads"))
    {
        RoadPtr pRoad(new Road());
        pRoad->FromXML(r.second);
        mRoadPtrs.emplace_back(pRoad);
    }
    Road::ROAD_ID = mRoadPtrs.back()->mRoadId + 1;

    for(auto & j : p.get_child("hdmap.junctions"))
    {
        JuncPtr pJunc(new Junction());
        pJunc->FromXML(j.second);
        mJuncPtrs.emplace_back(pJunc);
    }
}

RoadPtr Map::GetRoadPtrById(unsigned int _road_id)
{
    for(auto & x : mRoadPtrs)
    {
        if(x->mRoadId == _road_id)
            return x;
    }
    return hdmap::RoadPtr();
}

JuncPtr Map::GetJuncPtrById(unsigned int _junc_id)
{
    for(auto & x : mJuncPtrs)
    {
        if(x->mJunctionId == _junc_id)
            return x;
    }
    return hdmap::JuncPtr();
}

