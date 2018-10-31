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
        /*
        pt::ptree p_road;
        for(auto & x : mRoadPtrs)
        {
            p_road.add("<xmlattr>.id", x->iRoadId);
            p_road.add("<xmlattr>.length", x->dLength);
            p_road.add("<xmlattr>.prev_jid", x->GetPrevJid());
            p_road.add("<xmlattr>.next_jid", x->GetNextJid());
            pt::ptree p_sec;
            for(auto & sec : x->mSecPtrs)
            {
                p_sec.add("<xmlattr>.id", sec->iSectionId);
                p_sec.add("<xmlattr>.s", sec->s);

                p_sec.add("<xmlattr>.left_idx", sec->most_left_lane_idx);
                p_sec.add("<xmlattr>.right_idx",sec->most_right_lane_idx);

                p_sec.add("refer_line.<xmlattr>.type", "bezier");
                p_sec.add("refer_line.<xmlattr>.s", sec->s);
                p_sec.add("refer_line.start_pose.x", sec->mReferLine.start_pose.x);
                p_sec.add("refer_line.start_pose.y", sec->mReferLine.start_pose.y);
                p_sec.add("refer_line.start_pose.direction", sec->mReferLine.start_pose.direction);
                p_sec.add("refer_line.end_pose.x", sec->mReferLine.end_pose.x);
                p_sec.add("refer_line.end_pose.y", sec->mReferLine.end_pose.y);
                p_sec.add("refer_line.end_pose.direction", sec->mReferLine.end_pose.direction);
                p_sec.add("refer_line.ctrl_len1", sec->mReferLine.ctrl_len1);
                p_sec.add("refer_line.ctrl_len2", sec->mReferLine.ctrl_len2);

                p_sec.add("offset.<xmlattr>.type", "cubic_function");
                p_sec.add("offset.x0", sec->mLaneOffset.x0);
                p_sec.add("offset.y0", sec->mLaneOffset.y0);
                p_sec.add("offset.x1", sec->mLaneOffset.x1);
                p_sec.add("offset.y1", sec->mLaneOffset.y1);

                pt::ptree p_lane;
                for(auto & k : sec->mLanes)
                {
                    p_lane.add("<xmlattr>.idx", k.first);
                    p_lane.add("<xmlattr>.id", k.second.land_id);
                    p_lane.add("<xmlattr>.type", "Driving");
                    p_lane.add("offset.<xmlattr>.type", "cubic_function");
                    p_lane.add("offset.<xmlattr>.s", 0);
                    p_lane.add("offset.x0", k.second.offset.x0);
                    p_lane.add("offset.y0", k.second.offset.y0);
                    p_lane.add("offset.x1", k.second.offset.x1);
                    p_lane.add("offset.y1", k.second.offset.y1);

                    for(auto & s : k.second.predecessors)
                        p_lane.add("predecessors", s);

                    for(auto & s : k.second.successors)
                        p_lane.add("successors", s);

                    p_sec.add_child("lane", p_lane);
                    p_lane.clear();
                }
                p_road.add_child("lanesection", p_sec);
                p_sec.clear();
            }

            pt::ptree p_sig;
            for(auto & sig : x->mSigPtrs)
            {
                p_sig.add("x", sig->position.x);
                p_sig.add("y", sig->position.y);
                p_sig.add("direction", sig->direction);
                p_sig.add("type", sig->type);
                p_sig.add("info", sig->info);
                p_road.add_child("signals", p_sig);
                p_sig.clear();
            }

            tree.add_child("hdmap.roads.road", p_road);
            p_road.clear();
        }

        pt::ptree p_junc;
        for(auto & junc : mJuncPtrs)
        {

            p_junc.add("<xmlattr>.id",junc->iJunctionId);

            pt::ptree p_road_link;
            for(auto & road_link : junc->mRoadLinks)
            {

                p_road_link.add("<xmlattr>.from_road_id", road_link.first.first);
                p_road_link.add("<xmlattr>.to_road_id", road_link.first.second);

                pt::ptree p_lane_link;
                for(auto & lane_link : road_link.second.vLaneLinks)
                {

                    p_lane_link.add("<xmlattr>.from_lane_idx", lane_link.iFromIndex);
                    p_lane_link.add("<xmlattr>.to_lane_idx", lane_link.iToIndex);

                    p_lane_link.add("refer_line.<xmlattr>.type", "bezier");
                    p_lane_link.add("refer_line.<xmlattr>.s", 0);
                    p_lane_link.add("refer_line.start_pose.x", lane_link.mReferLine.start_pose.x);
                    p_lane_link.add("refer_line.start_pose.y", lane_link.mReferLine.start_pose.y);
                    p_lane_link.add("refer_line.start_pose.direction", lane_link.mReferLine.start_pose.direction);
                    p_lane_link.add("refer_line.end_pose.x", lane_link.mReferLine.end_pose.x);
                    p_lane_link.add("refer_line.end_pose.y", lane_link.mReferLine.end_pose.y);
                    p_lane_link.add("refer_line.end_pose.direction", lane_link.mReferLine.end_pose.direction);
                    p_lane_link.add("refer_line.ctrl_len1", lane_link.mReferLine.ctrl_len1);
                    p_lane_link.add("refer_line.ctrl_len2", lane_link.mReferLine.ctrl_len2);

                    p_road_link.add_child("lane-link", p_lane_link);
                    p_lane_link.clear();
                }

                p_junc.add_child("road-link", p_road_link);
                p_road_link.clear();

            }

            tree.add_child("hdmap.junctions.junction", p_junc);
            p_junc.clear();

        }
        */
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
        /*
        //region AddRoads
        for(auto road : tree.get_child("hdmap.roads"))
        {
            RoadPtr pRoad(new Road());
            pRoad->mRoadId = road.second.get<int>("<xmlattr>.id");
            pRoad->mLength = road.second.get<double>("<xmlattr>.length");
            pRoad->mPrevJid = road.second.get<int>("<xmlattr>.prev_jid");
            pRoad->mNextJid = road.second.get<int>("<xmlattr>.next_jid");

            for(auto section : road.second.get_child(""))
            {
                if(section.first == "lanesection")
                {
                    SecPtr pSection(new LaneSection());

                    //region AddSection
                    for(auto section_child : section.second.get_child(""))
                    {
                        if(section_child.first == "<xmlattr>")
                        {
                            pSection->mSectionId = section_child.second.get<int>("id");
                            pSection->mStartS = section_child.second.get<double>("s");
                            pSection->mLeftBoundary = section_child.second.get<int>("left_idx");
                            pSection->mRightBoundary = section_child.second.get<int>("right_idx");
                        }

                        if(section_child.first == "refer_line")
                        {
                            auto type = section_child.second.get<std::string>("<xmlattr>.type");
                            auto s = section_child.second.get<double>("<xmlattr>.s");

                            Pose start_pose = { section_child.second.get<double>("start_pose.x"),
                                                section_child.second.get<double>("start_pose.y"),
                                                section_child.second.get<double>("start_pose.direction")};
                            Pose end_pose = { section_child.second.get<double>("end_pose.x"),
                                              section_child.second.get<double>("end_pose.y"),
                                              section_child.second.get<double>("end_pose.direction")};
                            auto len1 = section_child.second.get<double>("ctrl_len1");
                            auto len2 = section_child.second.get<double>("ctrl_len2");

                            pSection->mReferLine = Bezier(start_pose, end_pose, len1, len2);
                        }

                        if(section_child.first == "offset")
                        {
                            auto y0 = section_child.second.get<double>("y0");
                            auto range = section_child.second.get<double>("x1");
                            auto y1 = section_child.second.get<double>("y1");

                            pSection->mLaneOffset = CubicFunction(y0, range, y1);
                        }

                        if(section_child.first == "lane")
                        {
                            auto lane_idx = section_child.second.get<int>("<xmlattr>.idx");
                            Lane lane;
                            lane.mLandId = section_child.second.get<int>("<xmlattr>.id");

                            auto y0 = section_child.second.get<double>("offset.y0");
                            auto x1 = section_child.second.get<double>("offset.x1");
                            auto y1 = section_child.second.get<double>("offset.y1");
                            lane.mOffset = CubicFunction(y0, x1, y1);

                            //Add link
                            for(auto link : section_child.second.get_child(""))
                            {
                                if(link.first == "predecessors")
                                {
                                    int n = atoi(link.second.data().c_str());
                                    lane.mPredecessors.emplace_back(n);
                                }
                                if(link.first == "successors")
                                {
                                    int n = atoi(link.second.data().c_str());
                                    lane.mSuccessors.emplace_back(n);
                                }
                            }
                            pSection->mLanes.insert({lane_idx, lane});
                        }
                        //endregion
                    };
                    //endregion
                    pRoad->mSecPtrs.emplace_back(pSection);
                }

                if(section.first == "signals")
                {
                    SigPtr s(new Signal());
                    s->mDirection = section.second.get<int>("direction");
                    s->mPosition.x = section.second.get<double>("x");
                    s->mPosition.y = section.second.get<double>("y");
                    s->mType = section.second.get<std::string>("type");
                    s->mInfo = section.second.get<std::string>("info");
                    pRoad->mSigPtrs.emplace_back(s);
                }
            }
            mRoadPtrs.emplace_back(pRoad);
        }
        Road::ROAD_ID = mRoadPtrs.back()->mRoadId + 1;
        //endregion

        //region AddJunctions
        for(auto junction : tree.get_child("hdmap.junctions"))
        {
            JuncPtr pJunction(new Junction());
            pJunction->mJunctionId = junction.second.get<int>("<xmlattr>.id");

            for(auto link : junction.second.get_child(""))
            {
                if(link.first == "road-link")
                {
                    auto from_road_id = link.second.get<int>("<xmlattr>.from_road_id");
                    auto to_road_id = link.second.get<int>("<xmlattr>.to_road_id");

                    RoadLink road_link(from_road_id, to_road_id);

                    for(auto lane_link : link.second.get_child(""))
                    {
                        if(lane_link.first == "lane-link")
                        {
                            auto from_lane_idx = lane_link.second.get<int>("<xmlattr>.from_lane_idx");
                            auto to_lane_idx = lane_link.second.get<int>("<xmlattr>.to_lane_idx");
                            Pose start_pose = { lane_link.second.get<double>("refer_line.start_pose.x"),
                                                lane_link.second.get<double>("refer_line.start_pose.y"),
                                                lane_link.second.get<double>("refer_line.start_pose.direction")};
                            Pose end_pose = { lane_link.second.get<double>("refer_line.end_pose.x"),
                                              lane_link.second.get<double>("refer_line.end_pose.y"),
                                              lane_link.second.get<double>("refer_line.end_pose.direction")};
                            auto len1 = lane_link.second.get<double>("refer_line.ctrl_len1");
                            auto len2 = lane_link.second.get<double>("refer_line.ctrl_len2");
                            road_link.AddLaneLink(from_lane_idx, to_lane_idx, Bezier(start_pose, end_pose, len1, len2));
                        }
                    }
                    pJunction->mRoadLinks[std::pair<unsigned int, unsigned int>(from_road_id, to_road_id)] = road_link;
                }
            }
            mJuncPtrs.emplace_back(pJunction);
        }
        //endregion

        for(auto & r : mRoadPtrs) r->InitSubRoad();
*/
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

