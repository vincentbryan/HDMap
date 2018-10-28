//
// Created by vincent on 18-10-9.
//

#include <boost/property_tree/ptree.hpp>
#include <algorithm>
#include <LocalMapRequest.h>
#include "Map.h"

using namespace hdmap;
namespace pt = boost::property_tree;

Map::Map() : mCurrPose(Pose()),
             mPrevPose(Pose()),
             mPrevSection(LaneSection()),
             mCurrSection(LaneSection())
{}

void Map::SetSender(std::shared_ptr<Sender> _sender)
{
    pSender = _sender;
}

void Map::SetCurrPose(const Pose &pose)
{
    mCurrPose = pose;
}

Pose Map::GetCurrPose() const
{
    return mCurrPose;
}

void Map::SetPrevPose(const Pose &pose)
{
    mPrevPose = pose;
}

Pose Map::GetPrevPose() const
{
    return mPrevPose;
}

LaneSection Map::GetCurrentSection() const
{
    return mCurrSection;
}

Junction Map::GetCurrentJunction() const
{
    return mJunctions.back();
}

std::vector<LaneSection> Map::GetAllSection()
{
    std::vector<LaneSection> res;
    for(auto road : mRoads)
    {
        for(auto sec : road.mSections)
        {
            res.emplace_back(sec);
        }
    }
    return res;
}

std::vector<Junction> Map::GetAllJunction()
{
    return mJunctions;
}

void Map::SetStartPoint(const Vector2d &v)
{
    mStartPoint = v;
    pSender->AddStartPoint(mStartPoint);
}

Vector2d Map::GetStartPoint() const
{
    return mStartPoint;
}

void Map::SetEndPoint(const Vector2d &v)
{
    mEndPoint = v;
    pSender->AddEndPoint(mEndPoint);
}

Vector2d Map::GetEndPoint() const
{
    return mEndPoint;
}

void Map::StartRoad(const Pose & _start_pose)
{
    mRoads.emplace_back(Road());
    mPrevSection.s = 0;
    mPrevSection.mReferLine = Bezier({0,0,0}, {0,0,0}, 0, 0);
    mPrevSection.mLanes.clear();
    mCurrSection = mPrevSection;
    mCurrPose = _start_pose;
}

void Map::EndRoad()
{
    if(!mRoads.empty())
    {
        mRoads.back().dLength = mCurrSection.s + mCurrSection.mReferLine.Length();
    }
}

void Map::StartSection(std::vector<std::tuple<int, double, double>> new_lane, std::vector<std::pair<int, int>>links)
{
    assert(!mRoads.empty());

    auto section_id_next = CalcuSectionId(mRoads.size()-1, mRoads.back().mSections.size());
    mCurrSection.iSectionId = section_id_next;
    mCurrSection.s = mPrevSection.s + mPrevSection.mReferLine.Length();
    mCurrSection.mLanes.clear();

    vTempLane = new_lane;
    vTempLink = links;
}

void Map::EndSection(const Pose & p, double _ctrl_len1, double _ctrl_len2)
{
    //Update Pose
    mPrevPose = mCurrPose;
    mCurrPose = p;

    mCurrSection.mReferLine = Bezier(mPrevPose, mCurrPose, _ctrl_len1, _ctrl_len2);
    mCurrSection.s = mPrevSection.s + mPrevSection.mReferLine.Length();
    mCurrSection.Clear();

    for(auto x : vTempLane)
    {
        auto idx = std::get<0>(x);
        auto start_width = std::get<1>(x);
        auto end_width = std::get<2>(x);
        auto id = CalcuLaneId(mCurrSection.iSectionId, idx);
        mCurrSection.AddLane(idx, id, start_width, end_width);
    }

    for(auto x : vTempLink)
    {
        auto a = std::get<0>(x);
        auto b = std::get<1>(x);
        mRoads.back().mSections.back().mLanes[a].AddSuccessors(b);
        mCurrSection.mLanes[b].AddPredecessor(a);
    }

    mRoads.back().mSections.emplace_back(mCurrSection);

    mPrevSection = mCurrSection;
}

void Map::AddSignal(Signal s)
{
    mRoads.back().mSignals.emplace_back(s);
}


unsigned int Map::CalcuSectionId(unsigned int road, unsigned int section)
{
    return road * 10 + section;
}

unsigned int Map::CalcuLaneId(unsigned int section, int lane)
{
    return section*10 + 5 + lane;
}

void Map::StartJunction()
{
    mJunctions.emplace_back(Junction());
}

void Map::AddConnection(unsigned int from_road, int from_lane_idx,
                          unsigned int to_road, int to_lane_idx,
                          double _ctrl_len1, double _ctrl_len2)
{
    Pose start_pose, end_pose;

    if(from_lane_idx >= 0)
        start_pose = mRoads[from_road].mSections.back().GetLanePoseByIndex(from_lane_idx).back();
    else
        start_pose = mRoads[from_road].mSections.front().GetLanePoseByIndex(from_lane_idx).back();

    if(to_lane_idx >= 0)
        end_pose = mRoads[to_road].mSections.front().GetLanePoseByIndex(to_lane_idx).front();
    else
        end_pose = mRoads[to_road].mSections.back().GetLanePoseByIndex(to_lane_idx).front();

    mJunctions.back().AddConnection(
         from_road, from_lane_idx, start_pose,
         to_road, to_lane_idx, end_pose,
         _ctrl_len1, _ctrl_len2
    );
    
    if(from_lane_idx > 0)
        mRoads[from_road].SetNextJid(mJunctions.back().iJunctionId);
    else
        mRoads[from_road].SetPrevJid(mJunctions.back().iJunctionId);
    
    if(to_lane_idx > 0)
        mRoads[to_road].SetPrevJid(mJunctions.back().iJunctionId);
    else
        mRoads[to_road].SetNextJid(mJunctions.back().iJunctionId);
}


void Map::EndJunction()
{}


void Map::Load(const std::string &file_name)
{
    try
    {
        pt::ptree tree;
        pt::read_xml(file_name, tree);

        //region AddRoads
        for(auto road : tree.get_child("hdmap.roads"))
        {
            Road oRoad;
            oRoad.iRoadId = road.second.get<int>("<xmlattr>.id");
            oRoad.dLength = road.second.get<double>("<xmlattr>.length");
            oRoad.SetPrevJid(road.second.get<int>("<xmlattr>.prev_jid"));
            oRoad.SetNextJid(road.second.get<int>("<xmlattr>.next_jid"));

            for(auto section : road.second.get_child(""))
            {
                if(section.first == "lanesection")
                {
                    LaneSection oSection;

                    //region AddSection
                    for(auto section_child : section.second.get_child(""))
                    {
                        if(section_child.first == "<xmlattr>")
                        {
                            oSection.iSectionId = section_child.second.get<int>("id");
                            oSection.s = section_child.second.get<double>("s");
                            oSection.most_left_lane_idx = section_child.second.get<int>("left_idx");
                            oSection.most_right_lane_idx = section_child.second.get<int>("right_idx");
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

                            oSection.mReferLine = Bezier(start_pose, end_pose, len1, len2);

                        }

                        if(section_child.first == "offset")
                        {
                            auto y0 = section_child.second.get<double>("y0");
                            auto range = section_child.second.get<double>("x1");
                            auto y1 = section_child.second.get<double>("y1");

                            oSection.mLaneOffset = CubicFunction(y0, range, y1);
                        }

                        if(section_child.first == "lane")
                        {
                            auto lane_idx = section_child.second.get<int>("<xmlattr>.idx");
                            Lane lane;
                            lane.land_id = section_child.second.get<int>("<xmlattr>.id");

                            //TODO
//                        lane.type = section_child.second.get<std::string>("<xmlattr>.type");

                            auto y0 = section_child.second.get<double>("offset.y0");
                            auto x1 = section_child.second.get<double>("offset.x1");
                            auto y1 = section_child.second.get<double>("offset.y1");
                            lane.width = CubicFunction(y0, x1, y1);

                            //Add link
                            for(auto link : section_child.second.get_child(""))
                            {
                                if(link.first == "predecessor")
                                {
                                    int n = atoi(link.second.data().c_str());
                                    lane.predecessors.emplace_back(n);
                                }
                                if(link.first == "successors")
                                {
                                    int n = atoi(link.second.data().c_str());
                                    lane.successors.emplace_back(n);
                                }
                            }
                            oSection.mLanes.insert({lane_idx, lane});
                        }
                        //endregion
                    };
                    //endregion
                    oRoad.mSections.emplace_back(oSection);
                }

                if(section.first == "signals")
                {
                    Signal s;
                    s.direction = section.second.get<int>("direction");
                    s.position.x = section.second.get<double>("x");
                    s.position.y = section.second.get<double>("y");
                    s.type = section.second.get<std::string>("type");
                    s.info = section.second.get<std::string>("info");
                    oRoad.mSignals.emplace_back(s);
                }
            }
            mRoads.emplace_back(oRoad);
        }
        mPrevSection = mCurrSection = mRoads.back().mSections.back();
        Road::ROAD_ID = mRoads.back().iRoadId + 1;
        //endregion

        //region AddJunctions
        for(auto junction : tree.get_child("hdmap.junctions"))
        {
            Junction oJunction;
            oJunction.iJunctionId = junction.second.get<int>("<xmlattr>.id");

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
//                    oJunction.mRoadLinks.insert(std::pair<std::pair<unsigned int, unsigned int>, RoadLink>(std::pair(from_road_id, to_road_id), road_link));
                    oJunction.mRoadLinks[std::pair<unsigned int, unsigned int>(from_road_id, to_road_id)] = road_link;
                    /*
                    Pose start_pose = { link.second.get<double>("refer_line.start_pose.x"),
                                        link.second.get<double>("refer_line.start_pose.y"),
                                        link.second.get<double>("refer_line.start_pose.direction")};
                    Pose end_pose = { link.second.get<double>("refer_line.end_pose.x"),
                                      link.second.get<double>("refer_line.end_pose.y"),
                                      link.second.get<double>("refer_line.end_pose.direction")};
                    auto len1 = link.second.get<double>("refer_line.ctrl_len1");
                    auto len2 = link.second.get<double>("refer_line.ctrl_len2");
                    Junction::LaneLink c(from_id, to_id, Bezier(start_pose, end_pose, len1, len2));
                    oJunction.mConnection.emplace_back(c);
                     */
                }
            }
            mJunctions.emplace_back(oJunction);
        }
        for(auto & r : mRoads) r.InitSubRoad();
    }
    catch (std::exception &e)
    {
        std::cout << "Error: " << e.what() << std::endl;
    }
}


void Map::Save(const std::string &file_name)
{
    try
    {
        pt::ptree tree;

        pt::ptree p_road;
        for(auto & x : mRoads)
        {
            p_road.add("<xmlattr>.id", x.iRoadId);
            p_road.add("<xmlattr>.length", x.dLength);
            p_road.add("<xmlattr>.prev_jid", x.GetPrevJid());
            p_road.add("<xmlattr>.next_jid", x.GetNextJid());
            pt::ptree p_sec;
            for(auto & sec : x.mSections)
            {
                p_sec.add("<xmlattr>.id", sec.iSectionId);
                p_sec.add("<xmlattr>.s", sec.s);

                p_sec.add("<xmlattr>.left_idx", sec.most_left_lane_idx);
                p_sec.add("<xmlattr>.right_idx",sec.most_right_lane_idx);

                p_sec.add("refer_line.<xmlattr>.type", "bezier");
                p_sec.add("refer_line.<xmlattr>.s", sec.s);
                p_sec.add("refer_line.start_pose.x", sec.mReferLine.start_pose.x);
                p_sec.add("refer_line.start_pose.y", sec.mReferLine.start_pose.y);
                p_sec.add("refer_line.start_pose.direction", sec.mReferLine.start_pose.direction);
                p_sec.add("refer_line.end_pose.x", sec.mReferLine.end_pose.x);
                p_sec.add("refer_line.end_pose.y", sec.mReferLine.end_pose.y);
                p_sec.add("refer_line.end_pose.direction", sec.mReferLine.end_pose.direction);
                p_sec.add("refer_line.ctrl_len1", sec.mReferLine.ctrl_len1);
                p_sec.add("refer_line.ctrl_len2", sec.mReferLine.ctrl_len2);

                p_sec.add("offset.<xmlattr>.type", "cubic_function");
                p_sec.add("offset.x0", sec.mLaneOffset.x0);
                p_sec.add("offset.y0", sec.mLaneOffset.y0);
                p_sec.add("offset.x1", sec.mLaneOffset.x1);
                p_sec.add("offset.y1", sec.mLaneOffset.y1);

                pt::ptree p_lane;
                for(auto & k : sec.mLanes)
                {
                    p_lane.add("<xmlattr>.idx", k.first);
                    p_lane.add("<xmlattr>.id", k.second.land_id);
                    p_lane.add("<xmlattr>.type", "Driving");
                    p_lane.add("offset.<xmlattr>.type", "cubic_function");
                    p_lane.add("offset.<xmlattr>.s", 0);
                    p_lane.add("offset.x0", k.second.width.x0);
                    p_lane.add("offset.y0", k.second.width.y0);
                    p_lane.add("offset.x1", k.second.width.x1);
                    p_lane.add("offset.y1", k.second.width.y1);

                    for(auto & s : k.second.predecessors)
                        p_lane.add("predecessor", s);

                    for(auto & s : k.second.successors)
                        p_lane.add("successors", s);

                    p_sec.add_child("lane", p_lane);
                    p_lane.clear();
                }
                p_road.add_child("lanesection", p_sec);
                p_sec.clear();
            }

            pt::ptree p_sig;
            for(auto & sig : x.mSignals)
            {
                p_sig.add("x", sig.position.x);
                p_sig.add("y", sig.position.y);
                p_sig.add("direction", sig.direction);
                p_sig.add("type", sig.type);
                p_sig.add("info", sig.info);
                p_road.add_child("signals", p_sig);
                p_sig.clear();
            }

            tree.add_child("hdmap.roads.road", p_road);
            p_road.clear();
        }

        pt::ptree p_junc;
        for(auto & junc : mJunctions)
        {

            p_junc.add("<xmlattr>.id",junc.iJunctionId);

            pt::ptree p_road_link;
            for(auto & road_link : junc.mRoadLinks)
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
        pt::write_xml(file_name, tree);
    }
    catch (std::exception &e)
    {
        std::cout << "Error: " << e.what() << std::endl;
    }

}


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


void Map::Send()
{
    for(auto & m : mRoads) m.Send(*pSender);
    for(auto & j : mJunctions) j.Send(*pSender);
}


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
     */
}

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

std::vector<std::shared_ptr<SubRoad>> Map::AdjacentRoadInfo(std::shared_ptr<SubRoad> pSubRoad)
{
    std::vector<std::shared_ptr<SubRoad>> res;

    int jid = pSubRoad->iNextJid;
    if(jid == -1) return res;

    for(auto const & m : mJunctions[jid].mRoadLinks)
    {
        if(m.first.first == pSubRoad->iRoadId)
        {
            int dir = m.second.vLaneLinks.front().iToIndex > 0 ? 1 : -1;

            res.emplace_back(mRoads[m.first.second].GetSubRoadPtr(dir));
        }
    }
    return res;
}


std::shared_ptr<SubRoad> Map::Locate(const Vector2d &v)
{
    double min_dist = 100000;
    unsigned int road_id = 0;

    for(auto & x : mRoads)
    {
        double t = Vector2d::SegmentDistance(x.GetStartPose(1).GetPosition(),
                                             x.GetEndPose(1).GetPosition(),
                                             v);
        if( t < min_dist)
        {
            min_dist = t;
            road_id = x.iRoadId;
        }
    }

    auto p1 = mRoads[road_id].Locate(v);
    return mRoads[road_id].GetSubRoadPtr(p1.second);
}


std::vector<unsigned int> Map::GetRoadId()
{
    std::vector<unsigned int> res;
    for(auto const & x : mRoads)
    {
        res.emplace_back(x.iRoadId);
    }
    return res;
}
