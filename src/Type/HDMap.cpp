//
// Created by vincent on 18-10-9.
//

#include <boost/property_tree/ptree.hpp>
#include <geos_c.h>
#include "HDMap.h"

using namespace hdmap;
namespace pt = boost::property_tree;

HDMap::HDMap() : mCurrPose(Pose()),
                 mPrevPose(Pose()),
                 mPrevSection(LaneSection()),
                 mCurrSection(LaneSection())
{}

void HDMap::StartRoad()
{
    mRoads.emplace_back(Road());
    mPrevSection.s = 0;
    mPrevSection.mReferLine = Bezier({0,0,0}, {0,0,0}, 0, 0);
    mPrevSection.mLanes.clear();
    mCurrSection = mPrevSection;
}

void HDMap::EndRoad()
{
    if(!mRoads.empty())
    {
        mRoads.back().length = mCurrSection.s + mCurrSection.mReferLine.Length();
    }
}

void HDMap::StartSection(std::vector<std::tuple<int, double, double>> new_lane, std::vector<std::pair<int, int>>links)
{
    assert(!mRoads.empty());

    auto section_id_next = CalcuSectionId(mRoads.size()-1, mRoads.back().mSections.size());
    mCurrSection.iSectionId = section_id_next;
    mCurrSection.s = mPrevSection.s + mPrevSection.mReferLine.Length();
    mCurrSection.mLanes.clear();

    vTempLane = new_lane;
    vTempLink = links;
}

void HDMap::EndSection(Pose p)
{
    //Update Pose
    mPrevPose = mCurrPose;
    mCurrPose = p;

    mCurrSection.mReferLine = Bezier(mPrevPose, mCurrPose);
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

unsigned int HDMap::CalcuSectionId(unsigned int road, unsigned int section)
{
    return road * 10 + section;
}

unsigned int HDMap::CalcuLaneId(unsigned int section, int lane)
{
    return section*10 + 5 + lane;
}

void HDMap::AddJunction()
{
    mJunctions.emplace_back(Junction());
}

void HDMap::AddConnection(unsigned int from_road, int from_lane_idx,
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
}

std::vector<LaneSection> HDMap::GetAllSection()
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

std::vector<Junction> HDMap::GetAllJunction()
{
    return mJunctions;
}

void HDMap::Load(const std::string &file_name)
{
    try
    {
        pt::ptree tree;
        pt::read_xml(file_name, tree);

        //region AddRoads
        for(auto road : tree.get_child("hdmap.roads"))
        {
            Road oRoad;
            oRoad.mRoadId = road.second.get<int>("<xmlattr>.id");
            oRoad.length = road.second.get<double>("<xmlattr>.length");

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
            }
            mRoads.emplace_back(oRoad);
        }
        mPrevSection = mCurrSection = mRoads.back().mSections.back();
        Road::ROAD_ID = mRoads.back().mRoadId + 1;
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
    }
    catch (std::exception &e)
    {
        std::cout << "Error: " << e.what() << std::endl;
    }
}

void HDMap::Summary()
{
    std::cout << "Summary: = = = = = = = = = = = = = = =\n";
    std::cout << "Road: " << mRoads.size() << std::endl;
    std::cout << "Junc: " << mJunctions.size() << std::endl;
    for(auto r : mRoads)
    {
        std::cout << "- - - - - - - - - - - - - - - - - - - -\n";
        std::cout << "road[" << r.mRoadId << "] length:" << r.length << std::endl;

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
