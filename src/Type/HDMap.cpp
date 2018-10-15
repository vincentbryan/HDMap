//
// Created by vincent on 18-10-9.
//

#include <boost/property_tree/ptree.hpp>
#include "HDMap.h"

using namespace hdmap;
namespace pt = boost::property_tree;

HDMap::HDMap() : mStartPose(Pose()),
                 mEndPose(Pose()),
                 mPrevSection(LaneSection()),
                 mCurrSection(LaneSection())
{}

void HDMap::AddRoad()
{
    mRoads.emplace_back(Road());
}

void HDMap::StartSection(std::vector<std::tuple<int, double, double>> new_lane, std::vector<std::pair<int, int>>links)
{
    assert(mRoads.size() >= 1);
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
    mEndPose = mStartPose;
    mStartPose = p;

    mCurrSection.mReferLine = Bezier(mEndPose, mStartPose);
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
        mPrevSection.mLanes[a].AddPredecessor(b);
        mCurrSection.mLanes[b].AddSuccessors(a);
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
                          unsigned int to_road, int to_lane_idx)
{
    auto from_section = mRoads[from_road].mSections.back();
    auto to_section = mRoads[to_road].mSections.front();

    mJunctions.back().AddConnection(
         from_section.GetLaneByIndex(from_lane_idx),
         from_section.GetLanePoseByIndex(from_lane_idx).back(),
         to_section.GetLaneByIndex(to_lane_idx),
         to_section.GetLanePoseByIndex(to_lane_idx).front()
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

/*
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
            oRoad.s = road.second.get<double>("<xmlattr>.s");

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
                        if(section_child.first == "refer_line" || section_child.first == "offset")
                        {
                            auto type = section_child.second.get<std::string>("<xmlattr>.type");
                            auto s = section_child.second.get<double>("<xmlattr>.s");
                            //TODO
                            if(type == "Line")
                            {
                                Pose start_pose = { section_child.second.get<double>("start_pose.x"),
                                                    section_child.second.get<double>("start_pose.y"),
                                                    section_child.second.get<double>("start_pose.yaw")};
                                Pose end_pose = { section_child.second.get<double>("end_pose.x"),
                                                  section_child.second.get<double>("end_pose.y"),
                                                  section_child.second.get<double>("end_pose.yaw")};

                                if(section_child.first == "refer_line")
                                    oSection.mReferLine.reset(new Line(s, start_pose, end_pose));
                                if(section_child.first == "offset")
                                    oSection.mLaneOffset.reset(new Line(s, start_pose, end_pose));
                            }
                        }
                        //region AddLane
                        if(section_child.first == "lane")
                        {
                            auto lane_idx = section_child.second.get<int>("<xmlattr>.idx");
                            Lane lane;
                            lane.land_id = section_child.second.get<int>("<xmlattr>.id");
                            //TODO
//                        lane.type = section_child.second.get<std::string>("<xmlattr>.type");

                            Pose start_pose = { section_child.second.get<double>("offset.start_pose.x"),
                                                section_child.second.get<double>("offset.start_pose.y"),
                                                section_child.second.get<double>("offset.start_pose.yaw")};
                            Pose end_pose = { section_child.second.get<double>("offset.end_pose.x"),
                                              section_child.second.get<double>("offset.end_pose.y"),
                                              section_child.second.get<double>("offset.end_pose.yaw")};
                            lane.width.reset(new Line(0, start_pose, end_pose));

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

            for(auto conn : junction.second.get_child(""))
            {
                if(conn.first == "connection")
                {
                    Junction::Connection connection;
                    connection.from_lane_id = conn.second.get<int>("<xmlattr>.from_lane_id");
                    connection.to_lane_id = conn.second.get<int>("<xmlattr>.to_lane_id");

                    Pose start_pose = { conn.second.get<double>("offset.start_pose.x"),
                                        conn.second.get<double>("offset.start_pose.y"),
                                        conn.second.get<double>("offset.start_pose.yaw")};
                    Pose end_pose = { conn.second.get<double>("offset.end_pose.x"),
                                      conn.second.get<double>("offset.end_pose.y"),
                                      conn.second.get<double>("offset.end_pose.yaw")};

                    connection.refer_line.reset(new Line(0, start_pose, end_pose));
                    oJunction.mConnection.emplace_back(connection);
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
 */