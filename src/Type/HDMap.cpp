//
// Created by vincent on 18-10-9.
//

#include <boost/property_tree/ptree.hpp>
#include "HDMap.h"

using namespace hdmap;
namespace pt = boost::property_tree;

HDMap::HDMap() : mCurrPose(Pose()),
                 mPrevPose(Pose()),
                 mPrevSection(LaneSection()),
                 mCurrSection(LaneSection())
{}

//region Setter And Getter
void HDMap::SetSender(std::shared_ptr<Sender> _sender)
{
    pSender = _sender;
}

void HDMap::SetCurrPose(const Pose &pose)
{
    mCurrPose = pose;
}

Pose HDMap::GetCurrPose() const
{
    return mCurrPose;
}

void HDMap::SetPrevPose(const Pose &pose)
{
    mPrevPose = pose;
}

Pose HDMap::GetPrevPose() const
{
    return mPrevPose;
}

LaneSection HDMap::GetCurrentSection() const
{
    return mCurrSection;
}

Junction HDMap::GetCurrentJunction() const
{
    return mJunctions.back();
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

void HDMap::SetStartPoint(const Vector2d &v)
{
    mStartPoint = v;
    pSender->AddStartPoint(mStartPoint);
}

Vector2d HDMap::GetStartPoint() const
{
    return mStartPoint;
}

void HDMap::SetEndPoint(const Vector2d &v)
{
    mEndPoint = v;
    pSender->AddEndPoint(mEndPoint);
}

Vector2d HDMap::GetEndPoint() const
{
    return mEndPoint;
}
//endregion

void HDMap::StartRoad(const Pose & _start_pose)
{
    mRoads.emplace_back(Road());
    mPrevSection.s = 0;
    mPrevSection.mReferLine = Bezier({0,0,0}, {0,0,0}, 0, 0);
    mPrevSection.mLanes.clear();
    mCurrSection = mPrevSection;

    mCurrPose = _start_pose;

    pSender->AddRoadId(mCurrPose, mRoads.back().iRoadId);
}

void HDMap::EndRoad()
{
    if(!mRoads.empty())
    {
        mRoads.back().dLength = mCurrSection.s + mCurrSection.mReferLine.Length();
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

void HDMap::EndSection(const Pose & p)
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

    pSender->AddSection(mCurrSection);
}

unsigned int HDMap::CalcuSectionId(unsigned int road, unsigned int section)
{
    return road * 10 + section;
}

unsigned int HDMap::CalcuLaneId(unsigned int section, int lane)
{
    return section*10 + 5 + lane;
}

void HDMap::StartJunction()
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
    mRoads[from_road].AddNextRoadId(to_road);
    mRoads[to_road].AddPrevRoadId(from_road);
}

void HDMap::EndJunction()
{
    pSender->AddJunction(mJunctions.back());
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
            oRoad.iRoadId = road.second.get<int>("<xmlattr>.id");
            oRoad.dLength = road.second.get<double>("<xmlattr>.length");

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

        for(auto & road : mRoads)
        {
            for(auto & sec : road.mSections)
            {
                pSender->AddSection(sec);
            }
        }

        for(auto & junc : mJunctions)
        {
            pSender->AddJunction(junc);
        }
    }
    catch (std::exception &e)
    {
        std::cout << "Error: " << e.what() << std::endl;
    }
}

void HDMap::Save(const std::string &file_name)
{
    try
    {
        pt::ptree tree;

        pt::ptree p_road;
        for(auto & x : mRoads)
        {

            p_road.add("<xmlattr>.id", x.iRoadId);
            p_road.add("<xmlattr>.length", x.dLength);

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

                    p_sec.add_child("lane", p_lane);
                    p_lane.clear();
                }
                p_road.add_child("lanesection", p_sec);
                p_sec.clear();
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

void HDMap::Summary()
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

void HDMap::GlobalPlanning()
{
    double min_dist = 100000;
    unsigned int start_road_id = 0;
    unsigned int end_road_id = 0;
    for(auto & x : mRoads)
    {
        if (x.Distance(mStartPoint).second < min_dist)
        {
            min_dist = x.Distance(mStartPoint).second;
            start_road_id = x.iRoadId;
        }
    }
    min_dist = 100000;
    for(auto & x : mRoads)
    {
        if(x.Distance(mEndPoint).second < min_dist)
        {
            min_dist = x.Distance(mEndPoint).second;
            end_road_id = x.iRoadId;
        }
    }
    std::cout << "start id: " << start_road_id << " end id: " << end_road_id << std::endl;

    std::vector<std::vector<unsigned int>>routings;
    std::vector<unsigned int>routing;
    std::map<unsigned int, bool> is_visited;
    for(auto x : mRoads)
    {
        is_visited.insert(std::pair<unsigned int, bool>(x.iRoadId, false));
    }

    auto search = std::function<void(unsigned int, std::vector<unsigned int>)>();

    search = [&](unsigned int curr_road_id, std::vector<unsigned int> v)
    {
        if(mRoads[curr_road_id].iRoadId == end_road_id)
        {
            routings.emplace_back(v);
        }
        else
        {
            for(auto x : mRoads[curr_road_id].GetNextRoads())
            {
                if(!is_visited[x])
                {
                    is_visited[x] = true;
                    v.emplace_back(x);
                    search(x, v);
                    v.pop_back();
                    is_visited[x] = false;
                }
            }
        }
    };

    routing.emplace_back(start_road_id);
    is_visited[start_road_id] = true;
    search(start_road_id, routing);

    for(auto &x : routings)
    {
        for(auto & y : x)
            std::cout << y << " ";
        std::cout << std::endl;
    }

    //TODO Evaluate
}

void HDMap::Send()
{
    pSender->Send();
}