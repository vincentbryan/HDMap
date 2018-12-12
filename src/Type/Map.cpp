#include <utility>

//
// Created by vincent on 18-10-9.
//

#include <boost/property_tree/ptree.hpp>
#include <Type/Map.h>

using namespace hdmap;
namespace pt = boost::property_tree;

Map::Map()
{}

void Map::SetSender(std::shared_ptr<Sender> sender)
{
    pSender = std::move(sender);
}


RoadPtr Map::AddRoad(const Pose &_start_pose)
{
    RoadPtr p(new Road(_start_pose));
    mRoadPtrs.emplace_back(p);
    return p;
}

JuncPtr Map::AddJunction()
{
    JuncPtr p(new Junction(MapPtr(this)));
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
        mRoadPtrs[from_road]->mNextJid = p->ID;
    else
        mRoadPtrs[from_road]->mPrevJid = p->ID;

    if(to_lane_idx > 0)
        mRoadPtrs[to_road]->mPrevJid = p->ID;
    else
        mRoadPtrs[to_road]->mNextJid = p->ID;
}

void Map::CommitRoadInfo()
{
    for(auto & p : mRoadPtrs)
    {
        if(!p->mSecPtrs.empty())
            p->Lenght = p->mSecPtrs.back()->mStartS + p->mSecPtrs.back()->mReferLine.Length();
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

std::vector<RoadPtr> Map::AdjacentRoadInfo(RoadPtr p_road)
{
    std::vector<RoadPtr> res;

    int jid = p_road->mNextJid;
    if(jid == -1) return res;

    for (auto const &m : mJuncPtrs[jid]->RoadLinks)
    {
        if (m.first.first == p_road->ID)
        {
            res.emplace_back(GetRoadPtrById(m.first.second));
        }
    }
    return res;
}


boost::property_tree::ptree Map::ToXML()
{
    pt::ptree p_map;
    for(auto & road : mRoadPtrs)
    {
        p_map.add_child("hdmap.roads.road", road->ToXML());
        mRoadIdToPtr[road->ID] = road;
    }


    for(auto & junc : mJuncPtrs){
        p_map.add_child("hdmap.junctions.junction", junc->ToXML());
        mJuncIdToPtr[junc->ID] = junc;
    }
    return p_map;
}

void Map::FromXML(const pt::ptree &p)
{

    mRoadPtrs.clear();
    mJuncPtrs.clear();

    for(auto & r : p.get_child("hdmap.roads"))
    {
        RoadPtr pRoad(new Road());
        pRoad->FromXML(r.second);
        mRoadPtrs.emplace_back(pRoad);
        mRoadIdToPtr[pRoad->ID] = pRoad;
        Road::ROAD_ID = std::max((unsigned) 0, pRoad->ID);
    }
    Road::ROAD_ID++;

    try
    {
        for(auto & j : p.get_child("hdmap.junctions"))
        {
            JuncPtr pJunc(new Junction(MapPtr(this)));
            pJunc->FromXML(j.second);
            mJuncPtrs.emplace_back(pJunc);
            mJuncIdToPtr[pJunc->ID] = pJunc;
            Junction::JUNCTION_ID = std::max((unsigned) 0, pJunc->ID);
        }
        Junction::JUNCTION_ID++;
    }
    catch (std::exception & e)
    {
        std::cout << "Error: " << e.what() << std::endl;
    }

}

RoadPtr Map::GetRoadPtrById(unsigned int road_id)
{
    if(mRoadIdToPtr.count(road_id))
    {
        return mRoadIdToPtr[road_id];
    }
    printf("No Found Road %d, Maybe initialization dose not finish.\n", road_id);
    return nullptr;
}

JuncPtr Map::GetJuncPtrById(unsigned int junc_id)
{
    if(mJuncIdToPtr.count(junc_id))
    {
        return mJuncIdToPtr[junc_id];
    }
    printf("No Found Junction %d, Maybe initialization dose not finish.\n", junc_id);
    return nullptr;
}

void Map::Clear()
{
    mRoadPtrs.clear();
    mJuncPtrs.clear();
    Road::ROAD_ID = 0;
    Junction::JUNCTION_ID = 0;
}

void Map::AddRoadLink(JuncPtr p,
                      unsigned _from_road_id,
                      unsigned _to_road_id,
                      std::string _direction,
                      std::vector<std::tuple<int, int, double, double>>_lane_links)
{
    RoadLink road_link(_from_road_id, _to_road_id, _direction);

    for(auto & k : _lane_links)
    {
        auto from_lane_idx = std::get<0>(k);
        auto to_lane_idx = std::get<1>(k);
        auto ctrl_len1 = std::get<2>(k);
        auto ctrl_len2 = std::get<3>(k);
        Pose p1, p2;

        p1 = mRoadPtrs[_from_road_id]->mSecPtrs.back()->GetLanePoseByIndex(from_lane_idx).back();
        p2 = mRoadPtrs[_from_road_id]->mSecPtrs.back()->GetLanePoseByIndex(from_lane_idx-1).back();
        Pose start_pose = {0.5 * (p1.GetPosition() + p2.GetPosition()), p1.GetAngle()};

        p1 = mRoadPtrs[_to_road_id]->mSecPtrs.front()->GetLanePoseByIndex(to_lane_idx).front();
        p2 = mRoadPtrs[_to_road_id]->mSecPtrs.front()->GetLanePoseByIndex(to_lane_idx-1).front();
        Pose end_pose = {0.5 * (p1.GetPosition() + p2.GetPosition()), p1.GetAngle()};

        road_link.AddLaneLink(from_lane_idx, to_lane_idx, Bezier(start_pose, end_pose, ctrl_len1, ctrl_len2));
    }
    std::pair<unsigned int, unsigned int> m(_from_road_id, _to_road_id);
    p->RoadLinks[m] = road_link;

    mRoadPtrs[_from_road_id]->mNextJid = p->ID;
    mRoadPtrs[_to_road_id]->mPrevJid = p->ID;
}

unsigned long Map::GetRoadSize() {
    return mRoadPtrs.size();
}

unsigned long Map::GetJunctionSize() {
    return mJuncPtrs.size();
}

RoadPtr Map::GetRoadNeighbor(RoadPtr ptr) {
    const double _MIN_DIS_NEIGHBOR = 8.0;
    Coor _tmp_coor = ptr->GetReferenceLinePoses()[0].GetPosition();
    for (auto &road: mRoadPtrs) {
        double dis = road->GetDistanceFromCoor(_tmp_coor);
        if (dis < _MIN_DIS_NEIGHBOR && ptr->ID != road->ID) {
            return road;
        }
    }
    return nullptr;
}
