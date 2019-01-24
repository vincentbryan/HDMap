#include <utility>
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
    RoadPtrs.emplace_back(p);
    return p;
}

JuncPtr Map::AddJunction()
{
    JuncPtr p(new Junction(MapPtr(this)));
    JuncPtrs.emplace_back(p);
    return p;
}

void Map::Send()
{
    for(auto & m : RoadPtrs) m->OnSend(*pSender);
    for(auto & j : JuncPtrs) j->OnSend(*pSender);

    pSender->Send();

}


void Map::AddConnection(JuncPtr p, unsigned int from_road, int from_lane_idx,
                        unsigned int to_road, int to_lane_idx,
                        double _ctrl_len1, double _ctrl_len2)
{
    Pose start_pose, end_pose;

    if(from_lane_idx > 0)
    {
        auto p1 = RoadPtrs[from_road]->mSecPtrs.back()->GetLanePoseByIndex(from_lane_idx).back();
        auto p2 = RoadPtrs[from_road]->mSecPtrs.back()->GetLanePoseByIndex(from_lane_idx-1).back();
        start_pose = {0.5 * (p1.GetPosition() + p2.GetPosition()), p1.GetAngle()};
    }
    else
    {
        auto p1 = RoadPtrs[from_road]->mSecPtrs.front()->GetLanePoseByIndex(from_lane_idx).front();
        auto p2 = RoadPtrs[from_road]->mSecPtrs.front()->GetLanePoseByIndex(from_lane_idx+1).front();
        start_pose = {0.5 * (p1.GetPosition() + p2.GetPosition()), p1.GetAngle()};
        start_pose.Rotate(180);
    }


    if(to_lane_idx >  0)
    {
        auto p1 = RoadPtrs[to_road]->mSecPtrs.front()->GetLanePoseByIndex(to_lane_idx).front();
        auto p2 = RoadPtrs[to_road]->mSecPtrs.front()->GetLanePoseByIndex(to_lane_idx-1).front();
        end_pose = {0.5 * (p1.GetPosition() + p2.GetPosition()), p1.GetAngle()};
    }
    else
    {
        auto p1 = RoadPtrs[to_road]->mSecPtrs.back()->GetLanePoseByIndex(to_lane_idx).back();
        auto p2 = RoadPtrs[to_road]->mSecPtrs.back()->GetLanePoseByIndex(to_lane_idx+1).back();
        end_pose = {0.5 * (p1.GetPosition() + p2.GetPosition()), p1.GetAngle()};
        end_pose.Rotate(180);
    }


    p->AddConnection(
        from_road, from_lane_idx, start_pose,
        to_road, to_lane_idx, end_pose,
        _ctrl_len1, _ctrl_len2
    );

    if(from_lane_idx > 0)
        RoadPtrs[from_road]->mNextJid = p->ID;
    else
        RoadPtrs[from_road]->mPrevJid = p->ID;

    if(to_lane_idx > 0)
        RoadPtrs[to_road]->mPrevJid = p->ID;
    else
        RoadPtrs[to_road]->mNextJid = p->ID;
}

void Map::CommitRoadInfo()
{
    for(auto & p : RoadPtrs)
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

    for (auto const &m : JuncPtrs[jid]->RoadLinks)
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
    for(auto & road : RoadPtrs)
    {
        p_map.add_child("hdmap.roads.road", road->ToXML());
        mRoadIdToPtr[road->ID] = road;
    }


    for(auto & junc : JuncPtrs){
        p_map.add_child("hdmap.junctions.junction", junc->ToXML());
        mJuncIdToPtr[junc->ID] = junc;
    }
    return p_map;
}

void Map::FromXML(const pt::ptree &p)
{
    Clear();

    try
    {
        for(auto & r : p.get_child("hdmap.roads"))
        {
            RoadPtr pRoad(new Road());
            pRoad->FromXML(r.second);
            AddRoadFromPtr(pRoad);
            Road::ROAD_ID = std::max((unsigned) 0, pRoad->ID);
        }
        Road::ROAD_ID++;

        for(auto & j : p.get_child("hdmap.junctions"))
        {
            JuncPtr pJunc(new Junction(MapPtr(this)));
            pJunc->FromXML(j.second);
            AddJunctionFromPtr(pJunc);
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
#ifdef DEBUG_INFO
    printf("No Found Road %d, Maybe initialization dose not finish.\n", road_id);
#endif
    return nullptr;
}

JuncPtr Map::GetJuncPtrById(unsigned int junc_id)
{
    if(mJuncIdToPtr.count(junc_id))
    {
        return mJuncIdToPtr[junc_id];
    }
#ifdef DEBUG_INFO
    printf("No Found Junction %d, Maybe initialization dose not finish.\n", junc_id);
#endif
    return nullptr;
}

void Map::Clear()
{
    RoadPtrs.clear();
    JuncPtrs.clear();
    mJuncIdToPtr.clear();
    mRoadIdToPtr.clear();
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

        p1 = RoadPtrs[_from_road_id]->mSecPtrs.back()->GetLanePoseByIndex(from_lane_idx).back();
        p2 = RoadPtrs[_from_road_id]->mSecPtrs.back()->GetLanePoseByIndex(from_lane_idx-1).back();
        Pose start_pose = {0.5 * (p1.GetPosition() + p2.GetPosition()), p1.GetAngle()};

        p1 = RoadPtrs[_to_road_id]->mSecPtrs.front()->GetLanePoseByIndex(to_lane_idx).front();
        p2 = RoadPtrs[_to_road_id]->mSecPtrs.front()->GetLanePoseByIndex(to_lane_idx-1).front();
        Pose end_pose = {0.5 * (p1.GetPosition() + p2.GetPosition()), p1.GetAngle()};

        road_link.AddLaneLink(from_lane_idx, to_lane_idx, Bezier(start_pose, end_pose, ctrl_len1, ctrl_len2));
    }
    std::pair<unsigned int, unsigned int> m(_from_road_id, _to_road_id);
    p->RoadLinks[m] = road_link;

    RoadPtrs[_from_road_id]->mNextJid = p->ID;
    RoadPtrs[_to_road_id]->mPrevJid = p->ID;
}

unsigned long Map::GetRoadSize() {
    return RoadPtrs.size();
}

unsigned long Map::GetJunctionSize() {
    return JuncPtrs.size();
}

RoadPtr Map::GetRoadNeighbor(RoadPtr ptr)
{
    const double _MIN_DIS_NEIGHBOR = 8.0;
    Coor _tmp_coor = ptr->GetReferenceLinePoses()[0].GetPosition();
    for (auto &road: RoadPtrs) {
        double dis = road->GetDistanceFromCoor(_tmp_coor);
        if (dis < _MIN_DIS_NEIGHBOR && ptr->ID != road->ID) {
            return road;
        }
    }
    return nullptr;
}

void Map::AddRoadFromPtr(RoadPtr road)
{
    if(GetRoadPtrById(road->ID) == nullptr)
    {
        RoadPtrs.emplace_back(road);
        mRoadIdToPtr[road->ID] = road;
    }
}

void Map::AddJunctionFromPtr(JuncPtr junction)
{
    if(GetJuncPtrById(junction->ID)== nullptr)
    {
        JuncPtrs.emplace_back(junction);
        mJuncIdToPtr[junction->ID] = junction;
    }
}

std::vector<RoadPtr> Map::GetRoadPtrByDistance(const Coor &coor, double distance, bool keep_one) {
    
    std::vector<RoadPtr> roads;
    std::vector<std::pair<double, RoadPtr>> _road_dis_vec;
    
    
    std::vector<JuncPtr> junctions = GetJuncPtrByDistance(coor, distance, true);
    std::set<uint> _road_id_set;
    
    for(auto& jptr: junctions){
        for (auto &rl: jptr->RoadLinks)
        {
            uint r1 = rl.first.first;
            uint r2 = rl.first.second;

            _road_id_set.insert(r1);
            _road_id_set.insert(r2);

            auto r1_neighbor_ptr = GetRoadNeighbor(GetRoadPtrById(r1));
            auto r2_neighbor_ptr = GetRoadNeighbor(GetRoadPtrById(r2));
            if(r1_neighbor_ptr)
            {
                _road_id_set.insert(r1_neighbor_ptr->ID);
            }
            if(r2_neighbor_ptr)
            {
                _road_id_set.insert(r2_neighbor_ptr->ID);
            }


        }
    }

    unsigned int _nearest_rid = 0;
    double _nearest_dis = std::numeric_limits<double>::max();
    
    for (auto &rid: _road_id_set)
    {
        auto r_ptr = GetRoadPtrById(rid);

        if (r_ptr != nullptr) {
            double _dis = r_ptr->GetDistanceFromCoor(coor);
            if (_dis > distance) continue;
            _road_dis_vec.emplace_back(_dis, r_ptr);
            if (_nearest_dis > _dis)
            {
                _nearest_dis = _dis;
                _nearest_rid = rid;
            }
        }
    }

    if (_road_dis_vec.empty())
    {
        return {};
    }
    
    std::sort(_road_dis_vec.begin(), _road_dis_vec.end());

    for (auto &p: _road_dis_vec) roads.emplace_back(p.second);

    if (roads.empty() && keep_one)
        roads.emplace_back(GetRoadPtrById(_nearest_rid));

    return roads;
}

std::vector<JuncPtr> Map::GetJuncPtrByDistance(const Coor &coor, double distance, bool keep_one) {


    std::vector<JuncPtr> junctions;
    std::vector<std::pair<double,JuncPtr >> _junc_dis_vec;
    
    unsigned int _nearest_jid = 0;
    double _nearest_dis = std::numeric_limits<double>::max();

    for (const auto& _jptr: JuncPtrs)
    {
        const auto _dis = _jptr->GetDistanceFromCoor(coor);
        if ( _dis <= distance )
        {
            _junc_dis_vec.emplace_back(_dis, _jptr);
        }
        if (_nearest_dis > _dis)
        {
            _nearest_dis = _dis;
            _nearest_jid = _jptr->ID;
        }
    }
    
    std::sort(_junc_dis_vec.begin(),_junc_dis_vec.end());

    for(auto &p :_junc_dis_vec) junctions.emplace_back(p.second);
    
    if(_junc_dis_vec.empty() && keep_one)
        junctions.emplace_back(GetJuncPtrById(_nearest_jid));
    
    return junctions;
}

std::tuple<RoadPtr, SecPtr, int> Map::GetLaneInfoByPose(const Pose &pose) {

    int _target_lane_id = -1;
    auto _near_roads = GetRoadPtrByDistance(pose.GetPosition(), 1);

    if (_near_roads.empty())
    {
        ROS_INFO("GetLaneInfoByPose: Not in any road, skip for lane matching.");
        return {nullptr, nullptr, _target_lane_id};
    }

    const double _ANGLE_TORELENT = 30;

    auto _cur_road_ptr = _near_roads.front();

    for (auto& rptr: _near_roads)
    {
        for(auto & _lane_sec_ptr: rptr->mSecPtrs)
        {
            // select road by direction and covering.
            const auto& _lane_region_rpose = _lane_sec_ptr->GetReferenceLinePose();
            Angle _diff_angle = pose.GetAngle() - _lane_region_rpose[_lane_region_rpose.size()/2].GetAngle();
            Angle::Wrap(_diff_angle, -180, 180);

            if(_lane_sec_ptr->IsCover(pose.GetPosition()) && abs(_diff_angle.Value()) <  _ANGLE_TORELENT)
            {
                std::vector<Pose> _left_side_pose = _lane_sec_ptr->GetReferenceLinePose();
                std::vector<Pose> _side_pose;

                for (int _lane_id = 1; _lane_id <= _lane_sec_ptr->mRightBoundary; ++ _lane_id)
                {
                    _side_pose.clear();
                    auto _right_side_pose = _lane_sec_ptr->GetLanePoseByIndex(_lane_id);

                    _side_pose.insert(_side_pose.end(),_right_side_pose.begin(),_right_side_pose.end());
                    _side_pose.insert(_side_pose.end(),_left_side_pose.rbegin(),_left_side_pose.rend());
                    if(IGeometry::Cover(_side_pose,{pose.GetPosition()}))
                    {
                        _target_lane_id = _lane_id;
                        break;
                    }
                    _left_side_pose = _right_side_pose;
                }
                return {rptr, _lane_sec_ptr, _target_lane_id};
            }
        }
    }

    return {nullptr, nullptr, _target_lane_id};
}

std::tuple<JuncPtr, RoadLink, int> Map::GetRoadLinkByPose(const Pose &pose)
{
    auto _res = GetJuncPtrByDistance({pose.x,pose.y},0);

    if (_res.empty())
    {
        std::cout << "Cannot not find a junction"<< std::endl;
        return {nullptr, RoadLink(), -1};
    }

    // select a roadlink by position, direction.
    JuncPtr _jptr = _res.front();

    std::vector<std::tuple<double, RoadLink*, int>> _select_result;

    for (auto& _roadlink: _jptr->RoadLinks)
    {
        double _min_distance = std::numeric_limits<double>::max();
        double _diff_angle = -1;
        int _link_id = -1;
        for (int _i = 0; _i < _roadlink.second.mLaneLinks.size(); ++_i)
        {
            auto& _link = _roadlink.second.mLaneLinks[_i];
            for(auto& _link_pose: _link.mReferLine.GetPoses(1.0))
            {
                double _cur_distance = Coor::Distance(_link_pose.GetPosition(), pose.GetPosition());

                if(_cur_distance < _min_distance)
                {
                    _min_distance = _cur_distance;
                    Angle _tmp_angle = _link_pose.GetAngle() - pose.GetAngle();
                    Angle::Wrap(_tmp_angle, -180.0, 180.0);
                    _diff_angle = abs(_tmp_angle.Value());
                    _link_id = _i;
                }
            }
            assert(_link_id!= -1);
            _select_result.emplace_back(_diff_angle, &_roadlink.second, _link_id);
        }
    }

    std::sort(_select_result.begin(),_select_result.end());

    return {_jptr, *std::get<1>(_select_result.front()), std::get<2>(_select_result.front())};
}
