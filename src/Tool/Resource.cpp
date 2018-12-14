#include <utility>

//
// Created by iceytan on 18-11-24.
//

#include <Tool/Resource.h>
#include <unordered_set>

hdmap::Resource::Resource(const std::string& res_path) {
    mMap.Load(res_path);
}

bool hdmap::Resource::OnRequest(HDMap::srv_map_data::Request &req, HDMap::srv_map_data::Response &res) {
    std::string _info;
    std::vector<RoadPtr > roads;
    std::vector<JuncPtr > junctions;

    if(req.type=="RoadById")
    {
        for(auto& idx: req.argv){
            if(idx>=0 && idx<mMap.GetRoadSize()){
                _info += std::to_string(uint(idx))+" ";
                roads.push_back(mMap.GetRoadPtrById(uint(idx)));
            }
            else{
                ROS_ERROR("%id is not in road's range.",uint(idx));
            }
        }
    }
    else if(req.type=="JunctionById")
    {
        for(auto& idx: req.argv){
            if(idx>=0 && idx<mMap.GetJunctionSize()){
                _info += std::to_string(uint(idx))+" ";
                junctions.push_back(mMap.GetJuncPtrById(uint(idx)));
            }
            else{
                ROS_ERROR("%id is not in junction's range.",uint(idx));
            }
        }
    }
    else if(req.type == "RoadByPos" || req.type == "JunctionByPos")
    {
        
        static double last_x, last_y;
        static std::vector<RoadPtr > _catch_roads;
        static std::vector<JuncPtr > _catch_junctions;

        // decltype(_catch_junctions)().swap(_catch_junctions);
        // decltype(_catch_roads)().swap(_catch_roads);
        // !!! the mem assigned to _catch_* will not release util end the program.!!!
        
        double cur_x,cur_y,dis;
        if (req.argv.size() == 2){
            dis = 100;
        }
        else if (req.argv.size() == 3){
            dis = req.argv[2];
        }
        else {
            ROS_ERROR("Resource Server Error, %s must specify x and y", req.type.c_str());
            return false;
        }
        cur_x = req.argv[0];
        cur_y = req.argv[1];
        
        // do catch
        const double  dis_tolerence_pow = 4*4;
        if( pow(last_x-cur_x,2)+pow(last_y-cur_y,2) < dis_tolerence_pow
            && (!_catch_junctions.empty() || !_catch_roads.empty()))
        {
            roads = _catch_roads;
            junctions = _catch_junctions;
            if (req.type == "RoadByPos")
            {
                for(auto &r: roads)
                    _info += std::to_string(r->ID) + " ";
                _info = _info.empty()? "[none] ":"[catch] "+_info;
            }
            else
            {
                for(auto &j: junctions)
                    _info += std::to_string(j->ID) + " ";
                _info = _info.empty()? "[none] ":"[catch] "+_info;
            }
        }
        else
        {
            // find junctions, and record the nearest junction
            int _nearest_jid = -1;
            double _nearest_dis = std::numeric_limits<double >::max();
            for(auto& j: mMap.mJuncPtrs)
            {
                auto dis_to_j = j->GetDistanceFromCoor({cur_x, cur_y});
                if(dis_to_j<=dis)
                {

                    _info += std::to_string(j->ID) + " ";
                    junctions.emplace_back(j);
                }
                if(dis_to_j < _nearest_dis)
                {
                    _nearest_dis = dis_to_j;
                    _nearest_jid = j->ID;
                }
            }

            _catch_junctions = junctions;


            if( req.type == "RoadByPos" )
            {
                _info = "";
                // else if none of road was found, do a search from the nearest junction
                if(junctions.empty() && _nearest_jid!=-1)
                {
                    junctions.emplace_back(mMap.GetJuncPtrById(_nearest_jid));
                }

                std::set<uint> _road_id_set;
                // get all adjacent roads from junction
                for(auto& jptr: junctions){
                    for (auto &rl: jptr->RoadLinks)
                    {
                        uint r1 = rl.first.first;
                        uint r2 = rl.first.second;

                        _road_id_set.insert(r1);
                        _road_id_set.insert(r2);

                        _road_id_set.insert(mMap.GetRoadNeighbor(mMap.GetRoadPtrById(r1))->ID);
                        _road_id_set.insert(mMap.GetRoadNeighbor(mMap.GetRoadPtrById(r2))->ID);
                    }
                }

                std::vector<std::pair<double, RoadPtr>> _tmp; // save for sort by distance.

                for (auto &rp: _road_id_set)
                {
                    auto r_ptr = mMap.GetRoadPtrById(rp);

                    if (r_ptr != nullptr) {
                        double r1_dis = r_ptr->GetDistanceFromCoor({cur_x, cur_y});
                        if (r1_dis > dis) continue;
                        _info += std::to_string(r_ptr->ID) + " ";
                        _tmp.emplace_back(r1_dis, r_ptr);
                    }
                }
                std::sort(_tmp.begin(), _tmp.end());

                for (auto &p: _tmp) {
                    roads.emplace_back(p.second);
                }
                _catch_roads = roads;
            }
            last_x = cur_x;
            last_y = cur_y;
        }

        auto obj_name = req.type=="RoadByPos"? "Road":"Junction";
        ROS_INFO("%s requests %s { %s} successfully",req.type.c_str(), obj_name, _info.c_str());

    }
    else{
        ROS_ERROR("Resource Server Error, Not such a type: %s",req.type.c_str());
        return false;
    }

    res.res = ToXML(roads,junctions);
    return true;
}

void hdmap::Resource::SetSender(std::shared_ptr<Sender> sender_ptr) {
    if (this->mMap.mRoadPtrs.empty()){
        ROS_ERROR("Resource maybe not loaded correctly, mMap is empty()");
    }
    else{
        mMap.SetSender(std::move(sender_ptr));
    }
}

std::shared_ptr<hdmap::Map> hdmap::Resource::GetMap() {
    return std::shared_ptr<hdmap::Map>(&mMap);
}

std::string hdmap::Resource::ToXML(std::vector<RoadPtr> roads, std::vector<JuncPtr > junctions) {
    try{
        pt::ptree tree;
        for(auto& r: roads){
            tree.add_child("hdmap.roads.road",r->ToXML());
        }
        for(auto& j: junctions){
            tree.add_child("hdmap.junctions.junction", j->ToXML());
        }
        std::stringstream ss;
        pt::write_xml(ss, tree);
        return ss.str();
    }
    catch (std::exception &e)
    {
        std::cout << "Error: " << e.what() << std::endl;
    }
    return "";
}


