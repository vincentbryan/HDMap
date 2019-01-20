#include <utility>
#include <Tool/Resource.h>
#include <unordered_set>

hdmap::Resource::Resource(const std::string& res_path)
{
    mMap.Load(res_path);
}

bool hdmap::Resource::OnRequest(HDMap::srv_map_data::Request &req, HDMap::srv_map_data::Response &res)
{
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
        
        double dis;
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
        
        Coor _cur_coor;
        _cur_coor.x = req.argv[0];
        _cur_coor.y = req.argv[1];

        if (req.type == "JunctionByPos")
        {
            junctions = mMap.GetJuncPtrByDistance(_cur_coor, dis);
            for (auto& jptr: junctions)  _info += std::to_string(jptr->ID) + " ";
        }
        else if (req.type == "RoadByPos" )
        {
            roads = mMap.GetRoadPtrByDistance(_cur_coor,dis);
            for (auto& rptr: roads)  _info += std::to_string(rptr->ID) + " ";
        }

        auto obj_name = req.type=="RoadByPos"? "Road":"Junction";
        ROS_INFO("(%4.3f,%4.3f) %s requests %s { %s} successfully",
                _cur_coor.x,_cur_coor.y , req.type.c_str(), obj_name, _info.c_str());

    }
    else
    {
        ROS_ERROR("Resource Server Error, Not such a type: %s",req.type.c_str());
        return false;
    }

    res.res = ToXML(roads,junctions);
    return true;
}

void hdmap::Resource::SetSender(std::shared_ptr<Sender> sender_ptr)
{
    if (this->mMap.RoadPtrs.empty()){
        ROS_ERROR("Resource maybe not loaded correctly, mMap is empty()");
    }
    else{
        mMap.SetSender(std::move(sender_ptr));
    }
}

hdmap::MapPtr hdmap::Resource::GetMap()
{
    return  &mMap;
}

std::string hdmap::Resource::ToXML(std::vector<RoadPtr> roads, std::vector<JuncPtr > junctions)
{
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


