//
// Created by iceytan on 18-11-24.
//

#ifndef HDMAP_RESOURCE_H
#define HDMAP_RESOURCE_H

#include <string>
#include "Type/Map.h"
#include "HDMap/srv_map_data.h"


namespace hdmap {

    class Resource {
        Map mMap;

        std::string ToXML(std::vector<RoadPtr>, std::vector<JuncPtr>);

    public:
        explicit Resource(const std::string &res_path);

        void SetSender(std::shared_ptr<Sender> sender_ptr);

        std::shared_ptr<Map> GetMap();

        bool OnRequest(HDMap::srv_map_data::Request &req, HDMap::srv_map_data::Response &res);
    };

}


#endif //HDMAP_RESOURCE_H
