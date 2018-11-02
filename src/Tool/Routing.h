//
// Created by vincent on 18-10-31.
//

#ifndef HDMAP_ROUTING_H
#define HDMAP_ROUTING_H

#include "../Type/Map.h"
#include "Location.h"

namespace hdmap
{
class Routing
{
private:
    Map mRouting;

    struct
    {
        bool is_init = false;
        int curr_idx = -1;
        int curr_rid = -1;
        int curr_dir =  0;
        int curr_jid = -1;
        int next_rid = -1;
        int next_dir =  0;
    }mRecord;

public:
    explicit Routing(const Map & map);

    void CallBack(const HDMap::Location & msg);

    void SendMap();
};
}


#endif //HDMAP_ROUTING_H
