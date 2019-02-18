//
// Created by iceytan on 18-11-24.
//

#ifndef HDMAP_RESOURCE_H
#define HDMAP_RESOURCE_H

#include <string>
#include "Type/Map.h"
#include "HDMap/srv_map_data.h"


namespace hdmap {

    class Resource
    {
        Map mMap;

        std::string ToXML(std::vector<RoadPtr>, std::vector<JuncPtr>);

    public:
        /**
         * 读取地图，生成 Resource 对象
         * @param res_path 地图数据xml文件的路径
         */
        explicit Resource(const std::string &res_path);

        /**
         * 设置 Sender，用于地图可视化信息的发送
         * @param sender_ptr Sender的指针
         */
        void SetSender(std::shared_ptr<Sender> sender_ptr);

        /**
         * 获取地图资源信息
         * @return
         */
        MapPtr GetMap();

        bool OnRequest(HDMap::srv_map_data::Request &req, HDMap::srv_map_data::Response &res);
    };

}


#endif //HDMAP_RESOURCE_H
