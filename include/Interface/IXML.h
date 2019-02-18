// IXML 定义了保存和加载存储地图元素的接口

#ifndef HDMAP_IXML_H
#define HDMAP_IXML_H

#include "Common/pointer_typedef.h"
#include <boost/property_tree/ptree.hpp>
namespace pt = boost::property_tree;
namespace hdmap
{
class IXML
{
public:

    /**
     * 将IXML对象信息存储为ptree接口
     * @return 包含IXML对象的信息的ptree
     */
    virtual pt::ptree ToXML() = 0;

    /**
     * 将ptree对象转换为IXML对象接口
     * @param p 包含IXML对象的信息的ptree
     */
    virtual void FromXML(const pt::ptree & p) = 0;
};
}
#endif //HDMAP_IXML_H
