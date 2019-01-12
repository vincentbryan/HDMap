//
// Created by vincent on 18-10-29.
//

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
    virtual pt::ptree ToXML() = 0;
    virtual void FromXML(const pt::ptree & p) = 0;
};
}
#endif //HDMAP_IXML_H
