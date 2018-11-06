//
// Created by vincent on 18-10-26.
//

#pragma once
#include <string>
#include "../Interface/IView.h"
#include "../Interface/IXML.h"

namespace hdmap
{
class Signal : public IView, public IXML
{
public:
    double x = 0, y = 0, z = 0;
    std::string mType;
    std::string mInfo;

public:
    explicit Signal(){};
    Signal(double _x, double _y, double _z, std::string _type, std::string _info);
    void Send(Sender &sender) override;
    boost::property_tree::ptree ToXML() override;
    void FromXML(const pt::ptree &p) override;
};
}



