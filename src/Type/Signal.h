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
    Vector2d mPosition;
    int mDirection = 0;
    std::string mType;
    std::string mInfo;

public:
    explicit Signal(){};
    Signal(Vector2d _v, int _dir, std::string _type, std::string _info);
    void Send(Sender &sender) override;
    boost::property_tree::ptree ToXML() override;
    void FromXML(const pt::ptree &p) override;
};
}



