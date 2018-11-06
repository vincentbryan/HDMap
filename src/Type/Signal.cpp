//
// Created by vincent on 18-10-26.
//

#include "Signal.h"
using namespace hdmap;

void Signal::Send(hdmap::Sender &sender)
{
    std::string text = mInfo;
    auto m = sender.GetText(text, {x, y}, 0, 1.0, 0, 1.0, 0.5, 1.0);
    sender.array.markers.emplace_back(m);
    sender.Send();
}

Signal::Signal(double _x, double _y, double _z, std::string _type, std::string _info)
{
    x = _x;
    y = _y;
    z = _z;
    mType = _type;
    mInfo = _info;
}

boost::property_tree::ptree Signal::ToXML()
{
    pt::ptree p_sig;
    p_sig.add("x", x);
    p_sig.add("y", y);
    p_sig.add("z", z);
    p_sig.add("type", mType);
    p_sig.add("info", mInfo);

    return p_sig;
}

void Signal::FromXML(const pt::ptree &p)
{
    x = p.get<double>("x");
    y = p.get<double>("y");
    z = p.get<double>("z");
    mType = p.get<std::string>("type");
    mInfo = p.get<std::string>("info");
}
