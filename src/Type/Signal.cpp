//
// Created by vincent on 18-10-26.
//

#include "Signal.h"
using namespace hdmap;

void Signal::Send(hdmap::Sender &sender)
{
    std::string text = mType + "[" + mInfo + "]";
    auto m = sender.GetText(text, mPosition, 0, 1.0, 0, 1.0, 0.5, 1.0);
    sender.array.markers.emplace_back(m);
    sender.Send();
}

Signal::Signal(Vector2d _v, int _dir, std::string _type, std::string _info)
{
    mPosition = _v;
    mDirection = _dir;
    mType = _type;
    mInfo = _info;
}

boost::property_tree::ptree Signal::ToXML()
{
    pt::ptree p_sig;
    p_sig.add("x", mPosition.x);
    p_sig.add("y", mPosition.y);
    p_sig.add("direction", mDirection);
    p_sig.add("type", mType);
    p_sig.add("info", mInfo);

    return p_sig;
}

void Signal::FromXML(const pt::ptree &p)
{
    mPosition.x = p.get<double>("x");
    mPosition.y = p.get<double>("y");
    mDirection = p.get<int>("direction");
    mType = p.get<std::string>("type");
    mInfo = p.get<std::string>("info");
}
