//
// Created by vincent on 18-10-26.
//

#include "Signal.h"
using namespace hdmap;

void Signal::Send(hdmap::Sender &sender)
{
    std::string text = type + "[" + info + "]";
    auto m = sender.GetText(text, position, 0, 1.0, 0, 1.0, 0.5, 1.0);
    sender.array.markers.emplace_back(m);
    sender.Send();
}

Signal::Signal(Vector2d v, int dir, std::string _type, std::string _info)
{
    position = v;
    direction = dir;
    type = _type;
    info = _info;
}
