//
// Created by vincent on 18-10-26.
//

#pragma once
#include <string>
#include "../Interface/IView.h"

namespace hdmap
{
class Signal : public IView
{
public:
    Vector2d position;
    int direction = 0;
    std::string type;
    std::string info;

    Signal(){};
    Signal(Vector2d v, int dir, std::string _type, std::string _info);
    void Send(Sender &sender) override;
};
}



