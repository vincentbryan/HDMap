//
// Created by vincent on 18-10-23.
//

#ifndef HDMAP_IVIEW_H
#define HDMAP_IVIEW_H

#include "Tool/Sender.h"

namespace hdmap
{
class IView
{
public:
    virtual void Send(Sender & sender) = 0;
};
}

#endif //HDMAP_IVIEW_H
