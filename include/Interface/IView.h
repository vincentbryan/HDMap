#ifndef HDMAP_IVIEW_H
#define HDMAP_IVIEW_H

#include "Tool/Sender.h"

namespace hdmap
{
class IView
{
public:
    virtual void OnSend(Sender &sender) = 0;
};
}

#endif //HDMAP_IVIEW_H
