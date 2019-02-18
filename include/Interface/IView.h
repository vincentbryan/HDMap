#ifndef HDMAP_IVIEW_H
#define HDMAP_IVIEW_H

#include "Tool/Sender.h"

namespace hdmap
{
class IView
{
public:
    /**
     * 可视化对象接口
     * @param sender 将marker添加到sender.array.markers中，在程序中使用Sender的send()方法进行
     *               发送，发送结束后清空sender.array.markers
     */
    virtual void OnSend(Sender &sender) = 0;
};
}

#endif //HDMAP_IVIEW_H
