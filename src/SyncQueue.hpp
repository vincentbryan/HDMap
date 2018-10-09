//
// Created by vincent on 18-10-9.
//

#ifndef HDMAP_SYNCQUEUE_H
#define HDMAP_SYNCQUEUE_H

#include <queue>
#include <mutex>
#include <iostream>
namespace hdmap
{
template <typename T>
class SyncQueue
{
private:
    std::queue<T> buffer;
    std::mutex m;
    int size;

public:

    explicit SyncQueue(int _size = 1000) : size(_size){};

    void Push(const T& t)
    {
        std::lock_guard<std::mutex> lock(m);
        buffer.push(t);
        if(buffer.size() > 1000)
            buffer.pop();
    }

    unsigned long Size()
    {
        std::lock_guard<std::mutex> lock(m);
        return buffer.size();
    }
};
}


#endif //HDMAP_SYNCQUEUE_H
