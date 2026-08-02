#include <mutex>
#include <stdint.h>

template <typename T>
class RingBufferQueue
{
public:
    explicit RingBufferQueue(uint32_t length) :
        capacity(length),
        buffer(new T[length]),
        head(0),
        tail(0),
        count(0)
    {
    }

    ~RingBufferQueue() { delete[] buffer; }

    void enqueue(T item)
    {
        std::lock_guard<std::mutex> lock(mutex);

        buffer[tail] = item;
        tail = (tail + 1) % capacity;

        if (count == capacity) head = (head + 1) % capacity;
        else count++;
    }

    bool dequeue(T* item)
    {
        std::lock_guard<std::mutex> lock(mutex);

        if (count == 0) return false;

        *item = buffer[head];
        head = (head + 1) % capacity;
        count--;

        return true;
    }

    size_t length()
    {
        std::lock_guard<std::mutex> lock(mutex);
        return count;
    }

private:
    uint32_t capacity;
    T* buffer;

    uint32_t head;
    uint32_t tail;
    uint32_t count;

    std::mutex mutex;
};