#include <stdint.h>
#include <string>
#include <memory>

class ImgFrame
{
public:
    virtual ~ImgFrame() = default;
    const void* data;
    uint32_t width;
    uint32_t height;
    uint32_t pixel_size;
    std::string format;
};

using ImgFrameSPtr = std::shared_ptr<ImgFrame>;

class Point
{
    double x;
    double y;
    double z;
    double w;
};

class ImuData
{
    Point lin_acc;
    Point ang_vel;
};