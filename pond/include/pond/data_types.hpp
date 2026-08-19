#include <stdint.h>
#include <string>
#include <vector>
#include <memory>
#include <array>

class Stamp
{
    double time;
    double hw_time;
    std::string frame_id;
};


class ImgFrame
{
public:

    enum class Format
    {
        ERROR,
        RGB8,
        BGR8,
        Depth8,
        Depth16,
        Mono8,
        Mono16,
    };

    static Format stringToFormat(const std::string& str)
    {
        if (str == "RGB8") return Format::RGB8;
        if (str == "BGR8") return Format::BGR8;
        if (str == "Depth8") return Format::Depth8;
        if (str == "Depth16") return Format::Depth16;
        if (str == "Mono8") return Format::Mono8;
        if (str == "Mono16") return Format::Mono16;
        return Format::ERROR;
    };

    virtual ~ImgFrame() = default;
    const void* data;
    uint32_t width;
    uint32_t height;
    uint32_t pixel_size;
    Format format;
    Stamp stamp;
};

class CameraInfo
{
public:
    Stamp stamp;
    uint32_t height;
    uint32_t width;
    std::string distortion_model;
    std::vector<double> d;
    std::array<double, 9> k;
    std::array<double, 9> r;
    std::array<double, 12> p;
    uint32_t binning_x;
    uint32_t binning_y;
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

class ImuDataStamped
{
    ImuData data;
    Stamp stamp;
};