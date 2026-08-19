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
        ERROR = 0,
        RGB8 = 1,
        BGR8 = 2,
        Depth8 = 3,
        Depth16 = 4,
        Mono8 = 5,
        Mono16 = 6,
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

    static std::string formatToString(Format format)
    {
        switch (format)
        {
        case Format::RGB8: return "RGB8";
        case Format::BGR8: return "BGR8";
        case Format::Depth8: return "Depth8";
        case Format::Depth16: return "Depth16";
        case Format::Mono8: return "Mono8";
        case Format::Mono16: return "Mono16";
        default: return "ERROR";
        }
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