#include <stdint.h>
#include <string>
#include <vector>
#include <memory>
#include <array>

class ImgFrame
{
public:

    enum class Format
    {
        ERROR,
        RGB8,
        BGR8,
        Z8,
        Z16,
        Y8,
        Y16,
    };

    static Format stringToFormat(const std::string& str)
    {
        if (str == "RGB8") return Format::RGB8;
        if (str == "BGR8") return Format::BGR8;
        if (str == "Z8") return Format::Z8;
        if (str == "Z16") return Format::Z16;
        if (str == "Y8") return Format::Y8;
        if (str == "Y16") return Format::Y16;
        return Format::ERROR;
    };

    virtual ~ImgFrame() = default;
    const void* data;
    uint32_t width;
    uint32_t height;
    uint32_t pixel_size;
    Format format;
    double time;
    double hw_time;
};

class CameraInfo
{
public:
    std::string frame_id;
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