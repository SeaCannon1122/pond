#pragma once

#include "stamp.hpp"

#include <stdint.h>
#include <vector>
#include <array>
#include <memory>

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
    void* data = NULL;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t pixel_size = 0;
    double depth_scale = 0;
    Format format = Format::ERROR;
    Stamp stamp;

    //default for storing data;
    std::vector<uint8_t> default_data_buffer;
};

struct CameraInfo
{
    Stamp stamp;
    uint32_t height = 0;
    uint32_t width = 0;
    std::string distortion_model = "";
    std::vector<double> d = {};
    std::array<double, 9> k = {};
    std::array<double, 9> r = {};
    std::array<double, 12> p = {};
    uint32_t binning_x = 0;
    uint32_t binning_y = 0;
};

using ImgFrameSPtr = std::shared_ptr<ImgFrame>;