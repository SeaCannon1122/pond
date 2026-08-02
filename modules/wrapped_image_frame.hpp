#pragma once

#include <pond/pond.hpp>

class WrappedImageFrame : public pond::ManagedMessage
{
public:
    virtual ~WrappedImageFrame() = default;
    const void* data;
    uint32_t width;
    uint32_t height;
    uint32_t pixel_size;
    std::string format;
};