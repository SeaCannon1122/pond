#include <pond/pond.hpp>
#include <librealsense2/rs.hpp>
#include "../wrapped_image_frame.hpp"

class RealSenseWrappedImageFrame : public WrappedImageFrame
{
public:

    explicit RealSenseWrappedImageFrame(rs2::video_frame& frame_, const std::string& format_) : frame(frame_)
    {

        data = frame.get_data();
        width = frame.get_width();
        height = frame.get_height();
        pixel_size = frame.get_bytes_per_pixel();
        format = format_;
    }

    RealSenseWrappedImageFrame* clone() const override
    {
        return new RealSenseWrappedImageFrame(*this);
    }

private:
    rs2::video_frame frame;
};

class RealsenseDriver : public pond::ModuleBase
{
public:
    virtual pond_result onActivate() override;
    virtual void onDeactivate() override;
    virtual void onFrame() override;
private:
    rs2::pipeline pipe;
    rs2::config cfg;
    pond::Distributor color_distributor;
    pond::Distributor left_distributor;
    pond::Distributor right_distributor;
};