#include <cstring>
#include <pond/pond.hpp>
#include <pond/data_types/cv_img_frame.hpp>

class DepthColorizer : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
    virtual void onFrame() override;
private:
    pond::Receiver<ImgFrameSPtr> receiver;
    pond::Distributor<ImgFrameSPtr> distributor;
    double max_depth;
};

POND_MODULE_CPP_DECLARE(DepthColorizer, "depth_colorizer", "coverts a depthimage to a colored rgb image")

pond_result DepthColorizer::onStartup(const std::vector<void*>& args)
{
    max_depth = parameter("max_depth").asDouble().get(4000);

    distributor = createDistributor<ImgFrameSPtr>({"out"});
    receiver = createReceiver<ImgFrameSPtr>({"in"}, [this](ImgFrameSPtr& frame){

        if (frame->format != ImgFrame::Format::Depth8 && frame->format != ImgFrame::Format::Depth16)
        {
            POND_LOG(
                "ERROR: frame->format (%s) != ImgFrame::Format::Depth8 && frame->format != ImgFrame::Format::Depth16",
                ImgFrame::formatToString(frame->format).c_str()
            );
            return;
        }

        cv::Mat depth_mat(frame->height, frame->width, frame->format == ImgFrame::Format::Depth8 ? CV_8UC1 : CV_16UC1, frame->data);
        
        cv::Mat colorized_mat;
        depth_mat.convertTo(colorized_mat, CV_8UC1, 255.0 / max_depth);
        cv::applyColorMap(colorized_mat, colorized_mat, cv::COLORMAP_JET);

        ImgFrameSPtr color_msg = std::make_shared<CVImgFrame>(colorized_mat, ImgFrame::Format::BGR8);

        distributor.distribute(color_msg);
    });
    return POND_SUCCESS;
}

void DepthColorizer::onShutdown()
{
    receiver.destroy();
    distributor.destroy();
}

void DepthColorizer::onFrame()
{
}
