#include <pond/pond.hpp>
#include "cv_img_frame.hpp"

class V4L2Camera : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
    virtual void onFrame() override;
private:
    cv::VideoCapture cap;
    pond::Distributor<ImgFrameSPtr> distributor;
};

POND_MODULE_CPP_DECLARE(V4L2Camera, "v4l2_camera", "v4l2 camera image distributor")

pond_result V4L2Camera::onStartup(const std::vector<void*>& args)
{
    distributor = createDistributor<ImgFrameSPtr>({"out"});

    cap = cv::VideoCapture(parameter("device").asString().get("/dev/video0"), cv::CAP_V4L2);

    if (!cap.isOpened())
    {
        POND_LOG("Failed to open webcam.\n");
        return POND_ERROR;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, parameter("width").asInt().get(640));
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, parameter("height").asInt().get(480));
    cap.set(cv::CAP_PROP_FPS, parameter("fps").asInt().get(30));

    return POND_SUCCESS;
}

void V4L2Camera::onShutdown()
{
    cap.release();
    distributor.destroy();
}

void V4L2Camera::onFrame()
{
    cv::Mat frame;
    cap >> frame;

    if (frame.empty())
    {
        POND_LOG("Failed to capture frame.");
        shutdown();
    }

    ImgFrameSPtr color_msg = std::make_shared<CVImgFrame>(frame, ImgFrame::Format::BGR8);
    distributor.distribute(color_msg);
    
}
