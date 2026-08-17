#include <pond/pond.hpp>
#include <opencv2/opencv.hpp>
#include <pond/data_types.hpp>

class CVImgFrame : public ImgFrame
{
public:

    explicit CVImgFrame(cv::Mat& frame_, const std::string& format_) : frame(frame_)
    {

        data = frame.data;
        width = frame.cols;
        height = frame.rows;
        pixel_size = frame.elemSize();
        format = format_;
    }

private:
    cv::Mat frame;
};

class V4L2Camera : public pond::ModuleBase
{
public:
    virtual pond_result onStartup() override;
    virtual void onShutdown() override;
    virtual void onFrame() override;
private:
    cv::VideoCapture cap;
    pond::Distributor<ImgFrameSPtr> distributor;
};

POND_MODULE_CPP_DECLARE(V4L2Camera, "v4l2_camera", "v4l2 camera image distributor")

pond_result V4L2Camera::onStartup()
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

    ImgFrameSPtr color_msg = std::make_shared<CVImgFrame>(frame, "rgb8");
    distributor.distribute(color_msg);
    
}
