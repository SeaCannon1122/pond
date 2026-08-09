#include <pond/pond.hpp>
#include <opencv2/opencv.hpp>

class UsbCamDriver : public pond::ModuleBase
{
public:
    virtual pond_result onStartup() override;
    virtual void onShutdown() override;
    virtual void onFrame() override;
private:
    cv::VideoCapture cap;
    pond::Distributor distributor;
};

POND_MODULE_CPP_DECLARE(UsbCamDriver, "usb_cam_driver", "simple driver bridge for a usb camera")

pond_result UsbCamDriver::onStartup()
{
    POND_LOG("starting ...");
    distributor = createDistributor("out");

    cap = cv::VideoCapture(stoi(args[0]));

    if (!cap.isOpened())
    {
        printf("Failed to open webcam.\n"); fflush(stdout);
        return POND_ERROR;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, stoi(args[1]));
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, stoi(args[2]));

    POND_LOG("started");
    return POND_SUCCESS;
}

void UsbCamDriver::onShutdown()
{
    POND_LOG("shutting down ...");
    cap.release();
    distributor.destroy();
    POND_LOG("shut down");
}

void UsbCamDriver::onFrame()
{
    cv::Mat frame;
    cap >> frame;

    if (frame.empty())
    {
        printf("Failed to capture frame.\n"); fflush(stdout);
        shutdown();
    }
}
