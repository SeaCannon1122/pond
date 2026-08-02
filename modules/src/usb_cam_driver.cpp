#include <pond/pond.hpp>
#include <opencv2/opencv.hpp>

class UsbCamDriver : public pond::ModuleBase
{
public:
    virtual pond_result onActivate() override;
    virtual void onDeactivate() override;
    virtual void onFrame() override;
private:
    cv::VideoCapture cap;
    pond::Distributor distributor;
};

POND_MODULE_CPP_DECLARE(UsbCamDriver, "usb_cam_driver", "simple driver bridge for a usb camera")

pond_result UsbCamDriver::onActivate()
{
    POND_LOG("activating ...");
    distributor = createDistributor("color");

    cap = cv::VideoCapture(0);

    if (!cap.isOpened())
    {
        printf("Failed to open webcam.\n"); fflush(stdout);
        return POND_ERROR;
    }
    POND_LOG("activated");
    return POND_SUCCESS;
}

void UsbCamDriver::onDeactivate()
{
    POND_LOG("deactivating ...");
    cap.release();
    distributor.destroy();
    POND_LOG("deactivated");
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
