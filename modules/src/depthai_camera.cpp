#include <pond/pond.hpp>
#include <pond/data_types/video_types.hpp>
#include <pond/data_types/imu_types.hpp>

#include <depthai/depthai.hpp>

class DepthaiImgFrame : public ImgFrame
{
public:

    explicit DepthaiImgFrame(std::shared_ptr<dai::ImgFrame>& frame_, ImgFrame::Format format_, double time) : frame(frame_)
    {
        data = frame->getFrame().data;
        width = frame->getWidth();
        height = frame->getHeight();
        pixel_size = frame->getBytesPerPixel();
        format = format_;
        stamp.hw_time = std::chrono::duration<double>(frame->getTimestamp().time_since_epoch()).count();
        stamp.time = time;
    }

private:
    std::shared_ptr<dai::ImgFrame> frame;
};

class DepthaiCamera : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
    virtual void onFrame() override;
private:
    std::shared_ptr<dai::Device> device;

    std::shared_ptr<dai::Pipeline> pipeline;
    std::shared_ptr<dai::node::Camera> left;
    std::shared_ptr<dai::node::Camera> right;
    std::shared_ptr<dai::node::Sync> sync;
    std::shared_ptr<dai::MessageQueue> out_queue;

    pond::Distributor<ImgFrameSPtr, CameraInfo, ImgFrameSPtr, CameraInfo, std::vector<ImuDataStamped>> distributor;
    CameraInfo mono_left_info, mono_right_info;
};

POND_MODULE_CPP_DECLARE(DepthaiCamera, "depthai_camera", "driver module for the Oak D Lite")

pond_result DepthaiCamera::onStartup(const std::vector<void*>& args)
{
    distributor = createDistributor<ImgFrameSPtr, CameraInfo, ImgFrameSPtr, CameraInfo, std::vector<ImuDataStamped>>(
        {
            "mono_left/image", "mono_left/cam_info", 
            "mono_right/image", "mono_right/cam_info", 
            "imu_data"
        }
    );

    if (auto mxid = parameter("mxid").asString().getStrict({}, false))
    {
        auto devices = dai::Device::getAllConnectedDevices();
        POND_LOG("%d device(s) connected", devices.size());

        for (int i = 0;; i++)
        {
            if (i == devices.size())
            {
                POND_LOG("No device with mxID %s connected", mxid->c_str());
                return POND_ERROR;
            }

            if(devices[i].getDeviceId() == *mxid) break;
        }
        device = std::make_shared<dai::Device>(dai::DeviceInfo(*mxid));
    }
    else device = std::make_shared<dai::Device>();

    device->setMaxReconnectionAttempts(0);

    pipeline = std::make_shared<dai::Pipeline>(device);

    left = pipeline->create<dai::node::Camera>();
    left->build(dai::CameraBoardSocket::CAM_B);

    right = pipeline->create<dai::node::Camera>();
    right->build(dai::CameraBoardSocket::CAM_C);

    sync = pipeline->create<dai::node::Sync>();
    sync->setRunOnHost(true);

    left->requestOutput({640, 480}, dai::ImgFrame::Type::GRAY8, dai::ImgResizeMode::CROP, 30.f)->link(sync->inputs["left"]);
    right->requestOutput({640, 480}, dai::ImgFrame::Type::GRAY8, dai::ImgResizeMode::CROP, 30.f)->link(sync->inputs["right"]);

    out_queue = sync->out.createOutputQueue();

    pipeline->start();

    return POND_SUCCESS;
}

void DepthaiCamera::onShutdown()
{
    pipeline->stop();
    pipeline->wait();
    sync.reset();
    left.reset();
    right.reset();
    out_queue.reset();
    pipeline.reset();
    device.reset();

    distributor.destroy();
}

void DepthaiCamera::onFrame()
{
    if (!pipeline->isRunning())
    {
        shutdown();
        return;
    }

    try {
        auto sync_group = out_queue->tryGet<dai::MessageGroup>();
        double time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    
        if (sync_group)
        {
            auto left_frame = sync_group->get<dai::ImgFrame>("left");
            auto right_frame = sync_group->get<dai::ImgFrame>("right");

            if (left_frame && right_frame)
            {
                ImgFrameSPtr left_msg = std::make_shared<DepthaiImgFrame>(left_frame, ImgFrame::Format::Mono8, time);
                ImgFrameSPtr right_msg = std::make_shared<DepthaiImgFrame>(right_frame, ImgFrame::Format::Mono8, time);
                std::vector<ImuDataStamped> imu_data;
                distributor.distribute(left_msg, mono_left_info, right_msg, mono_right_info, imu_data);
            }
        }
    }
    catch (const dai::MessageQueue::QueueException& e)
    {
        POND_LOG("DepthAI message queue closed: %s", e.what());
        shutdown();
    }
}
