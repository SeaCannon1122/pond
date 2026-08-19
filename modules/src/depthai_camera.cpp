#include <depthai/capabilities/ImgFrameCapability.hpp>
#include <depthai/pipeline/datatype/ImgFrame.hpp>
#include <pond/pond.hpp>
#include <depthai/depthai.hpp>
#include <memory>
#include <vector>
#include "depthai/depthai.hpp"
#include "pond/data_types.hpp"

class DepthaiImgFrame : public ImgFrame
{
public:

    explicit DepthaiImgFrame(std::shared_ptr<dai::ImgFrame>& frame_, ImgFrame::Format format_) : frame(frame_)
    {
        data = frame->getFrame().data;
        width = frame->getWidth();
        height = frame->getHeight();
        pixel_size = frame->getBytesPerPixel();
        format = format_;
    }

private:
    std::shared_ptr<dai::ImgFrame> frame;
};

class DepthaiCamera : public pond::ModuleBase
{
public:
    virtual pond_result onStartup() override;
    virtual void onShutdown() override;
    virtual void onFrame() override;
private:
    std::shared_ptr<dai::Pipeline> pipeline;
    std::shared_ptr<dai::node::Camera> left;
    std::shared_ptr<dai::node::Camera> right;
    std::shared_ptr<dai::node::Sync> sync;
    std::shared_ptr<dai::MessageQueue> out_queue;
    pond::Distributor<ImgFrameSPtr, CameraInfo, ImgFrameSPtr, CameraInfo, std::vector<ImuDataStamped>> distributor;
    CameraInfo mono_left_info, mono_right_info;
};

POND_MODULE_CPP_DECLARE(DepthaiCamera, "dephai_camera", "driver module for the Oak D Lite")

pond_result DepthaiCamera::onStartup()
{
    distributor = createDistributor<ImgFrameSPtr, CameraInfo, ImgFrameSPtr, CameraInfo, std::vector<ImuDataStamped>>(
        {
            "mono_left/image", "mono_left/cam_info", 
            "mono_right/image", "mono_right/cam_info", 
            "imu_data"
        }
    );

    pipeline = std::make_shared<dai::Pipeline>();

    left = pipeline->create<dai::node::Camera>();
    left->build(dai::CameraBoardSocket::CAM_B);

    right = pipeline->create<dai::node::Camera>();
    right->build(dai::CameraBoardSocket::CAM_C);

    // Create and configure sync node
    sync = pipeline->create<dai::node::Sync>();
    sync->setRunOnHost(true);  // Can also run on device

    // Link cameras to sync inputs
    left->requestOutput({640, 480}, dai::ImgFrame::Type::GRAY8, dai::ImgResizeMode::CROP, 30.f)->link(sync->inputs["left"]);
    right->requestOutput({640, 480}, dai::ImgFrame::Type::GRAY8, dai::ImgResizeMode::CROP, 30.f)->link(sync->inputs["right"]);

    // Create output queue
    out_queue = sync->out.createOutputQueue();

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

    distributor.destroy();
}

void DepthaiCamera::onFrame()
{
    if (!pipeline->isRunning())
    {
        shutdown();
        return;
    }

    auto sync_group = out_queue->tryGet<dai::MessageGroup>();
    if (sync_group)
    {
        auto left_frame = sync_group->get<dai::ImgFrame>("left");
        auto right_frame = sync_group->get<dai::ImgFrame>("right");

        if (left_frame && right_frame)
        {
            ImgFrameSPtr left_msg = std::make_shared<DepthaiImgFrame>(left_frame, ImgFrame::Format::Mono8);
            ImgFrameSPtr right_msg = std::make_shared<DepthaiImgFrame>(left_frame, ImgFrame::Format::Mono8);
            std::vector<ImuDataStamped> imu_data;
            distributor.distribute(left_msg, mono_left_info, right_msg, mono_right_info, imu_data);

            return;

            // auto imu_data = imu_queue->tryGet<dai::IMUData>();
            // if (imu_data)
            // {
            //     for (const auto& packet : imu_data->packets)
            //     {
            //         auto accel = packet.acceleroMeter;
            //         auto gyro = packet.gyroscope;

            //     }
            // }
        }
    }
}
