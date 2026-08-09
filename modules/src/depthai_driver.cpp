#include <depthai/pipeline/datatype/ImgFrame.hpp>
#include <pond/pond.hpp>
#include <depthai/depthai.hpp>
#include <memory>
#include "depthai/depthai.hpp"
#include "wrapped_image_frame.hpp"

class DepthaiWrappedImageFrame : public WrappedImageFrame
{
public:

    explicit DepthaiWrappedImageFrame(std::shared_ptr<dai::ImgFrame>& frame_, const std::string& format_) : frame(frame_)
    {
        data = frame->getFrame().data;
        width = frame->getWidth();
        height = frame->getHeight();
        pixel_size = frame->getBytesPerPixel();
        format = format_;
    }

    DepthaiWrappedImageFrame* clone() const override
    {
        return new DepthaiWrappedImageFrame(*this);
    }

private:
    std::shared_ptr<dai::ImgFrame> frame;
};

class DepthaiDriver : public pond::ModuleBase
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
    pond::Distributor left_distributor;
    pond::Distributor right_distributor;
};

POND_MODULE_CPP_DECLARE(DepthaiDriver, "dephai_driver", "driver module for the Oak D Lite")

pond_result DepthaiDriver::onStartup()
{
    left_distributor = createDistributor("left");
    right_distributor = createDistributor("right");

    pipeline = std::make_shared<dai::Pipeline>();

    left = pipeline->create<dai::node::Camera>();
    left->build(dai::CameraBoardSocket::CAM_B);

    right = pipeline->create<dai::node::Camera>();
    right->build(dai::CameraBoardSocket::CAM_C);

    // Create and configure sync node
    sync = pipeline->create<dai::node::Sync>();
    sync->setRunOnHost(true);  // Can also run on device

    // Link cameras to sync inputs
    left->requestFullResolutionOutput()->link(sync->inputs["left"]);
    right->requestFullResolutionOutput()->link(sync->inputs["right"]);

    // Create output queue
    out_queue = sync->out.createOutputQueue();

    return POND_SUCCESS;
}

void DepthaiDriver::onShutdown()
{
    pipeline->stop();
    pipeline->wait();
    sync.reset();
    left.reset();
    right.reset();
    out_queue.reset();
    pipeline.reset();

    left_distributor.destroy();
    right_distributor.destroy();
}

void DepthaiDriver::onFrame()
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
            auto* left_msg = new std::shared_ptr<WrappedImageFrame>(std::make_shared<DepthaiWrappedImageFrame>(left_frame, "mono8"));
            left_distributor.distribute(static_cast<void*>(left_msg));

            auto* right_msg = new std::shared_ptr<WrappedImageFrame>(std::make_shared<DepthaiWrappedImageFrame>(right_frame, "mono8"));
            right_distributor.distribute(static_cast<void*>(right_msg));

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
