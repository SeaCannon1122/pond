#define POND_MODULE_CPP_MAKE_IMPLEMENTATION
#include <pond/pond.hpp>
#include <pond/data_types/video_types.hpp>
#include <pond/data_types/imu_types.hpp>

#include <depthai/depthai.hpp>

class DepthaiImgFrame : public ImgFrame
{
public:

    explicit DepthaiImgFrame(std::shared_ptr<dai::ImgFrame>& frame_, ImgFrame::Format format_, const std::string& frame_id) : frame(frame_)
    {
        data = frame->getFrame().data;
        width = frame->getWidth();
        height = frame->getHeight();
        pixel_size = frame->getBytesPerPixel();
        format = format_;
        stamp.frame_id = frame_id;
        stamp.hw_time = std::chrono::duration<double>(frame->getTimestampDevice().time_since_epoch()).count();
        stamp.time = std::chrono::duration<double>(frame->getTimestamp().time_since_epoch()).count();
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

    std::shared_ptr<dai::node::Camera> color_node;
    std::shared_ptr<dai::MessageQueue> color_out_queue;

    std::shared_ptr<dai::node::Camera> stereo_left_node;
    std::shared_ptr<dai::node::Camera> stereo_right_node;
    std::shared_ptr<dai::node::Sync> stereo_sync_node;
    std::shared_ptr<dai::MessageQueue> stereo_out_queue;
    
    std::shared_ptr<dai::node::IMU> imu_node;
    std::shared_ptr<dai::MessageQueue> imu_queue;
    bool imu_running = true;
    std::thread imu_thread;
    uint32_t imu_rate;

    pond::Distributor<ImgFrameSPtr, CameraInfo, ImgFrameSPtr, CameraInfo, ImgFrameSPtr, CameraInfo> image_distributor;
    pond::Distributor<ImuData> imu_distributor;
    CameraInfo stereo_left_info, stereo_right_info, color_info;
    ImuData imu_data;
};

POND_MODULE_CPP_DECLARE(DepthaiCamera, "camera", "driver module for the Oak D Lite")

POND_BUNDLE_DECLARE(
    "Depthai modules", 
    1,
    POND_MODULE(DepthaiCamera),
)

pond_result DepthaiCamera::onStartup(const std::vector<void*>& args)
{
    std::vector<int32_t> color_dims = parameter("color.dims").asIntArray().get({1280, 720}, 2, 2);
    std::vector<int32_t> stereo_dims = parameter("stereo.dims").asIntArray().get({640, 480}, 2, 2);
    uint32_t fps = parameter("fps").asInt().get(30);

    color_info.stamp.frame_id = parameter("color.frame_id").asString().get("color_optical_frame");
    stereo_left_info.stamp.frame_id = parameter("stereo.left_frame_id").asString().get("stereo_left_optical_frame");
    stereo_right_info.stamp.frame_id = parameter("stereo.right_frame_id").asString().get("stereo_right_optical_frame");
    imu_data.stamp.frame_id = parameter("imu.frame_id").asString().get("imu");

    image_distributor = createDistributor<ImgFrameSPtr, CameraInfo, ImgFrameSPtr, CameraInfo, ImgFrameSPtr, CameraInfo>(
        {
            "color/image", "color/cam_info", 
            "stereo_left/image", "stereo_left/cam_info", 
            "stereo_right/image", "stereo_right/cam_info",
        }
    );

    imu_distributor = createDistributor<ImuData>({"imu"});

    if (auto mxid = parameter("MxId").asString().getStrict({}, false))
    {
        auto devices = dai::Device::getAllConnectedDevices();
        POND_LOG("%d device(s) connected", devices.size());

        for (int i = 0;; i++)
        {
            if (i == devices.size())
            {
                POND_LOG("No device with MxId '%s' connected", mxid->c_str());
                return POND_ERROR;
            }

            if(devices[i].getDeviceId() == *mxid) break;
        }
        device = std::make_shared<dai::Device>(dai::DeviceInfo(*mxid));
    }
    else device = std::make_shared<dai::Device>();

    device->setMaxReconnectionAttempts(0);

    pipeline = std::make_shared<dai::Pipeline>(device);

    color_node = pipeline->create<dai::node::Camera>();
    color_node->build(dai::CameraBoardSocket::CAM_A);
    color_out_queue = color_node->requestOutput({color_dims[0], color_dims[1]}, dai::ImgFrame::Type::RGB888i, dai::ImgResizeMode::CROP, fps)->createOutputQueue(1);

    stereo_left_node = pipeline->create<dai::node::Camera>();
    stereo_left_node->build(dai::CameraBoardSocket::CAM_B);

    stereo_right_node = pipeline->create<dai::node::Camera>();
    stereo_right_node->build(dai::CameraBoardSocket::CAM_C);

    stereo_sync_node = pipeline->create<dai::node::Sync>();
    stereo_sync_node->setRunOnHost(true);

    stereo_left_node->requestOutput({stereo_dims[0], stereo_dims[1]}, dai::ImgFrame::Type::GRAY8, dai::ImgResizeMode::CROP, fps)->link(stereo_sync_node->inputs["left"]);
    stereo_left_node->requestOutput({stereo_dims[0], stereo_dims[1]}, dai::ImgFrame::Type::GRAY8, dai::ImgResizeMode::CROP, fps)->link(stereo_sync_node->inputs["right"]);
    stereo_out_queue = stereo_sync_node->out.createOutputQueue(1);

    imu_node = pipeline->create<dai::node::IMU>();
    imu_node->enableIMUSensor({dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW}, imu_rate = parameter("imu.rate").asInt().get(10));
    imu_node->setBatchReportThreshold(1);
    imu_node->setMaxBatchReports(4);
    imu_queue = imu_node->out.createOutputQueue();

    imu_thread = std::thread([this](){
        while (imu_running)
        {
            bool has_timeout = false;
            auto imu_batch = imu_queue->get<dai::IMUData>(std::chrono::seconds(1), has_timeout);
            if (has_timeout)
            {
                POND_LOG("Imu data timed out");
                shutdown();
                return;
            }

            for(auto& packet : imu_batch->packets)
            {
                auto& accel = packet.acceleroMeter;
                auto& gyro  = packet.gyroscope;

                double accel_time = std::chrono::duration<double>(accel.getTimestamp().time_since_epoch()).count();
                double accel_hw_time = std::chrono::duration<double>(accel.getTimestampDevice().time_since_epoch()).count();
                double gyro_time = std::chrono::duration<double>(gyro.getTimestamp().time_since_epoch()).count();
                double gyro_hw_time = std::chrono::duration<double>(gyro.getTimestampDevice().time_since_epoch()).count();

                if (std::abs(accel_hw_time - gyro_hw_time) > 1.0 / (double)imu_rate) POND_LOG(
                    "| accel_time (%f) - gyro_time (%f) | = %f > %f = 1 / imu_rate (%d)", 
                    accel_hw_time, gyro_hw_time, 
                    std::abs(accel_hw_time - gyro_hw_time),
                    1.0 / (double)imu_rate,
                    imu_rate
                );

                imu_data.stamp.time = (accel_time + gyro_time) / 2.0;
                imu_data.stamp.time = (accel_hw_time + gyro_hw_time) / 2.0;

                imu_data.lin_acc[0] = accel.x;
                imu_data.lin_acc[1] = accel.y;
                imu_data.lin_acc[2] = -accel.z;

                imu_data.ang_vel[0] = gyro.x;
                imu_data.ang_vel[0] = gyro.y;
                imu_data.ang_vel[0] = -gyro.z;

                imu_distributor.distribute(imu_data);
            }
        }
    });

    pipeline->start();

    auto calib_data = device->readCalibration();
    auto color_intrinsics = calib_data.getCameraIntrinsics(
        dai::CameraBoardSocket::CAM_A, 
        color_dims[0],
        color_dims[1]
    );
    
    color_info.width = color_dims[0];
    color_info.height = color_dims[1];
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) color_info.k(i, j) = color_intrinsics[i][j];

    return POND_SUCCESS;
}

void DepthaiCamera::onShutdown()
{
    imu_running = false;
    imu_thread.join();

    pipeline->stop();
    pipeline->wait();

    color_node.reset();
    color_out_queue.reset();

    stereo_sync_node.reset();
    stereo_out_queue.reset();
    stereo_left_node.reset();
    stereo_right_node.reset();
    
    imu_node.reset();
    imu_queue.reset();

    pipeline.reset();
    device.reset();

    image_distributor.destroy();
    imu_distributor.destroy();
}

void DepthaiCamera::onFrame()
{
    if (!pipeline->isRunning())
    {
        shutdown();
        return;
    }

    try {
        bool has_timeout = false;
        auto stereo_group = stereo_out_queue->get<dai::MessageGroup>(std::chrono::seconds(1), has_timeout);
        if (has_timeout)
        {
            POND_LOG("Stereo images timed out");
            shutdown();
            return;
        }

        auto color_frame = color_out_queue->get<dai::ImgFrame>(std::chrono::seconds(1), has_timeout);
        if (has_timeout)
        {
            POND_LOG("Color image timed out");
            shutdown();
            return;
        }
    
        auto left_frame = stereo_group->get<dai::ImgFrame>("left");
        auto right_frame = stereo_group->get<dai::ImgFrame>("right");

        if (left_frame && right_frame && color_frame)
        {
            ImgFrameSPtr color_msg = std::make_shared<DepthaiImgFrame>(color_frame, ImgFrame::Format::RGB8, color_info.stamp.frame_id);
            ImgFrameSPtr left_msg = std::make_shared<DepthaiImgFrame>(left_frame, ImgFrame::Format::Mono8, stereo_left_info.stamp.frame_id);
            ImgFrameSPtr right_msg = std::make_shared<DepthaiImgFrame>(right_frame, ImgFrame::Format::Mono8, stereo_right_info.stamp.frame_id);

            color_info.stamp = color_msg->stamp;
            stereo_left_info.stamp = left_msg->stamp;
            stereo_right_info.stamp = right_msg->stamp;

            image_distributor.distribute(color_msg, color_info, left_msg, stereo_left_info, right_msg, stereo_right_info);
        }
        else
        {
            if (!left_frame) POND_LOG("left stereo frame is empty");
            if (!right_frame) POND_LOG("right stereo frame is empty");
            if (!color_frame) POND_LOG("color frame is empty");
        }
    }
    catch (const dai::MessageQueue::QueueException& e)
    {
        POND_LOG("DepthAI message queue closed: %s", e.what());
        shutdown();
    }
}
