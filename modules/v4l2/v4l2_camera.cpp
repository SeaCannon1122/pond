#include <pond/pond.hpp>
#include <pond_data_types/cv_img_frame.hpp>

#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <linux/videodev2.h>

#define MAX_BUFFER_COUNT 16

struct buffer
{
    void   *start;
    size_t  length;
};

class V4L2Camera : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
    virtual void onFrame() override;
private:
    pond::Distributor distributor;

    uint32_t width, height, fps;

    int fd;
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer buffers[MAX_BUFFER_COUNT];
    uint32_t n_buffers;
};

#define CLEAR(x) memset(&(x), 0, sizeof(x))

static int xioctl(int fh, int request, void *arg) {
    int r;
    do {
        r = ioctl(fh, request, arg);
    } while (-1 == r && EINTR == errno);
    return r;
}


POND_MODULE_CPP_DECLARE(V4L2Camera, "v4l2_camera", "v4l2 camera image distributor")

POND_BUNDLE_DECLARE("v4l2 bundle", POND_MODULE(V4L2Camera))

pond_result V4L2Camera::onStartup(const std::vector<void*>& args)
{
    auto device = parameter("device").asString().getStrict(); if (!device) return POND_ERROR;
    width = parameter("width").asInt().get(640);
    height = parameter("height").asInt().get(480);
    fps = parameter("fps").asInt().get(30);

    POND_LOG("Opening device: %s", device->c_str());
    
    if ((fd = open(device->c_str(), O_RDWR | O_NONBLOCK, 0)) == -1)
        POND_LOG_RETURN_ERROR(stderr, "Cannot open '%s': %s", device->c_str(), strerror(errno));
      
    struct v4l2_capability cap;
    if (xioctl(fd, VIDIOC_QUERYCAP, &cap) == -1) { close(fd); POND_LOG_RETURN_ERROR("Is not V4L2 device");}
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) { close(fd); POND_LOG_RETURN_ERROR(stderr, "Device is no video capture device");}
    if (!(cap.capabilities & V4L2_CAP_STREAMING)) { close(fd); POND_LOG_RETURN_ERROR(stderr, "Device does not support streaming i/o");}

    // 2. Set format (Resolution and Pixel Format)
    struct v4l2_format fmt;
    CLEAR(fmt);
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width       = width;
    fmt.fmt.pix.height      = height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV; // Change to V4L2_PIX_FMT_MJPEG if needed
    fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;

    if (xioctl(fd, VIDIOC_S_FMT, &fmt) == -1) { close(fd); POND_LOG_RETURN_ERROR("VIDIOC_S_FMT: %s", strerror(errno)); }

    // Note VIDIOC_S_FMT may alter width/height based on hardware limits
    if (width != fmt.fmt.pix.width || height != fmt.fmt.pix.height) { close(fd); POND_LOG_RETURN_ERROR("Got dimensions %dx%d instead of %dx%d", fmt.fmt.pix.width, fmt.fmt.pix.height, width, height);}

    // 3. Set Frame Rate (FPS)
    struct v4l2_streamparm streamparm; CLEAR(streamparm);

    streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    streamparm.parm.capture.timeperframe.numerator = 1;
    streamparm.parm.capture.timeperframe.denominator = fps;

    if (xioctl(fd, VIDIOC_S_PARM, &streamparm) == -1) POND_LOG("VIDIOC_S_PARM (FPS setting might not be supported by driver): %s", strerror(errno));
    else POND_LOG("Requested FPS: %d, Set FPS: %d/%d", fps, streamparm.parm.capture.timeperframe.denominator, streamparm.parm.capture.timeperframe.numerator);

    // 4. Request Buffers
    struct v4l2_requestbuffers req; CLEAR(req);

    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (xioctl(fd, VIDIOC_REQBUFS, &req) == -1) { close(fd); POND_LOG_RETURN_ERROR("VIDIOC_REQBUFS: %s", strerror(errno)); }

    if (req.count < 2) { close(fd); POND_LOG_RETURN_ERROR("Insufficient buffer memory on %s", device->c_str()); }
    if (req.count > MAX_BUFFER_COUNT) { close(fd); POND_LOG_RETURN_ERROR("Too many buffers (%d)", req.count);}
    
    // 5. Map Buffers
    for (n_buffers = 0; n_buffers < req.count; n_buffers++)
    {
        struct v4l2_buffer buf; CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = n_buffers;

        if (xioctl(fd, VIDIOC_QUERYBUF, &buf) == -1)
        {
            for (uint32_t i = 0; i < n_buffers; i++) munmap(buffers[i].start, buffers[i].length);
            close(fd);
            POND_LOG_RETURN_ERROR("VIDIOC_QUERYBUF: %s", strerror(errno));
        }

        buffers[n_buffers].length = buf.length;
        buffers[n_buffers].start = mmap(
            NULL, buf.length,
            PROT_READ | PROT_WRITE, MAP_SHARED,
            fd, buf.m.offset
        );

        if (buffers[n_buffers].start == MAP_FAILED)
        {
            for (uint32_t i = 0; i < n_buffers; i++) munmap(buffers[i].start, buffers[i].length);
            close(fd);
            POND_LOG_RETURN_ERROR("mmap: %s", strerror(errno));
        }
    }

    // 6. Queue Buffers
    for (uint32_t i = 0; i < n_buffers; i++)
    {
        struct v4l2_buffer buf;
        CLEAR(buf);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (xioctl(fd, VIDIOC_QBUF, &buf) == -1)
        {
            for (uint32_t i = 0; i < n_buffers; i++) munmap(buffers[i].start, buffers[i].length);
            close(fd);
            POND_LOG_RETURN_ERROR("VIDIOC_QBUF: %s", strerror(errno));
        }
    }

    // 7. Start Streaming
    if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
    {
        for (uint32_t i = 0; i < n_buffers; i++) munmap(buffers[i].start, buffers[i].length);
        close(fd);
        POND_LOG_RETURN_ERROR("VIDIOC_STREAMON: %s", strerror(errno));
    }

    distributor = createDistributor<ImgFrameSPtr>({"out"});

    return POND_SUCCESS;
}

void V4L2Camera::onShutdown()
{
    distributor.destroy();

    if (xioctl(fd, VIDIOC_STREAMOFF, &type) == -1) POND_LOG("VIDIOC_STREAMOFF: %s", strerror(errno));
    
    for (uint32_t i = 0; i < n_buffers; i++) munmap(buffers[i].start, buffers[i].length);

    close(fd);
}

void V4L2Camera::onFrame()
{
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd, &fds);

    struct timeval tv = {.tv_sec = 2, .tv_usec = 0};

    int r = select(fd + 1, &fds, NULL, NULL, &tv);
    if (r == -1)
    {
        if (EINTR == errno) return;

        POND_LOG("select: %s", strerror(errno));
        shutdown(); return;
    }
    if (r == 0) { POND_LOG("Select timeout"); shutdown(); return;}

    // Dequeue buffer
    struct v4l2_buffer buf; CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (xioctl(fd, VIDIOC_DQBUF, &buf) == -1)
    {
        if (EAGAIN == errno) return;

        POND_LOG("VIDIOC_DQBUF: %s", strerror(errno));
        shutdown(); return;
    }

    cv::Mat yuyv_mat(height, width, CV_8UC2, buffers[buf.index].start);
    cv::Mat rgb_mat;
    cv::cvtColor(yuyv_mat, rgb_mat, cv::COLOR_YUV2RGB_YUYV);

    // Re-queue buffer
    if (xioctl(fd, VIDIOC_QBUF, &buf) == -1)
    {
        POND_LOG("VIDIOC_QBUF: %s", strerror(errno));
        shutdown(); return;
    }

    
    
    ImgFrameSPtr color_msg = std::make_shared<CVImgFrame>(rgb_mat, ImgFrame::Format::RGB8);
    color_msg->stamp.time = pond::get_time();
    color_msg->stamp.hw_time = color_msg->stamp.time;
    distributor.distribute(&color_msg);
    
}
