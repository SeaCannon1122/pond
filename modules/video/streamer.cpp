#include <pond/pond.hpp>
#include <pond_data_types/video_types.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
}

#include <vector>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>


class H264UDPStreamer : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
private:

    int width, height, fps;
    int sockfd, port;
    std::string ip;
    ImgFrame::Format image_format;
    bool is_yuv_native = false; // Flag to track if we can bypass conversion

    struct sockaddr_in dest_addr;

    const AVCodec* codec = nullptr;
    AVCodecContext* codec_ctx = nullptr;
    AVFrame* src_frame = nullptr;  // Renamed to handle BGR or YUV input
    AVFrame* yuv_frame = nullptr;
    AVPacket* pkt = nullptr;
    SwsContext* sws_ctx = nullptr;
    int64_t frame_count = 0;

    pond::Receiver receiver;
};

POND_MODULE_CPP_DECLARE(H264UDPStreamer, "streamer", "template module info")

pond_result H264UDPStreamer::onStartup(const std::vector<void*>& args)
{
    auto _width = parameter("width").asInt().getStrict();
    auto _height = parameter("height").asInt().getStrict();
    auto _port = parameter("port").asInt().getStrict();
    auto _ip = parameter("ip").asString().getStrict();
    auto _fps = parameter("fps").asInt().getStrict();

    auto _format_string = parameter("format").asString().getStrict({"RGB8", "BGR8", "Depth8", "Depth16", "Mono8", "Mono16"});
    if (!_format_string || !_width || !_height || !_port || !_ip || !_fps) return POND_ERROR;
    width = *_width; height = *_height; port = *_port; ip = *_ip; fps  = *_fps; image_format = ImgFrame::stringToFormat(*_format_string);

    // 1. Setup UDP Socket with optimized send buffer and non-blocking flags
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    int send_buffer_size = 1048576; // 1MB buffer to prevent drops
    setsockopt(sockfd, SOL_SOCKET, SO_SNDBUF, &send_buffer_size, sizeof(send_buffer_size));
    
    int flags = fcntl(sockfd, F_GETFL, 0);
    fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);

    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(port);
    inet_pton(AF_INET, ip.c_str(), &dest_addr.sin_addr);

    // 2. Initialize FFmpeg H.264 Encoder (tuned for ultra-fast zero latency)
    codec = avcodec_find_encoder(AV_CODEC_ID_H264);
    codec_ctx = avcodec_alloc_context3(codec);

    codec_ctx->width = width;
    codec_ctx->height = height;
    codec_ctx->time_base = (AVRational){1, fps};
    codec_ctx->framerate = (AVRational){fps, 1};
    codec_ctx->pix_fmt = AV_PIX_FMT_YUV420P;
    codec_ctx->bit_rate = 3000000; // 3 Mbps for crisp 720p/480p motion

    // CRITICAL LOW-LATENCY & NO-BUFFER FLAGS
    av_opt_set(codec_ctx->priv_data, "preset", "ultrafast", 0);
    av_opt_set(codec_ctx->priv_data, "tune", "zerolatency", 0);
    codec_ctx->gop_size = 1;          // Every frame is an I-frame (Keyframe)
    codec_ctx->max_b_frames = 0;      // Zero B-frames
    codec_ctx->thread_count = 1;      // Single thread to eliminate decoding queue delay

    avcodec_open2(codec_ctx, codec, nullptr);

    // 3. Map Input Format to FFmpeg Pixel Format
    AVPixelFormat src_pix_fmt = AV_PIX_FMT_BGR24;
    if (image_format == ImgFrame::Format::BGR8) {
        src_pix_fmt = AV_PIX_FMT_BGR24;
    } else if (image_format == ImgFrame::Format::RGB8) {
        src_pix_fmt = AV_PIX_FMT_RGB24;
    }

    // 4. Allocate Frames and Scaling Context
    src_frame = av_frame_alloc();
    src_frame->format = src_pix_fmt;
    src_frame->width = width;
    src_frame->height = height;
    // Note: We don't allocate internal memory via av_frame_get_buffer for src_frame 
    // because we will point it directly to Pond's memory buffer (Zero-Copy).

    yuv_frame = av_frame_alloc();
    yuv_frame->format = AV_PIX_FMT_YUV420P;
    yuv_frame->width = width;
    yuv_frame->height = height;
    av_frame_get_buffer(yuv_frame, 32);

    sws_ctx = sws_getContext(width, height, src_pix_fmt,
                             width, height, AV_PIX_FMT_YUV420P,
                             SWS_FAST_BILINEAR, nullptr, nullptr, nullptr);
    
    pkt = av_packet_alloc();

    receiver = createReceiver<ImgFrameSPtr>(
    {"in"},
    [this](ImgFrameSPtr* frame)
        {
            if ((*frame)->width != width || (*frame)->height != height || (*frame)->format != image_format)
            {
                POND_LOG(
                    "ERROR: frame->width (%d) != width (%d) || frame->height (%d) != height (%d) ||  frame->format (%s) != image_format (%s)",
                    (*frame)->width, width, (*frame)->height, height, ImgFrame::formatToString((*frame)->format).c_str(), ImgFrame::formatToString(image_format).c_str()
                );
                return;
            }

            // ZERO-COPY POINTER BINDING: Point FFmpeg directly to Pond's memory buffer 
            // instead of calling std::memcpy()
            src_frame->data[0] = reinterpret_cast<uint8_t*>((*frame)->data);
            src_frame->linesize[0] = width * 3; // 3 bytes per pixel for RGB/BGR

            // Convert BGR24/RGB24 to YUV420P via optimized sws_scale
            sws_scale(sws_ctx, src_frame->data, src_frame->linesize, 0, height, yuv_frame->data, yuv_frame->linesize);

            yuv_frame->pts = frame_count++;

            // Send frame to encoder
            if (avcodec_send_frame(codec_ctx, yuv_frame) < 0) return;

            // Receive encoded H.264 packets and push straight over UDP
            while (avcodec_receive_packet(codec_ctx, pkt) == 0)
            {
                sendto(sockfd, pkt->data, pkt->size, 0, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
                av_packet_unref(pkt);
            }
        }
    );

    return POND_SUCCESS;
}

void H264UDPStreamer::onShutdown()
{
    receiver.destroy();

    sws_freeContext(sws_ctx);
    // Do not free src_frame->data[0] since it references Pond's internal memory
    av_frame_free(&src_frame);
    av_frame_free(&yuv_frame);
    av_packet_free(&pkt);
    avcodec_free_context(&codec_ctx);
    close(sockfd);
}