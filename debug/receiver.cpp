extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>

class H264UDPReceiver {
private:
    int width, height;
    int sockfd;

    const AVCodec* codec = nullptr;
    AVCodecContext* codec_ctx = nullptr;
    AVCodecParserContext* parser = nullptr;
    AVFrame* frame = nullptr;
    AVFrame* rgb_frame = nullptr;
    AVPacket* pkt = nullptr;
    SwsContext* sws_ctx = nullptr;

public:
    H264UDPReceiver(int w, int h, int port) : width(w), height(h) {
        // 1. Setup Non-blocking UDP Socket
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        sockaddr_in server_addr{};
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port);
        server_addr.sin_addr.s_addr = INADDR_ANY;

        bind(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr));
        
        // Set non-blocking so we never stall waiting for packets
        int flags = fcntl(sockfd, F_GETFL, 0);
        fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);

        // 2. Initialize FFmpeg H.264 Decoder & Parser
        codec = avcodec_find_decoder(AV_CODEC_ID_H264);
        codec_ctx = avcodec_alloc_context3(codec);
        
        // CRITICAL LOW-LATENCY DECODER FLAGS
        codec_ctx->flags |= AV_CODEC_FLAG_LOW_DELAY;
        codec_ctx->flags2 |= AV_CODEC_FLAG2_FAST;
        codec_ctx->thread_count = 1; // Single thread to eliminate frame-queue lag

        // Tell FFmpeg to try and conceal errors rather than dropping the frame entirely
        // FF_EC_GUESS_MV: Guess motion vectors for missing blocks
        // FF_EC_DEBLOCK: Apply deblocking filter to hide severe tearing edges
        codec_ctx->error_concealment = FF_EC_GUESS_MVS | FF_EC_DEBLOCK;

        // Lower error detection strictness so it doesn't give up on corrupted packets
        codec_ctx->err_recognition = AV_EF_CAREFUL;

        avcodec_open2(codec_ctx, codec, nullptr);

        parser = av_parser_init(AV_CODEC_ID_H264);
        pkt = av_packet_alloc();
        frame = av_frame_alloc();

        // 3. Setup Conversion Context (YUV420P -> BGR24 for OpenCV)
        rgb_frame = av_frame_alloc();
        rgb_frame->format = AV_PIX_FMT_BGR24;
        rgb_frame->width = width;
        rgb_frame->height = height;
        av_frame_get_buffer(rgb_frame, 32);

        sws_ctx = sws_getContext(width, height, AV_PIX_FMT_YUV420P,
                                 width, height, AV_PIX_FMT_BGR24,
                                 SWS_FAST_BILINEAR, nullptr, nullptr, nullptr);
    }

    ~H264UDPReceiver() {
        sws_freeContext(sws_ctx);
        av_frame_free(&frame);
        av_frame_free(&rgb_frame);
        av_packet_free(&pkt);
        av_parser_close(parser);
        avcodec_free_context(&codec_ctx);
        close(sockfd);
    }

    void run() {
        std::vector<unsigned char> buffer(65535);
        std::cout << "Listening for low-latency H.264 stream..." << std::endl;

        while (true) {
            ssize_t bytes_received = recv(sockfd, buffer.data(), buffer.size(), 0);
            
            if (bytes_received < 0) {
                if (errno == EWOULDBLOCK || errno == EAGAIN) {
                    usleep(1000); // Sleep 1ms to prevent 100% CPU pinning while idle
                    continue;
                }
                break;
            }

            // Parse raw incoming NAL units
            uint8_t* data = buffer.data();
            int size = bytes_received;

            while (size > 0) {
                uint8_t* out_data = nullptr;
                int out_size = 0;

                int parsed_bytes = av_parser_parse2(parser, codec_ctx, &out_data, &out_size,
                                                    data, size, AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0);
                data += parsed_bytes;
                size -= parsed_bytes;

                if (out_size > 0) {
                    pkt->data = out_data;
                    pkt->size = out_size;

                    // Send packet to decoder
                    if (avcodec_send_packet(codec_ctx, pkt) == 0) {
                        while (avcodec_receive_frame(codec_ctx, frame) == 0) {
                            // Convert YUV420P to OpenCV BGR24
                            sws_scale(sws_ctx, frame->data, frame->linesize, 0, height,
                                      rgb_frame->data, rgb_frame->linesize);

                            // Wrap OpenCV Mat around ffmpeg buffer (zero-copy view)
                            cv::Mat img(height, width, CV_8UC3, rgb_frame->data[0], rgb_frame->linesize[0]);

                            cv::imshow("Robot FPV - Ultra Low Latency", img);
                            if (cv::waitKey(1) == 'q') {
                                return;
                            }
                        }
                    }
                }
            }
        }
    }
};

int main() {
    int width = 640;   // Match robot resolution
    int height = 480;  // Match robot resolution
    int port = 5000;

    H264UDPReceiver receiver(width, height, port);
    receiver.run();

    return 0;
}