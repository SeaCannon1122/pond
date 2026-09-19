#include <pond/pond.hpp>
#include <pond_data_types/qrcode_types.hpp>
#include <opencv2/opencv.hpp>

#include "quirc/quirc.h"


class QRCodeDetector : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
private:

    quirc* q;
    uint32_t width = 0, height = 0;
    pond::Receiver receiver;
};

POND_MODULE_CPP_DECLARE(QRCodeDetector, "qrcode_detection_service", "service for detecting qrcodes")

pond_result QRCodeDetector::onStartup(const std::vector<void*>& args)
{
    if ((q = quirc_new()) == NULL) return POND_ERROR;

    receiver = createReceiver<QRCodeDetectionRequest>({"request"}, [this](QRCodeDetectionRequest* request) {
        
        if (request->image->format != ImgFrame::Format::RGB8 && request->image->format != ImgFrame::Format::BGR8 && request->image->format != ImgFrame::Format::Mono8)
        {
            POND_LOG("Unsupported format %s", ImgFrame::formatToString(request->image->format).c_str());
            return;
        }

        if (request->image->width != width || request->image->height != height) quirc_resize(q, request->image->width, request->image->height);
        width = request->image->width;
        height = request->image->height;

        cv::Mat gray(height, width, CV_8UC1, quirc_begin(q, NULL, NULL));

        if (request->image->format == ImgFrame::Format::Mono8)
        {
            cv::Mat gray_image(height, width, CV_8UC1, request->image->data);
            gray_image.copyTo(gray);
        }
        else
        {
            cv::Mat color(height, width, CV_8UC3, request->image->data);

            if (request->image->format == ImgFrame::Format::RGB8) cv::cvtColor(color, gray, cv::COLOR_RGB2GRAY);
            else cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);
        }

        quirc_end(q);

        int count = quirc_count(q);
        request->detections.clear();
        request->detections.reserve(count);

        for (int j = 0; j < count; j++)
        {
            struct quirc_code code;
            struct quirc_data data;

            quirc_extract(q, j, &code);

            if (quirc_decode(&code, &data) == QUIRC_SUCCESS)
            {
                QRCodeDetection box;

                for (int k = 0; k < 4; k++) {box.corners[k][0] = code.corners[k].x; box.corners[k][1] = code.corners[k].y; }
                box.data = std::string((char*)data.payload);

                request->detections.push_back(std::move(box));
            }
        }

        request->fulfilled = true;
    });

    return POND_SUCCESS;
}

void QRCodeDetector::onShutdown()
{
    receiver.destroy();
    quirc_destroy(q);
}
