#include <pond/pond.hpp>
#include <pond_data_types/landoltc_types.hpp>
#include <opencv2/opencv.hpp>


class LandoltCDetector : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
private:

    pond::Receiver receiver;
    uint32_t min_edge_count, color_threshold;
    double min_ratio_circle, min_depth;
};

POND_MODULE_CPP_DECLARE(LandoltCDetector, "landoltc_detection_service", "service for detecting landolt cs")

pond_result LandoltCDetector::onStartup(const std::vector<void*>& args)
{
    min_edge_count = parameter("min_edge_count").asInt().get(12);
    min_ratio_circle = parameter("min_ratio_circle").asDouble().get(0.8);
    min_depth = parameter("min_depth").asDouble().get(10.0);
    color_threshold = parameter("color_threshold").asInt().get(100);

    receiver = createReceiver<LandoltcDetectionRequest>({"request"}, [this](LandoltcDetectionRequest* request) {
        
        if (request->image->format != ImgFrame::Format::RGB8 && request->image->format != ImgFrame::Format::BGR8 && request->image->format != ImgFrame::Format::Mono8)
        {
            POND_LOG("Unsupported format %s", ImgFrame::formatToString(request->image->format).c_str());
            return;
        }

        cv::Mat thresholdMat;

        if (request->image->format == ImgFrame::Format::Mono8)
        {
            cv::Mat gray_image(request->image->height, request->image->width, CV_8UC1, request->image->data);
            gray_image.copyTo(thresholdMat);
        }
        else
        {
            cv::Mat color(request->image->height, request->image->width, CV_8UC3, request->image->data);

            if (request->image->format == ImgFrame::Format::RGB8) cv::cvtColor(color, thresholdMat, cv::COLOR_RGB2GRAY);
            else cv::cvtColor(color, thresholdMat, cv::COLOR_BGR2GRAY);
        }

        cv::blur(thresholdMat, thresholdMat, cv::Size(3, 3));
        cv::threshold(thresholdMat, thresholdMat, color_threshold, 255, cv::THRESH_BINARY);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(thresholdMat, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE,cv::Point(0, 0));

        request->detections.clear();
        request->detections.reserve(contours.size());

        for (auto &contour : contours) if (contour.size() > (size_t)min_edge_count)
        {
            std::vector<cv::Point> hull;
            cv::convexHull(contour, hull, true, true);
            double hullArea = cv::contourArea(hull);

            float contourRadius;
            cv::Point2f contourCenter;
            cv::minEnclosingCircle(contour, contourCenter, contourRadius);
            double minArea = contourRadius * contourRadius * M_PI;

            if (hullArea / minArea > min_ratio_circle)
            {
                std::vector<cv::Vec4i> defects;
                std::vector<int> hullsI;

                try
                {
                    cv::convexHull(contour, hullsI, false, false);
                    cv::convexityDefects(contour, hullsI, defects);
                }
                catch (const cv::Exception &e)
                {
                    POND_LOG("OpenCV error while calculating defects: %s", e.what());
                    continue;
                }

                std::vector<cv::Vec4i> deepDefects;
                for (const auto &v : defects) if ((double)v[3] / 256.0 > min_depth) deepDefects.push_back(v);

                if (deepDefects.size() == 1)
                {
                    const cv::Vec4i &v = deepDefects[0];

                    LandoltcDetection detection;
                    detection.midpoint = {contourCenter.x, contourCenter.y};
                    detection.radius = contourRadius;
                    detection.opening[0] = {contour[v[0]].x, contour[v[0]].y};
                    detection.opening[1] = {contour[v[1]].x, contour[v[1]].y};

                    request->detections.push_back(detection);
                }
            }
        }
        request->fulfilled = true;
    });

    return POND_SUCCESS;
}

void LandoltCDetector::onShutdown()
{
    receiver.destroy();
}
