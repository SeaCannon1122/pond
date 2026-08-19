#include <pond/data_types/data_types.hpp>
#include <opencv2/opencv.hpp>

class CVImgFrame : public ImgFrame
{
public:

    explicit CVImgFrame(cv::Mat& frame_, ImgFrame::Format format_) : frame(frame_)
    {

        data = frame.data;
        width = frame.cols;
        height = frame.rows;
        pixel_size = frame.elemSize();
        format = format_;
    }

private:
    cv::Mat frame;
};