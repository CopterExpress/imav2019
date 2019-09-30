#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <pluginlib/class_list_macros.h>

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

#include <functional>


namespace text_detector
{
class TextDetectorNodelet : public nodelet::Nodelet
{
private:
    ros::NodeHandle nh_, nh_priv_;
    image_transport::Publisher debug_pub_;
    image_transport::Subscriber src_sub_;

    cv::dnn::Net model_;

    float confThreshold_;
    float nmsThreshold_;
    int inputWidth_;
    int inputHeight_;

    void decode(const cv::Mat& scores, const cv::Mat& geometry, float scoreThresh,
                std::vector<cv::RotatedRect>& detections, std::vector<float>& confidences)
    {
        NODELET_INFO("Decoding network output");
        detections.clear();
        const int height = scores.size[2];
        const int width = scores.size[3];
        for(int y = 0; y < height; ++y)
        {
            const float* scoresData = scores.ptr<float>(0, 0, y);
            const float* x0_data = geometry.ptr<float>(0, 0, y);
            const float* x1_data = geometry.ptr<float>(0, 1, y);
            const float* x2_data = geometry.ptr<float>(0, 2, y);
            const float* x3_data = geometry.ptr<float>(0, 3, y);
            const float* anglesData = geometry.ptr<float>(0, 4, y);
            for(int x = 0; x < width; ++x)
            {
                float score = scoresData[x];
                if (score < scoreThresh) continue;

                float offsetX = x * 4.0f, offsetY = y * 4.0f;
                float angle = anglesData[x];
                float cosA = std::cos(angle);
                float sinA = std::sin(angle);
                float h = x0_data[x] + x2_data[x];
                float w = x1_data[x] + x3_data[x];

                cv::Point2f offset(offsetX + cosA * x1_data[x] + sinA * x2_data[x],
                           offsetY - sinA * x1_data[x] + cosA * x2_data[x]);
                cv::Point2f p1 = cv::Point2f(-sinA * h, -cosA * h) + offset;
                cv::Point2f p3 = cv::Point2f(-cosA * w, sinA * w) + offset;
                cv::RotatedRect r(0.5f * (p1 + p3), cv::Size2f(w, h), -angle * 180.0f / (float)CV_PI);
                detections.push_back(r);
                confidences.push_back(score);
            }
        }
    }

std::vector<cv::String> outNames{"feature_fusion/Conv_7/Sigmoid", "feature_fusion/concat_3"};

void detectionCallback(const sensor_msgs::ImageConstPtr &image)
{
    std::vector<cv::Mat> outs(2);
    bool hasSubscribers = (debug_pub_.getNumSubscribers() > 0);
    NODELET_INFO("Finding text in image");
    cv_bridge::CvImageConstPtr sourceImg = cv_bridge::toCvShare(image);
    cv::Mat blob;
    NODELET_INFO("Generating input from image");
    cv::dnn::blobFromImage(sourceImg->image, blob, 1.0, cv::Size(inputWidth_, inputHeight_), cv::Scalar(123.68, 116.78, 103.94), true, false);
    NODELET_INFO("Setting up input for model");
    model_.setInput(blob);
    NODELET_INFO("Setting up outputs and their names");
    model_.forward(outs, outNames);

    cv::Mat scores = outs[0];
    cv::Mat geometry = outs[1];

    std::vector<cv::RotatedRect> boxes;
    std::vector<float> confidences;
    NODELET_INFO("Starting to decode outputs");
    decode(scores, geometry, confThreshold_, boxes, confidences);

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, confThreshold_, nmsThreshold_, indices);
    NODELET_INFO("Found %ld potential texts", indices.size());
    if (debug_pub_.getNumSubscribers() > 0)
    {
        cv_bridge::CvImagePtr debugImg = cv_bridge::toCvCopy(image);
        // Render detections.
        NODELET_INFO("Publishing debug image");
        cv::Point2f ratio((float)sourceImg->image.cols / inputWidth_, (float)sourceImg->image.rows / inputHeight_);
        for (size_t i = 0; i < indices.size(); ++i)
        {
            cv::RotatedRect& box = boxes[indices[i]];

            cv::Point2f vertices[4];
            box.points(vertices);
            for (int j = 0; j < 4; ++j)
            {
                vertices[j].x *= ratio.x;
                vertices[j].y *= ratio.y;
            }
            for (int j = 0; j < 4; ++j)
            cv::line(debugImg->image, vertices[j], vertices[(j + 1) % 4], cv::Scalar(0, 255, 0), 1);
        }
        debug_pub_.publish(debugImg->toImageMsg());
    }
}

public:
    TextDetectorNodelet() {}
    ~TextDetectorNodelet() {}
    void onInit()
    {
        nh_ = getNodeHandle();
        nh_priv_ = getPrivateNodeHandle();
        image_transport::ImageTransport it(nh_), it_priv(nh_priv_);

        std::string modelName = nh_priv_.param<std::string>("model_name", "");
        confThreshold_ = nh_priv_.param("confidence_threshold", 0.6);
        nmsThreshold_ = nh_priv_.param("nms_threshold", 0.4);
        inputWidth_ = nh_priv_.param("input_width", 320);
        inputHeight_ = nh_priv_.param("input_height", 320);

        NODELET_INFO("Loading model: %s", modelName.c_str());

        model_ = cv::dnn::readNet(modelName);

        NODELET_INFO("Subscribing to image topic");

        debug_pub_ = it_priv.advertise("debug", 1);
        src_sub_ = it.subscribe("image", 1, &TextDetectorNodelet::detectionCallback, this);
    }

};

}

PLUGINLIB_EXPORT_CLASS(text_detector::TextDetectorNodelet, nodelet::Nodelet);
