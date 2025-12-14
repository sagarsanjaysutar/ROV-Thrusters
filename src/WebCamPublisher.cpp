#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace rclcpp;

/**
 * @brief Run `ros2 run rqt_image_view rqt_image_view` to view custom video frames published /webcam_publisher
 */
class WebCamPublisher : public Node
{
public:
    WebCamPublisher() : Node("webcam_publisher")
    {

        RCLCPP_INFO(this->get_logger(), "Webcam publisher started...");

        m_webCamVideoSubscriber = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw",
            10,
            std::bind(&WebCamPublisher::readWebCamVideo, this, std::placeholders::_1));

        m_customVideoPublisher = this->create_publisher<sensor_msgs::msg::Image>("/webcam_publisher", 10);
    }

private:
    void readWebCamVideo(const sensor_msgs::msg::Image::SharedPtr rawFrame)
    {
        cv_bridge::CvImagePtr cvPtr = cv_bridge ::toCvCopy(rawFrame, "bgr8");
        cv::putText(cvPtr->image, "Sagar Sutar", cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
        m_customVideoPublisher->publish(*cvPtr->toImageMsg());
    }
    Publisher<sensor_msgs::msg::Image>::SharedPtr m_customVideoPublisher;
    Subscription<sensor_msgs::msg::Image>::SharedPtr m_webCamVideoSubscriber;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WebCamPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}