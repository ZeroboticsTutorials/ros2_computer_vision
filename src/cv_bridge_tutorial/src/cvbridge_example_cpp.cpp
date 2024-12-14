// the library ROS2 C++
#include "rclcpp/rclcpp.hpp"
// ROS image message
#include "sensor_msgs/msg/image.hpp"
// This library provides the bridge for converting ROS image <-> OpenCV
#include "cv_bridge/cv_bridge.hpp"
// The library used for image processing
#include "opencv2/opencv.hpp"

class ImageProcessor : public rclcpp::Node
{
public:
    ImageProcessor() : Node("image_processor")
    {
        // Subscribe on topic /camera/image_raw
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10, std::bind(&ImageProcessor::imageCallback, this, std::placeholders::_1));

        // Create a publisher to republish images after processing
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("processed_image_topic_cpp", 10);
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            // Create a cv_brigde pointer
            cv_bridge::CvImagePtr cv_ptr;
            // Convert the ROS message image received into an OpenCV image using toCvCopy
            // The OpenCV image can be accessed using cv_ptr->image
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            /*
            Once your image has been converted to OpenCV format, you can use all the methods
            provided by the opencv2/opencv C++ library.
            */
            // Create a cv::Mat (OpenCV Object) object which to store the image
            cv::Mat grayscale_image;

            // Converts the bridge image to greyscale and assigns it to grayscale_image
            cv::cvtColor(cv_ptr->image, grayscale_image, cv::COLOR_BGR2GRAY);

            // Create another cv::Mat object to receive a new image
            cv::Mat thresholded_image;

            // Performs thresholding on the greyscale image and assigns the output image
            // to thresholded_image
            cv::threshold(grayscale_image, thresholded_image, 128, 255, cv::THRESH_BINARY);

            // Converts the OpenCV image into a ROS message image
            sensor_msgs::msg::Image::SharedPtr processed_msg = cv_bridge::CvImage(
                                                                   std_msgs::msg::Header(),
                                                                   sensor_msgs::image_encodings::MONO8,
                                                                   thresholded_image)
                                                                   .toImageMsg();

            // Publish the resulting image on the processed_image_topic topic
            publisher_->publish(*processed_msg);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}