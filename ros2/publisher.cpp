#include <filesystem>
#include <fstream>
#include <cstdio>
#include <errno.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "api/withrobot_camera.hpp"	/* withrobot camera API */

// using namespace std::chrono_literals;

class ImagePublisher : public rclcpp::Node
{
public:
  ImagePublisher() : Node("oCam_publisher")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_topic", 10);
  }
  
  void publish_image(const cv::Mat& img)
  {
    sensor_msgs::msg::Image msg;
    msg.header.frame_id = count_++; // Set the frame ID for the image
    msg.height = img.rows;
    msg.width = img.cols;
    msg.encoding = "rgb8";

    // Convert the OpenCV image to ROS2 sensor_msgs/Image format
    uint32_t size = img.total() * img.elemSize();
    msg.data.resize(size);
    memcpy(&msg.data[0], img.data, size);

    publisher_->publish(msg);
  }

  int32_t count_ = 0;
private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImagePublisher>();

  cv::Mat img = cv::Mat(640, 480, CV_8UC3);
  
  rclcpp::Rate loop_rate(10); // Adjust the publishing rate as needed

  while (rclcpp::ok()) {
      node->publish_image(img);
      std::cout << node->count_ <<std::endl;
      if (node->count_%3 == 0)
        img.setTo(cv::Scalar(255, 0, 0));
      else if (node->count_ % 3 == 1)
        img.setTo(cv::Scalar(0, 255, 0));
      else
        img.setTo(cv::Scalar(0, 0, 255));

      rclcpp::spin_some(node);
      loop_rate.sleep();
  }
  
  rclcpp::shutdown();
  return 0;
}
