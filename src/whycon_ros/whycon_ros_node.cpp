#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "whycon_ros/CWhyconROSNode.h"


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<whycode_ros2::CWhyconROSNode>());
    rclcpp::shutdown();

    return 0;
}