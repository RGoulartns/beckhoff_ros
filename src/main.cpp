#include <exception>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "beckhoff_ros_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto beckhoffROS = std::make_shared<beckhoff_ros::BeckhoffROS>();

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 6);
    executor.add_node(beckhoffROS);
    executor.spin();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("beckhoff_ros"), "%s", e.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
