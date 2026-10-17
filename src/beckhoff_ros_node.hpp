#ifndef BECKHOFF_ROS_NODE_HPP_
#define BECKHOFF_ROS_NODE_HPP_

#include <memory>
#include <string>

#include "beckhoff_interfaces/msg/plc_variable.hpp"
#include "beckhoff_interfaces/srv/write_plc_variable.hpp"
#include "rclcpp/rclcpp.hpp"

#include "beckhoff_ros/beckhoff_plc.hpp"
#include "beckhoff_ros/value.hpp"

namespace beckhoff_ros
{

/// ROS 2 interface of a Beckhoff PLC (port of beckhoff_ros/beckhoff_ros_node.py).
///
/// Parameters:
///   config_file  JSON file describing the PLC (see BeckhoffPLC), default: the params.json
///                installed in this package's share directory
/// Publishes the monitored variables on "variable_state" (beckhoff_interfaces/msg/PlcVariable,
/// with the type and value formatted like Python's str(type(value)) and str(value)) and writes
/// the variables of variables_to_write with the "write_variable" service
/// (beckhoff_interfaces/srv/WritePlcVariable).
class BeckhoffROS : public rclcpp::Node
{
public:
  using PlcVariable = beckhoff_interfaces::msg::PlcVariable;
  using WritePlcVariable = beckhoff_interfaces::srv::WritePlcVariable;

  explicit BeckhoffROS(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onPlcChanged(const std::string & varName, const Value & varContents);
  void writeVarSrvCB(
    const std::shared_ptr<WritePlcVariable::Request> request,
    std::shared_ptr<WritePlcVariable::Response> response);

  rclcpp::CallbackGroup::SharedPtr cbGroup_;
  rclcpp::Publisher<PlcVariable>::SharedPtr varChanged_pub_;
  rclcpp::Service<WritePlcVariable>::SharedPtr writeVar_srv_;
  std::unique_ptr<BeckhoffPLC> plc_;  // last member: stopped before the publisher is destroyed
};

}  // namespace beckhoff_ros

#endif  // BECKHOFF_ROS_NODE_HPP_
