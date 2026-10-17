#include "beckhoff_ros_node.hpp"

#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "ament_index_cpp/get_package_share_directory.hpp"
#if __has_include("rclcpp/version.h")
#include "rclcpp/version.h"
#endif

namespace beckhoff_ros
{

namespace
{

nlohmann::ordered_json loadConfig(const std::string & path)
{
  std::ifstream file(path);
  if (!file) {
    throw ConfigError("Unable to open the configuration file '" + path + "'");
  }
  try {
    // ordered_json keeps the fields of the structures in the order of the file
    return nlohmann::ordered_json::parse(file);
  } catch (const nlohmann::json::exception & e) {
    throw ConfigError("Unable to parse the configuration file '" + path + "': " + e.what());
  }
}

}  // namespace

BeckhoffROS::BeckhoffROS(const rclcpp::NodeOptions & options)
: rclcpp::Node("beckhoff_ros", options)
{
  // The PLC is described by a JSON file: ROS parameters cannot describe nested structures.
  std::string config_file = declare_parameter<std::string>("config_file", "");
  if (config_file.empty()) {
    config_file = ament_index_cpp::get_package_share_directory("beckhoff_ros") +
      "/config/params.json";
  }
  const nlohmann::ordered_json jsonData = loadConfig(config_file);
  if (!jsonData.contains("PLC")) {
    throw ConfigError("The configuration file '" + config_file + "' has no \"PLC\" object");
  }

  cbGroup_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  varChanged_pub_ = create_publisher<PlcVariable>("variable_state", 1);

  const rclcpp::Logger logger = get_logger();
  plc_ = std::make_unique<BeckhoffPLC>(
    jsonData.at("PLC"),
    [logger](BeckhoffPLC::LogLevel level, const std::string & message) {
      switch (level) {
        case BeckhoffPLC::LogLevel::Debug:
          RCLCPP_DEBUG(logger, "%s", message.c_str());
          break;
        case BeckhoffPLC::LogLevel::Info:
          RCLCPP_INFO(logger, "%s", message.c_str());
          break;
        case BeckhoffPLC::LogLevel::Warn:
          RCLCPP_WARN(logger, "%s", message.c_str());
          break;
        case BeckhoffPLC::LogLevel::Error:
          RCLCPP_ERROR(logger, "%s", message.c_str());
          break;
      }
    });
  RCLCPP_INFO(
    get_logger(), "Connected to the PLC %s (configuration: %s)",
    jsonData.at("PLC").at("net_id").get<std::string>().c_str(), config_file.c_str());
  plc_->addObserver(
    std::bind(&BeckhoffROS::onPlcChanged, this, std::placeholders::_1, std::placeholders::_2));

#if defined(RCLCPP_VERSION_MAJOR) && RCLCPP_VERSION_MAJOR >= 28
  const auto qos = rclcpp::ServicesQoS();  // rmw_qos_profile_t is deprecated since Jazzy
#else
  const auto qos = rmw_qos_profile_services_default;
#endif
  writeVar_srv_ = create_service<WritePlcVariable>(
    "write_variable",
    std::bind(&BeckhoffROS::writeVarSrvCB, this, std::placeholders::_1, std::placeholders::_2),
    qos, cbGroup_);
}

void BeckhoffROS::onPlcChanged(const std::string & varName, const Value & varContents)
{
  PlcVariable msg;
  msg.name = varName;
  msg.type = varContents.typeString();
  msg.value = varContents.str();
  RCLCPP_DEBUG(get_logger(), "new message: %s %s", varName.c_str(), msg.value.c_str());
  varChanged_pub_->publish(msg);
}

void BeckhoffROS::writeVarSrvCB(
  const std::shared_ptr<WritePlcVariable::Request> request,
  std::shared_ptr<WritePlcVariable::Response> response)
{
  const auto & variable = request->variable;
  try {
    const Variable * target = plc_->findVariable(variable.name);
    if (target == nullptr || !target->writable()) {
      throw ConfigError("Variable '" + variable.name + "' is not listed in variables_to_write");
    }
    plc_->setVariable(variable.name, parseTypedValue(variable.type, variable.value));
    response->result = true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Unable to write '%s': %s", variable.name.c_str(), e.what());
    response->result = false;
  }
}

}  // namespace beckhoff_ros
