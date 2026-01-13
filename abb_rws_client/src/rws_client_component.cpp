#include <rclcpp_components/register_node_macro.hpp>
#include "abb_rws_client/rws_client_component.hpp"

namespace abb_rws_client {

RWSClientComponent::RWSClientComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("rws_client_component", options), logger_(this->get_logger())
{
  this->declare_parameter("robot_nickname", std::string{});
  this->declare_parameter("no_connection_timeout", false);
  this->declare_parameter<std::string>("robot_ip", "127.0.0.1");
  this->declare_parameter<int>("robot_port", 80);

  std::string robot_ip = this->get_parameter("robot_ip").as_string();
  int robot_port = this->get_parameter("robot_port").as_int();

  auto node = rclcpp::Node::SharedPtr(this, [](rclcpp::Node*){});

  srv_provider_ = std::make_shared<abb_rws_client::RWSServiceProviderROS>(
      node, robot_ip, static_cast<unsigned short>(robot_port));

  state_publisher_ = std::make_shared<abb_rws_client::RWSStatePublisherROS>(
      node, robot_ip, static_cast<unsigned short>(robot_port));
}

}  // namespace abb_rws_client

RCLCPP_COMPONENTS_REGISTER_NODE(abb_rws_client::RWSClientComponent)