/*
==============================================================================
MIT License

Copyright (c) 2024 Ethan M Brown

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
==============================================================================
*/

#pragma once

#include <rclcpp/rclcpp.hpp>

#ifdef ROS_FOXY

// Foxy compatibility layer
// - GenericPublisher does not exist in Foxy's rclcpp (added in Galactic)
// - rclcpp::Serialization<T> does not exist in Foxy (added in Galactic)
// - tf2_ros::StaticListenerQoS / DynamicListenerQoS do not exist in Foxy

#include <string>
#include <memory>
#include <stdexcept>

#include <rcl/publisher.h>
#include <rcl/error_handling.h>
#include <rcpputils/shared_library.hpp>
#include <rmw/rmw.h>
#include <rmw/serialized_message.h>
#include <rosidl_typesupport_cpp/message_type_support.hpp>

namespace network_bridge
{
namespace compat
{

/**
 * @brief A minimal generic publisher for Foxy that can publish serialized
 * messages for types not known at compile time.
 *
 * Uses the rcl API directly since rclcpp::GenericPublisher was not available
 * until Galactic.
 */
class FoxyGenericPublisher
{
public:
  using SharedPtr = std::shared_ptr<FoxyGenericPublisher>;

  FoxyGenericPublisher(
    rclcpp::Node * node,
    const std::string & topic,
    const std::string & type,
    const rclcpp::QoS & qos)
  : initialized_(false)
  {
    const auto * ts = load_type_support(type);

    rcl_publisher_ = rcl_get_zero_initialized_publisher();
    rcl_publisher_options_t options = rcl_publisher_get_default_options();
    options.qos = qos.get_rmw_qos_profile();

    rcl_node_ = node->get_node_base_interface()->get_rcl_node_handle();

    auto ret = rcl_publisher_init(
      &rcl_publisher_, rcl_node_, ts, topic.c_str(), &options);
    if (ret != RCL_RET_OK) {
      throw std::runtime_error(
              std::string("Failed to create generic publisher: ") +
              rcl_get_error_string().str);
    }
    initialized_ = true;
  }

  ~FoxyGenericPublisher()
  {
    if (initialized_) {
      rcl_publisher_fini(&rcl_publisher_, rcl_node_);
    }
  }

  // Non-copyable
  FoxyGenericPublisher(const FoxyGenericPublisher &) = delete;
  FoxyGenericPublisher & operator=(const FoxyGenericPublisher &) = delete;

  void publish(const rclcpp::SerializedMessage & msg)
  {
    auto ret = rcl_publish_serialized_message(
      &rcl_publisher_, &msg.get_rcl_serialized_message(), nullptr);
    if (ret != RCL_RET_OK) {
      throw std::runtime_error(
              std::string("Failed to publish serialized message: ") +
              rcl_get_error_string().str);
    }
  }

private:
  const rosidl_message_type_support_t * load_type_support(const std::string & type)
  {
    // Parse type string, e.g. "std_msgs/msg/String"
    auto pos1 = type.find('/');
    auto pos2 = type.find('/', pos1 + 1);
    if (pos1 == std::string::npos || pos2 == std::string::npos) {
      throw std::runtime_error("Invalid message type format: " + type);
    }

    std::string package = type.substr(0, pos1);
    std::string subfolder = type.substr(pos1 + 1, pos2 - pos1 - 1);
    std::string name = type.substr(pos2 + 1);

    // Load the typesupport shared library
    std::string lib_name = "lib" + package + "__rosidl_typesupport_cpp.so";
    typesupport_library_ = std::make_shared<rcpputils::SharedLibrary>(lib_name);

    // Look up the typesupport handle symbol
    std::string symbol_name =
      "rosidl_typesupport_cpp__get_message_type_support_handle__" +
      package + "__" + subfolder + "__" + name;

    if (!typesupport_library_->has_symbol(symbol_name)) {
      throw std::runtime_error("Could not find typesupport symbol: " + symbol_name);
    }

    typedef const rosidl_message_type_support_t * (* GetTypeSupportFunc)();
    auto func = reinterpret_cast<GetTypeSupportFunc>(
      typesupport_library_->get_symbol(symbol_name));

    return func();
  }

  rcl_publisher_t rcl_publisher_;
  rcl_node_t * rcl_node_;
  std::shared_ptr<rcpputils::SharedLibrary> typesupport_library_;
  bool initialized_;
};

/**
 * @brief Serialize a ROS message to a SerializedMessage using rmw_serialize.
 *
 * Foxy does not have rclcpp::Serialization<T>::serialize_message().
 */
template<typename MessageT>
void serialize_message(
  const MessageT & msg,
  rclcpp::SerializedMessage * serialized_msg)
{
  const auto * ts =
    rosidl_typesupport_cpp::get_message_type_support_handle<MessageT>();
  auto & rcl_msg = serialized_msg->get_rcl_serialized_message();
  auto ret = rmw_serialize(&msg, ts, &rcl_msg);
  if (ret != RMW_RET_OK) {
    throw std::runtime_error("Failed to serialize message");
  }
}

/**
 * @brief Foxy-compatible QoS for static TF listeners.
 *
 * Equivalent to tf2_ros::StaticListenerQoS (added post-Foxy).
 */
inline rclcpp::QoS static_tf_qos()
{
  return rclcpp::QoS(100).keep_all().transient_local().reliable();
}

/**
 * @brief Foxy-compatible QoS for dynamic TF listeners.
 *
 * Equivalent to tf2_ros::DynamicListenerQoS (added post-Foxy).
 */
inline rclcpp::QoS dynamic_tf_qos()
{
  return rclcpp::SensorDataQoS();
}

/**
 * @brief Get the size of a serialized message.
 *
 * Foxy may not have SerializedMessage::size().
 */
inline size_t serialized_message_size(const rclcpp::SerializedMessage & msg)
{
  return msg.get_rcl_serialized_message().buffer_length;
}

namespace detail
{
inline rcl_subscription_options_t get_subscription_options(const rclcpp::QoS & qos)
{
  auto options = rcl_subscription_get_default_options();
  options.qos = qos.get_rmw_qos_profile();
  return options;
}
}  // namespace detail

/**
 * @brief A generic subscription for Foxy that subscribes to serialized messages
 * whose topic type is not known at compile time.
 *
 * Adapted from rosbag2_transport::GenericSubscription (ROS 2 Foxy branch).
 * Uses rclcpp::SubscriptionBase directly since Foxy's create_generic_subscription
 * has a different callback signature than later distros.
 */
class FoxyGenericSubscription : public rclcpp::SubscriptionBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(FoxyGenericSubscription)

  FoxyGenericSubscription(
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    const rosidl_message_type_support_t & ts,
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> callback)
  : SubscriptionBase(
      node_base,
      ts,
      topic_name,
      detail::get_subscription_options(qos),
      true),
    default_allocator_(rcutils_get_default_allocator()),
    callback_(callback),
    qos_(qos)
  {}

  std::shared_ptr<void> create_message() override
  {
    return create_serialized_message();
  }

  std::shared_ptr<rclcpp::SerializedMessage> create_serialized_message() override
  {
    return std::make_shared<rclcpp::SerializedMessage>(0);
  }

  void handle_message(
    std::shared_ptr<void> & message, const rclcpp::MessageInfo & message_info) override
  {
    (void) message_info;
    auto typed_message = std::static_pointer_cast<rclcpp::SerializedMessage>(message);
    callback_(typed_message);
  }

  void handle_loaned_message(
    void * message, const rclcpp::MessageInfo & message_info) override
  {
    (void) message;
    (void) message_info;
  }

  void return_message(std::shared_ptr<void> & message) override
  {
    auto typed_message = std::static_pointer_cast<rclcpp::SerializedMessage>(message);
    return_serialized_message(typed_message);
  }

  void return_serialized_message(
    std::shared_ptr<rclcpp::SerializedMessage> & message) override
  {
    message.reset();
  }

  const rclcpp::QoS & qos_profile() const
  {
    return qos_;
  }

private:
  RCLCPP_DISABLE_COPY(FoxyGenericSubscription)

  rcutils_allocator_t default_allocator_;
  std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> callback_;
  const rclcpp::QoS qos_;
};

/**
 * @brief Create a FoxyGenericSubscription and register it with the node.
 *
 * @param node The ROS 2 node
 * @param topic Topic name
 * @param type Message type string (e.g. "std_msgs/msg/String")
 * @param qos QoS profile
 * @param callback Callback for serialized messages
 * @return Shared pointer to the created subscription
 */
inline FoxyGenericSubscription::SharedPtr create_generic_subscription(
  rclcpp::Node * node,
  const std::string & topic,
  const std::string & type,
  const rclcpp::QoS & qos,
  std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> callback)
{
  // Load type support from the message type string
  auto pos1 = type.find('/');
  auto pos2 = type.find('/', pos1 + 1);
  if (pos1 == std::string::npos || pos2 == std::string::npos) {
    throw std::runtime_error("Invalid message type format: " + type);
  }

  std::string package = type.substr(0, pos1);
  std::string subfolder = type.substr(pos1 + 1, pos2 - pos1 - 1);
  std::string name = type.substr(pos2 + 1);

  std::string lib_name = "lib" + package + "__rosidl_typesupport_cpp.so";
  auto typesupport_library = std::make_shared<rcpputils::SharedLibrary>(lib_name);

  std::string symbol_name =
    "rosidl_typesupport_cpp__get_message_type_support_handle__" +
    package + "__" + subfolder + "__" + name;

  if (!typesupport_library->has_symbol(symbol_name)) {
    throw std::runtime_error("Could not find typesupport symbol: " + symbol_name);
  }

  typedef const rosidl_message_type_support_t * (* GetTypeSupportFunc)();
  auto func = reinterpret_cast<GetTypeSupportFunc>(
    typesupport_library->get_symbol(symbol_name));
  const auto * ts = func();

  auto subscription = std::make_shared<FoxyGenericSubscription>(
    node->get_node_base_interface().get(),
    *ts,
    topic,
    qos,
    callback);

  node->get_node_topics_interface()->add_subscription(subscription, nullptr);

  return subscription;
}

}  // namespace compat
}  // namespace network_bridge

// Convenience type aliases
using GenericPublisherSharedPtr = network_bridge::compat::FoxyGenericPublisher::SharedPtr;
using GenericSubscriptionSharedPtr = network_bridge::compat::FoxyGenericSubscription::SharedPtr;

#else  // !ROS_FOXY (Galactic and later)

// Convenience type aliases - use standard rclcpp types
using GenericPublisherSharedPtr = rclcpp::GenericPublisher::SharedPtr;
using GenericSubscriptionSharedPtr = rclcpp::GenericSubscription::SharedPtr;

#endif  // ROS_FOXY
