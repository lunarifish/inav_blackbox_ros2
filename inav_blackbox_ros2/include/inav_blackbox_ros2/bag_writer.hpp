// Copyright 2026 lunarifish
// SPDX-License-Identifier: MIT
//
// Thin rosbag2 wrapper that takes care of the two things a hand-rolled bag
// needs: registering every topic with its full message definition (Foxglove
// refuses to open an MCAP whose schema encoding is empty) and writing messages
// with an explicit timestamp.

#ifndef INAV_BLACKBOX_ROS2__BAG_WRITER_HPP_
#define INAV_BLACKBOX_ROS2__BAG_WRITER_HPP_

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/message_definition.hpp>
#include <rosidl_runtime_cpp/traits.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/time.hpp>

namespace inav_blackbox_ros2
{

class BagWriter
{
public:
  /// Creates the bag at `uri` (a directory for the mcap plugin).
  BagWriter(const std::string & uri, const std::string & storage_id);

  ~BagWriter();

  /// Writes one message with `stamp_ns` as both its header stamp source and the
  /// bag's receive/send time. Topics are created on first use.
  template<typename MessageT>
  void write(const MessageT & message, const std::string & topic, int64_t stamp_ns)
  {
    ensure_topic(topic, rosidl_generator_traits::name<MessageT>());
    writer_.write(message, topic, rclcpp::Time(stamp_ns, RCL_SYSTEM_TIME));
    ++message_count_;
  }

  /// Writes the same message to every topic in `topics` (used for /tf).
  template<typename MessageT>
  void write(const MessageT & message, const std::vector<std::string> & topics, int64_t stamp_ns)
  {
    for (const std::string & topic : topics) {
      write(message, topic, stamp_ns);
    }
  }

  /// Number of messages written so far.
  size_t message_count() const { return message_count_; }

  /// The topics created so far, name -> message type, in name order.
  const std::map<std::string, std::string> & topics() const { return topic_types_; }

private:
  void ensure_topic(const std::string & topic, const std::string & type);

  rosbag2_cpp::Writer writer_;
  std::map<std::string, std::string> topic_types_;
  std::map<std::string, rosbag2_storage::MessageDefinition> definitions_;
  size_t message_count_ = 0;
};

}  // namespace inav_blackbox_ros2

#endif  // INAV_BLACKBOX_ROS2__BAG_WRITER_HPP_
