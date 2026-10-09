// Copyright 2026 lunarifish
// SPDX-License-Identifier: MIT

#include "inav_blackbox_ros2/bag_writer.hpp"

#include <stdexcept>

#include <rosbag2_cpp/message_definitions/local_message_definition_source.hpp>
#include <rosbag2_storage/storage_options.hpp>

namespace inav_blackbox_ros2
{

BagWriter::BagWriter(const std::string & uri, const std::string & storage_id)
{
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = uri;
  storage_options.storage_id = storage_id;
  storage_options.max_cache_size = 100 * 1024 * 1024;
  writer_.open(storage_options);
}

BagWriter::~BagWriter()
{
  try {
    writer_.close();
  } catch (const std::exception &) {
    // Nothing useful to do while unwinding; the metadata may simply be absent.
  }
}

void BagWriter::ensure_topic(const std::string & topic, const std::string & type)
{
  const auto existing = topic_types_.find(topic);
  if (existing != topic_types_.end()) {
    return;
  }

  rosbag2_storage::TopicMetadata metadata;
  metadata.name = topic;
  metadata.type = type;
  metadata.serialization_format = "cdr";

  auto definition = definitions_.find(type);
  if (definition == definitions_.end()) {
    // Foxglove and `ros2 bag` need the self-contained schema text, not just the
    // type name; an empty encoding makes the file unreadable for them.
    rosbag2_cpp::LocalMessageDefinitionSource source;
    definition = definitions_.emplace(type, source.get_full_text(type)).first;
  }

  writer_.create_topic(metadata, definition->second);
  topic_types_.emplace(topic, type);
}

}  // namespace inav_blackbox_ros2
