// Copyright 2026 lunarifish
// SPDX-License-Identifier: MIT
//
// Turns decoded blackbox rows into ROS messages and writes them to a bag.
//
// Design notes that matter for the output quality:
//   * blackbox_decode forward-fills the S and G frame values onto every main
//     frame row, so a naive writer would emit 250 Hz copies of data that really
//     changes at ~100 Hz (S) and ~10 Hz (G). Those topics are therefore written
//     on change instead of on every row.
//   * The nav position (North, East, Down) and the GPS track (from lat/lon) live
//     in different origins. The GPS track is anchored into the nav frame at the
//     first valid fix so both paths overlay in rviz2 / Foxglove, and the anchor
//     also gives the position of the GPS home point for the markers.
//   * The navigation path and the GPS track are written as one nav_msgs/Path
//     each at the end, rather than as per-tick PoseStamped, which keeps the 3D
//     panels responsive.

#ifndef INAV_BLACKBOX_ROS2__TOPIC_WRITER_HPP_
#define INAV_BLACKBOX_ROS2__TOPIC_WRITER_HPP_

#include <cstdint>
#include <iosfwd>
#include <string>
#include <vector>

#include <nav_msgs/msg/path.hpp>

#include "inav_blackbox_ros2/bag_writer.hpp"
#include "inav_blackbox_ros2/csv_table.hpp"
#include "inav_blackbox_ros2/inav_sample.hpp"

namespace inav_blackbox_ros2
{

struct TopicWriterOptions
{
  std::string topic_prefix = "/inav";
  std::string frame_map = "map";
  std::string frame_base_link = "base_link";
  /// Keep every Nth main-frame row for the high rate topics (default: all).
  int decimate = 1;
  /// Rate limit for /tf, in Hz.
  double tf_rate = 50.0;
  /// Rate limit for the /plot scalar topics, in Hz.
  double plot_rate = 50.0;
  /// Use the barometric altitude for the nav path / odometry height, since the
  /// navigation estimator's own vertical reference can sit metres away from it.
  bool nav_alt_from_baro = false;
  bool write_paths = true;
  bool write_plots = true;
  bool write_imu = true;
  /// Microtesla per magnetometer count; 0 publishes raw counts.
  double mag_scale = 0.0;
  /// Cells used for the battery percentage; 0 derives it from the pack voltage.
  int battery_cells = 0;
};

class TopicWriter
{
public:
  TopicWriter(BagWriter & bag, TopicWriterOptions options);

  /// Locks in the log's available columns and where timestamp zero is.
  /// `start_epoch_ns` is the absolute UTC time of the log's first row, or 0 to
  /// start the bag at 0 (--relative).
  void start_log(const CsvTable & table, const InavColumns & columns, int64_t start_epoch_ns);

  /// Consumes one decoded row. Rows must arrive in log order.
  void add_row(const CsvRow & row);

  /// Flushes the accumulated paths and markers for the log.
  void finish_log();

  /// Writes which topic groups were skipped because the log lacks the columns.
  void report_skipped(std::ostream & out) const;

private:
  struct PlotTopic
  {
    int column = -1;
    std::string topic;
    double scale = 1.0;
  };

  void emit_imu(int64_t stamp_ns, const CsvRow & row);
  void emit_pose_and_odom(int64_t stamp_ns, const CsvRow & row);
  void emit_tf(int64_t stamp_ns, const CsvRow & row);
  void emit_gps(int64_t stamp_ns, const CsvRow & row);
  void emit_status(int64_t stamp_ns, const CsvRow & row);
  void emit_battery(int64_t stamp_ns, const CsvRow & row);
  void emit_rc(int64_t stamp_ns, const CsvRow & row);
  void emit_scalars(int64_t stamp_ns, const CsvRow & row);
  void emit_plots(int64_t stamp_ns, const CsvRow & row);
  void emit_markers(int64_t stamp_ns, const CsvRow & row);
  void emit_nav_target(int64_t stamp_ns, const CsvRow & row);
  void publish_mode_text(int64_t stamp_ns, const CsvRow & row);
  /// Appends this row to the accumulated nav and GPS tracks.
  void accumulate_paths(int64_t stamp_ns, const CsvRow & row);
  /// Fixes the GPS track's placement inside the nav frame at the first fix.
  void update_gps_anchor(const CsvRow & row, double lat, double lon, int fix_type);

  /// Positions of the craft in the map frame, in metres.
  void nav_position(const CsvRow & row, double out_m[3]) const;
  void craft_orientation(const CsvRow & row, double out_xyzw[4]) const;

  std::string topic(const std::string & suffix) const { return options_.topic_prefix + suffix; }

  BagWriter & bag_;
  TopicWriterOptions options_;
  InavColumns columns_;
  int64_t start_epoch_ns_ = 0;
  int64_t first_time_us_ = 0;
  int64_t last_stamp_ns_ = 0;
  int64_t row_counter_ = 0;
  int64_t last_tf_ns_ = 0;
  int64_t last_plot_ns_ = 0;
  int64_t last_marker_ns_ = 0;

  // Change detectors: blackbox_decode forward-fills S/G values onto every row,
  // so these suppress the repeats.
  std::string last_gps_key_;
  std::string last_status_key_;
  std::string last_battery_key_;
  std::string last_rc_key_;
  std::string last_scalar_key_;
  std::string last_text_key_;

  // GPS -> nav frame anchor, fixed at the first valid fix.
  bool anchor_set_ = false;
  bool home_is_gps_home_ = false;
  double home_in_nav_m_[2] = {0.0, 0.0};
  double home_alt_m_ = 0.0;
  double home_nav_z_m_ = 0.0;
  double ref_lat_deg_ = 0.0;
  double ref_lon_deg_ = 0.0;
  bool gps_home_seen_ = false;

  nav_msgs::msg::Path nav_path_;
  nav_msgs::msg::Path gps_path_;
  std::vector<PlotTopic> plots_;

  // Which groups the loaded log cannot feed.
  std::vector<std::string> skipped_groups_;

  bool nav_path_written_ = false;
  bool gps_path_written_ = false;
  bool home_marker_written_ = false;
  bool gps_home_marker_written_ = false;
};

}  // namespace inav_blackbox_ros2

#endif  // INAV_BLACKBOX_ROS2__TOPIC_WRITER_HPP_
