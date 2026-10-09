// Copyright 2026 lunarifish
// SPDX-License-Identifier: MIT

#include "inav_blackbox_ros2/topic_writer.hpp"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <ostream>
#include <stdexcept>
#include <utility>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <inav_blackbox_msgs/msg/inav_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "inav_blackbox_ros2/inav_frames.hpp"

namespace inav_blackbox_ros2
{

namespace
{

void set_stamp(builtin_interfaces::msg::Time & stamp, int64_t nanoseconds)
{
  int64_t seconds = nanoseconds / 1000000000;
  int64_t rest = nanoseconds % 1000000000;
  if (rest < 0) {
    seconds -= 1;
    rest += 1000000000;
  }
  stamp.sec = static_cast<int32_t>(seconds);
  stamp.nanosec = static_cast<uint32_t>(rest);
}

/// True when any of `columns` changed since the last call. blackbox_decode
/// forward-fills S and G frame values onto every main frame row, so comparing
/// the raw text is exactly the "did this row carry new data" test.
bool key_changed(std::string & last, const CsvRow & row, std::initializer_list<int> columns)
{
  std::string key;
  for (const int column : columns) {
    key += row.text(column);
    key += '\x1f';
  }
  if (key == last) {
    return false;
  }
  last = std::move(key);
  return true;
}

int64_t period_ns(double rate_hz)
{
  if (rate_hz <= 0.0) {
    return 0;
  }
  return static_cast<int64_t>(1000000000.0 / rate_hz);
}

/// Scalar blackbox columns worth exposing as plottable <prefix>/plot/<name>
/// topics. Only the columns a log actually has are created, so this one table
/// serves INAV logs (axisRate, navTgt...), Betaflight logs (setpoint,
/// gyroUnfilt...) and older firmware alike.
struct PlotSignalDef
{
  const char * column;
  const char * name;
  double scale;
};

const PlotSignalDef kPlotSignals[] = {
  {"axisRate[0]", "axisRate_0", 1.0},
  {"axisRate[1]", "axisRate_1", 1.0},
  {"axisRate[2]", "axisRate_2", 1.0},
  {"setpoint[0]", "setpoint_0", 1.0},
  {"setpoint[1]", "setpoint_1", 1.0},
  {"setpoint[2]", "setpoint_2", 1.0},
  {"setpoint[3]", "setpoint_3", 1.0},
  {"axisP[0]", "axisP_0", 1.0},
  {"axisP[1]", "axisP_1", 1.0},
  {"axisP[2]", "axisP_2", 1.0},
  {"axisI[0]", "axisI_0", 1.0},
  {"axisI[1]", "axisI_1", 1.0},
  {"axisI[2]", "axisI_2", 1.0},
  {"axisD[0]", "axisD_0", 1.0},
  {"axisD[1]", "axisD_1", 1.0},
  {"axisD[2]", "axisD_2", 1.0},
  {"axisF[0]", "axisF_0", 1.0},
  {"axisF[1]", "axisF_1", 1.0},
  {"axisF[2]", "axisF_2", 1.0},
  {"gyroUnfilt[0]", "gyroUnfilt_0", 1.0},
  {"gyroUnfilt[1]", "gyroUnfilt_1", 1.0},
  {"gyroUnfilt[2]", "gyroUnfilt_2", 1.0},
  {"rcCommand[0]", "rcCommand_0", 1.0},
  {"rcCommand[1]", "rcCommand_1", 1.0},
  {"rcCommand[2]", "rcCommand_2", 1.0},
  {"rcCommand[3]", "rcCommand_3", 1.0},
  {"motor[0]", "motor_0", 1.0},
  {"motor[1]", "motor_1", 1.0},
  {"motor[2]", "motor_2", 1.0},
  {"motor[3]", "motor_3", 1.0},
  {"accVib", "accVib", 1.0},
  {"rssi", "rssi", 1.0},
  {"navEPH", "navEPH_cm", 1.0},
  {"navEPV", "navEPV_cm", 1.0},
  {"navTgtPos[0]", "navTgtPos_0_cm", 1.0},
  {"navTgtPos[1]", "navTgtPos_1_cm", 1.0},
  {"navTgtPos[2]", "navTgtPos_2_cm", 1.0},
  {"navTgtVel[0]", "navTgtVel_0_cm_s", 1.0},
  {"navTgtVel[1]", "navTgtVel_1_cm_s", 1.0},
  {"navTgtVel[2]", "navTgtVel_2_cm_s", 1.0},
  {"navTgtHdg", "navTgtHdg_deg", 0.01},
  {"energyCumulative", "energyCumulative_mAh", 1.0},
  {"BaroAlt", "baroAlt_m", 0.01},
  {"AirSpeed", "airSpeed_m_s", 0.01},
  {"IMUTemperature", "IMUTemperature_degC", 0.1},
  {"baroTemperature", "baroTemperature_degC", 0.1},
  {"escRPM", "escRPM", 1.0},
  {"escTemperature", "escTemperature_degC", 0.1},
  {"powerSupplyImpedance", "powerSupplyImpedance_mOhm", 1.0},
  {"sagCompensatedVBat", "sagCompensatedVBat_V", 0.001},
  {"wind[0]", "wind_0_cm_s", 1.0},
  {"wind[1]", "wind_1_cm_s", 1.0},
  {"wind[2]", "wind_2_cm_s", 1.0},
};

bool contains(const std::vector<std::string> & names, const char * wanted)
{
  return std::find(names.begin(), names.end(), wanted) != names.end();
}

}  // namespace

TopicWriter::TopicWriter(BagWriter & bag, TopicWriterOptions options)
: bag_(bag), options_(std::move(options))
{
  if (options_.decimate < 1) {
    options_.decimate = 1;
  }
}

void TopicWriter::start_log(const CsvTable & table, const InavColumns & columns, int64_t start_epoch_ns)
{
  columns_ = columns;
  start_epoch_ns_ = start_epoch_ns;
  row_counter_ = 0;
  first_time_us_ = 0;
  last_stamp_ns_ = 0;
  last_tf_ns_ = 0;
  last_plot_ns_ = 0;
  last_marker_ns_ = 0;
  last_gps_key_.clear();
  last_status_key_.clear();
  last_battery_key_.clear();
  last_rc_key_.clear();
  last_scalar_key_.clear();
  last_text_key_.clear();
  anchor_set_ = false;
  home_is_gps_home_ = false;
  gps_home_seen_ = false;
  home_in_nav_m_[0] = home_in_nav_m_[1] = 0.0;
  home_alt_m_ = home_nav_z_m_ = 0.0;
  ref_lat_deg_ = ref_lon_deg_ = 0.0;
  nav_path_ = nav_msgs::msg::Path();
  gps_path_ = nav_msgs::msg::Path();
  nav_path_.header.frame_id = options_.frame_map;
  gps_path_.header.frame_id = options_.frame_map;
  nav_path_written_ = false;
  gps_path_written_ = false;
  home_marker_written_ = false;

  if (columns_.time < 0) {
    throw std::runtime_error("the decoded CSV has no 'time' column; is the log complete?");
  }

  skipped_groups_.clear();
  if (!columns_.has_imu()) {
    skipped_groups_.emplace_back(options_.topic_prefix + "/imu (no gyroADC/accSmooth)");
  }
  if (!columns_.has_attitude()) {
    skipped_groups_.emplace_back(
      options_.topic_prefix + "/pose, /tf and the imu attitude (no attitude[] columns)");
  }
  if (!columns_.has_nav_position()) {
    skipped_groups_.emplace_back(options_.topic_prefix + "/odom, " + options_.topic_prefix + "/path/nav (no navPos[])");
  }
  if (!columns_.has_gps_fix()) {
    skipped_groups_.emplace_back(options_.topic_prefix + "/gps/fix, " + options_.topic_prefix + "/gps/vel");
  }
  if (!columns_.has_rc()) {
    skipped_groups_.emplace_back(options_.topic_prefix + "/rc, " + options_.topic_prefix + "/rc_raw (no rcData[])");
  }
  if (!columns_.has_status()) {
    skipped_groups_.emplace_back(options_.topic_prefix + "/status (no S-frame flag columns)");
  }
  if (columns_.vbat < 0) {
    skipped_groups_.emplace_back(options_.topic_prefix + "/battery (no vbat)");
  }
  if (columns_.baro_alt < 0) {
    skipped_groups_.emplace_back(options_.topic_prefix + "/baro (no BaroAlt)");
  }
  if (columns_.airspeed < 0) {
    skipped_groups_.emplace_back(options_.topic_prefix + "/airspeed (no AirSpeed)");
  }
  if (columns_.mag[0] < 0) {
    skipped_groups_.emplace_back(options_.topic_prefix + "/mag_raw (no magADC[])");
  }

  plots_.clear();
  if (options_.write_plots) {
    for (const PlotSignalDef & def : kPlotSignals) {
      const int column = table.index(def.column);
      if (column < 0) {
        continue;
      }
      PlotTopic plot;
      plot.column = column;
      plot.topic = topic("/plot/" + std::string(def.name));
      plot.scale = def.scale;
      plots_.push_back(std::move(plot));
    }
  }
}

void TopicWriter::add_row(const CsvRow & row)
{
  const int64_t time_us = static_cast<int64_t>(row.get(columns_.time));
  if (row_counter_ == 0) {
    first_time_us_ = time_us;
  }
  const int64_t stamp_ns = start_epoch_ns_ + (time_us - first_time_us_) * 1000;
  last_stamp_ns_ = stamp_ns;
  const bool keep = (row_counter_ % options_.decimate) == 0;
  ++row_counter_;
  if (!keep) {
    return;
  }

  if (options_.write_imu) {
    emit_imu(stamp_ns, row);
  }
  emit_pose_and_odom(stamp_ns, row);
  if (last_tf_ns_ == 0 || stamp_ns - last_tf_ns_ >= period_ns(options_.tf_rate)) {
    last_tf_ns_ = stamp_ns;
    emit_tf(stamp_ns, row);
  }
  emit_gps(stamp_ns, row);
  emit_status(stamp_ns, row);
  emit_battery(stamp_ns, row);
  emit_rc(stamp_ns, row);
  emit_scalars(stamp_ns, row);
  if (options_.write_plots) {
    if (last_plot_ns_ == 0 || stamp_ns - last_plot_ns_ >= period_ns(options_.plot_rate)) {
      last_plot_ns_ = stamp_ns;
      emit_plots(stamp_ns, row);
    }
  }
  emit_markers(stamp_ns, row);
  accumulate_paths(stamp_ns, row);
}

void TopicWriter::nav_position(const CsvRow & row, double out_m[3]) const
{
  if (!columns_.has_nav_position()) {
    out_m[0] = out_m[1] = out_m[2] = 0.0;
    return;
  }
  frames::nav_position_to_enu(
    row.get(columns_.nav_pos[0]), row.get(columns_.nav_pos[1]), row.get(columns_.nav_pos[2]), out_m);
  if (options_.nav_alt_from_baro && columns_.baro_alt >= 0) {
    out_m[2] = row.get(columns_.baro_alt) * frames::kCmToM;
  }
}

void TopicWriter::craft_orientation(const CsvRow & row, double out_xyzw[4]) const
{
  if (!columns_.has_attitude()) {
    out_xyzw[0] = out_xyzw[1] = out_xyzw[2] = 0.0;
    out_xyzw[3] = 1.0;
    return;
  }
  frames::attitude_to_quaternion(
    row.get(columns_.attitude[0]) * frames::kAttitudeLsb,
    row.get(columns_.attitude[1]) * frames::kAttitudeLsb,
    row.get(columns_.attitude[2]) * frames::kAttitudeLsb, out_xyzw);
}

void TopicWriter::emit_imu(int64_t stamp_ns, const CsvRow & row)
{
  if (!columns_.has_imu()) {
    return;
  }
  sensor_msgs::msg::Imu imu;
  set_stamp(imu.header.stamp, stamp_ns);
  imu.header.frame_id = options_.frame_base_link;

  double rate[3];
  frames::gyro_to_ros(row.get(columns_.gyro[0]), row.get(columns_.gyro[1]), row.get(columns_.gyro[2]), rate);
  imu.angular_velocity.x = rate[0];
  imu.angular_velocity.y = rate[1];
  imu.angular_velocity.z = rate[2];

  // accSmooth is specific force in the FLU body frame, which is already REP-103.
  imu.linear_acceleration.x = row.get(columns_.acc[0]);
  imu.linear_acceleration.y = row.get(columns_.acc[1]);
  imu.linear_acceleration.z = row.get(columns_.acc[2]);

  if (columns_.has_attitude()) {
    double xyzw[4];
    craft_orientation(row, xyzw);
    imu.orientation.x = xyzw[0];
    imu.orientation.y = xyzw[1];
    imu.orientation.z = xyzw[2];
    imu.orientation.w = xyzw[3];
  } else {
    // REP-145: -1 in the first element means "no estimate available".
    imu.orientation_covariance[0] = -1.0;
  }

  bag_.write(imu, topic("/imu"), stamp_ns);
}

void TopicWriter::emit_pose_and_odom(int64_t stamp_ns, const CsvRow & row)
{
  const bool has_position = columns_.has_nav_position();
  const bool has_attitude = columns_.has_attitude();
  if (!has_position && !has_attitude) {
    return;
  }

  double enu[3] = {0.0, 0.0, 0.0};
  double xyzw[4] = {0.0, 0.0, 0.0, 1.0};
  if (has_position) {
    nav_position(row, enu);
  }
  craft_orientation(row, xyzw);

  if (has_attitude) {
    geometry_msgs::msg::PoseStamped pose;
    set_stamp(pose.header.stamp, stamp_ns);
    pose.header.frame_id = options_.frame_map;
    pose.pose.position.x = enu[0];
    pose.pose.position.y = enu[1];
    pose.pose.position.z = enu[2];
    pose.pose.orientation.x = xyzw[0];
    pose.pose.orientation.y = xyzw[1];
    pose.pose.orientation.z = xyzw[2];
    pose.pose.orientation.w = xyzw[3];
    bag_.write(pose, topic("/pose"), stamp_ns);
  }

  if (!has_position) {
    return;
  }

  nav_msgs::msg::Odometry odom;
  set_stamp(odom.header.stamp, stamp_ns);
  odom.header.frame_id = options_.frame_map;
  odom.child_frame_id = options_.frame_base_link;
  odom.pose.pose.position.x = enu[0];
  odom.pose.pose.position.y = enu[1];
  odom.pose.pose.position.z = enu[2];
  odom.pose.pose.orientation.x = xyzw[0];
  odom.pose.pose.orientation.y = xyzw[1];
  odom.pose.pose.orientation.z = xyzw[2];
  odom.pose.pose.orientation.w = xyzw[3];

  if (columns_.has_nav_velocity()) {
    double velocity[3];
    frames::nav_velocity_to_enu(
      row.get(columns_.nav_vel[0]), row.get(columns_.nav_vel[1]), row.get(columns_.nav_vel[2]), velocity);
    odom.twist.twist.linear.x = velocity[0];
    odom.twist.twist.linear.y = velocity[1];
    odom.twist.twist.linear.z = velocity[2];
  }
  if (columns_.has_imu()) {
    double rate[3];
    frames::gyro_to_ros(row.get(columns_.gyro[0]), row.get(columns_.gyro[1]), row.get(columns_.gyro[2]), rate);
    odom.twist.twist.angular.x = rate[0];
    odom.twist.twist.angular.y = rate[1];
    odom.twist.twist.angular.z = rate[2];
  }

  // The estimator's own horizontal / vertical error estimates, cm -> m^2.
  if (columns_.nav_eph >= 0) {
    const double sigma = row.get(columns_.nav_eph) * frames::kCmToM;
    odom.pose.covariance[0] = sigma * sigma;
    odom.pose.covariance[7] = sigma * sigma;
  }
  if (columns_.nav_epv >= 0) {
    const double sigma = row.get(columns_.nav_epv) * frames::kCmToM;
    odom.pose.covariance[14] = sigma * sigma;
  }

  bag_.write(odom, topic("/odom"), stamp_ns);
}

void TopicWriter::emit_tf(int64_t stamp_ns, const CsvRow & row)
{
  if (!columns_.has_attitude()) {
    return;
  }
  double enu[3] = {0.0, 0.0, 0.0};
  double xyzw[4];
  if (columns_.has_nav_position()) {
    nav_position(row, enu);
  }
  craft_orientation(row, xyzw);

  geometry_msgs::msg::TransformStamped transform;
  set_stamp(transform.header.stamp, stamp_ns);
  transform.header.frame_id = options_.frame_map;
  transform.child_frame_id = options_.frame_base_link;
  transform.transform.translation.x = enu[0];
  transform.transform.translation.y = enu[1];
  transform.transform.translation.z = enu[2];
  transform.transform.rotation.x = xyzw[0];
  transform.transform.rotation.y = xyzw[1];
  transform.transform.rotation.z = xyzw[2];
  transform.transform.rotation.w = xyzw[3];

  tf2_msgs::msg::TFMessage message;
  message.transforms.push_back(transform);
  bag_.write(message, "/tf", stamp_ns);
}

void TopicWriter::emit_gps(int64_t stamp_ns, const CsvRow & row)
{
  if (!columns_.has_gps_fix()) {
    return;
  }
  const double lat = row.get(columns_.gps_lat);
  const double lon = row.get(columns_.gps_lon);
  if (lat == 0.0 && lon == 0.0) {
    return;
  }
  if (!key_changed(last_gps_key_, row,
                   {columns_.gps_lat, columns_.gps_lon, columns_.gps_altitude, columns_.gps_fix_type,
                    columns_.gps_num_sat, columns_.gps_velned[0]})) {
    return;
  }

  const int fix_type = static_cast<int>(row.get(columns_.gps_fix_type));
  update_gps_anchor(row, lat, lon, fix_type);

  sensor_msgs::msg::NavSatFix fix;
  set_stamp(fix.header.stamp, stamp_ns);
  fix.header.frame_id = "gps";
  fix.latitude = lat;
  fix.longitude = lon;
  fix.altitude = row.get(columns_.gps_altitude) * frames::kCmToM;
  fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
  // INAV GPS_fixType: 0 = no GPS, 1 = no fix, 2 = 2D fix, 3 = 3D fix.
  fix.status.status =
    fix_type >= 2 ? sensor_msgs::msg::NavSatStatus::STATUS_FIX : sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  if (columns_.gps_eph >= 0 && columns_.gps_epv >= 0) {
    const double horizontal = row.get(columns_.gps_eph) * frames::kCmToM;
    const double vertical = row.get(columns_.gps_epv) * frames::kCmToM;
    fix.position_covariance[0] = horizontal * horizontal;
    fix.position_covariance[4] = horizontal * horizontal;
    fix.position_covariance[8] = vertical * vertical;
    fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  }
  bag_.write(fix, topic("/gps/fix"), stamp_ns);

  if (columns_.has_gps_velocity()) {
    double velocity[3];
    frames::ned_velocity_to_enu(
      row.get(columns_.gps_velned[0]), row.get(columns_.gps_velned[1]), row.get(columns_.gps_velned[2]), velocity);
    geometry_msgs::msg::TwistStamped twist;
    set_stamp(twist.header.stamp, stamp_ns);
    twist.header.frame_id = options_.frame_map;
    twist.twist.linear.x = velocity[0];
    twist.twist.linear.y = velocity[1];
    twist.twist.linear.z = velocity[2];
    bag_.write(twist, topic("/gps/vel"), stamp_ns);
  }
}

void TopicWriter::update_gps_anchor(const CsvRow & row, double lat, double lon, int fix_type)
{
  if (anchor_set_ || fix_type < 2 || !columns_.has_nav_position()) {
    return;
  }
  if (!gps_home_seen_) {
    const double home_lat = row.get(columns_.gps_home_lat);
    const double home_lon = row.get(columns_.gps_home_lon);
    if (home_lat != 0.0 || home_lon != 0.0) {
      ref_lat_deg_ = home_lat;
      ref_lon_deg_ = home_lon;
      home_is_gps_home_ = true;
    } else {
      // No home recorded: use the first fix as the reference point instead.
      ref_lat_deg_ = lat;
      ref_lon_deg_ = lon;
    }
    gps_home_seen_ = true;
  }

  double relative[2];
  frames::geodetic_to_local_enu(lat, lon, ref_lat_deg_, ref_lon_deg_, relative);
  double enu[3];
  nav_position(row, enu);
  home_in_nav_m_[0] = enu[0] - relative[0];
  home_in_nav_m_[1] = enu[1] - relative[1];
  home_alt_m_ = row.get(columns_.gps_altitude) * frames::kCmToM;
  home_nav_z_m_ = enu[2];
  anchor_set_ = true;
}

void TopicWriter::emit_status(int64_t stamp_ns, const CsvRow & row)
{
  if (!columns_.has_status()) {
    return;
  }
  if (!key_changed(last_status_key_, row,
                   {columns_.flight_mode_flags, columns_.active_flight_mode_flags, columns_.state_flags,
                    columns_.failsafe_phase, columns_.nav_state, columns_.nav_flags, columns_.active_wp_number,
                    columns_.rx_signal_received, columns_.rx_flight_channels_valid, columns_.hw_health_status,
                    columns_.msp_override_flags})) {
    return;
  }

  inav_blackbox_msgs::msg::InavStatus status;
  set_stamp(status.header.stamp, stamp_ns);
  status.header.frame_id = options_.frame_base_link;

  status.flight_mode_names = split_flag_names(row.text(columns_.flight_mode_flags));
  status.armed = contains(status.flight_mode_names, "ARM");
  if (columns_.active_flight_mode_flags >= 0) {
    // Unlike the other flag columns the decoder emits this one as a bare bitmask.
    status.active_flight_mode_flags = static_cast<uint32_t>(std::max(0.0, row.get(columns_.active_flight_mode_flags)));
  }
  status.state_flags_names = split_flag_names(row.text(columns_.state_flags));
  status.failsafe_phase = row.text(columns_.failsafe_phase);

  if (columns_.nav_state >= 0) {
    const int nav_state = static_cast<int>(row.get(columns_.nav_state));
    status.nav_state = static_cast<uint8_t>(std::clamp(nav_state, 0, 255));
    status.nav_state_name = frames::nav_state_name(status.nav_state);
  }
  if (columns_.nav_flags >= 0) {
    status.nav_flags = static_cast<uint16_t>(std::max(0.0, row.get(columns_.nav_flags)));
  }
  if (columns_.nav_eph >= 0) {
    status.nav_eph = static_cast<float>(row.get(columns_.nav_eph));
  }
  if (columns_.nav_epv >= 0) {
    status.nav_epv = static_cast<float>(row.get(columns_.nav_epv));
  }
  if (columns_.active_wp_number >= 0) {
    status.active_wp_number = static_cast<uint8_t>(std::max(0.0, row.get(columns_.active_wp_number)));
  }

  status.gps_fix_type = static_cast<uint8_t>(std::max(0.0, row.get(columns_.gps_fix_type)));
  status.gps_num_sat = static_cast<uint8_t>(std::max(0.0, row.get(columns_.gps_num_sat)));

  status.rx_signal_received = row.get(columns_.rx_signal_received) != 0.0;
  status.rx_flight_channels_valid = row.get(columns_.rx_flight_channels_valid) != 0.0;
  status.rx_update_rate = static_cast<float>(row.get(columns_.rx_update_rate));

  status.hw_health_status = static_cast<uint32_t>(std::max(0.0, row.get(columns_.hw_health_status)));
  for (int i = 0; i < 3; ++i) {
    status.wind[static_cast<size_t>(i)] = static_cast<float>(row.get(columns_.wind[i]));
  }
  status.imu_temperature = static_cast<float>(row.get(columns_.imu_temperature) * 0.1);
  status.baro_temperature = static_cast<float>(row.get(columns_.baro_temperature) * 0.1);
  status.esc_rpm = static_cast<float>(row.get(columns_.esc_rpm));
  status.esc_temperature = static_cast<float>(row.get(columns_.esc_temperature) * 0.1);
  status.power_supply_impedance = static_cast<float>(row.get(columns_.power_supply_impedance));
  // The decoder leaves this column in millivolts even with --unit-vbat V.
  status.sag_compensated_vbat = static_cast<float>(row.get(columns_.sag_compensated_vbat) * 0.001);

  bag_.write(status, topic("/status"), stamp_ns);

  // A floating label with the current modes, so the 3D view says what is going
  // on without a separate panel.
  if (key_changed(last_text_key_, row,
                  {columns_.flight_mode_flags, columns_.failsafe_phase, columns_.nav_state})) {
    publish_mode_text(stamp_ns, row);
  }
}

void TopicWriter::publish_mode_text(int64_t stamp_ns, const CsvRow & row)
{
  double enu[3] = {0.0, 0.0, 0.0};
  if (columns_.has_nav_position()) {
    nav_position(row, enu);
  }
  const std::string nav_state =
    frames::nav_state_name(static_cast<uint8_t>(std::max(0.0, row.get(columns_.nav_state))));

  visualization_msgs::msg::Marker marker;
  set_stamp(marker.header.stamp, stamp_ns);
  marker.header.frame_id = options_.frame_map;
  marker.ns = "inav_status_text";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = enu[0];
  marker.pose.position.y = enu[1];
  marker.pose.position.z = enu[2] + 1.5;
  marker.pose.orientation.w = 1.0;
  marker.scale.z = 0.5;
  marker.color.r = 1.0f;
  marker.color.g = 1.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0f;
  marker.text = row.text(columns_.flight_mode_flags) + "\n" + row.text(columns_.failsafe_phase) + "\n" + nav_state;

  visualization_msgs::msg::MarkerArray array;
  array.markers.push_back(marker);
  bag_.write(array, topic("/markers"), stamp_ns);
}

void TopicWriter::emit_battery(int64_t stamp_ns, const CsvRow & row)
{
  if (columns_.vbat < 0) {
    return;
  }
  if (!key_changed(last_battery_key_, row,
                   {columns_.vbat, columns_.amperage, columns_.energy_cumulative, columns_.sag_compensated_vbat,
                    columns_.power_supply_impedance})) {
    return;
  }

  const double voltage = row.get(columns_.vbat);
  const double current = row.get(columns_.amperage);

  sensor_msgs::msg::BatteryState battery;
  set_stamp(battery.header.stamp, stamp_ns);
  battery.header.frame_id = options_.frame_base_link;
  battery.voltage = static_cast<float>(voltage);
  battery.current = static_cast<float>(current);
  battery.charge = static_cast<float>(row.get(columns_.energy_cumulative) / 1000.0);
  battery.power_supply_status = current > 0.05 ? sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING
                                                : sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  battery.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  battery.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
  battery.present = true;

  if (voltage > 1.0) {
    int cells = options_.battery_cells;
    if (cells <= 0) {
      // Assume the usual 3.7 V nominal cell when the caller did not say.
      cells = std::max(1, static_cast<int>(std::lround(voltage / 3.7)));
    }
    const float per_cell = static_cast<float>(voltage / cells);
    battery.cell_voltage.push_back(per_cell);
    // Rough LiPo state of charge between the usual empty and full cell voltages.
    battery.percentage = std::clamp((per_cell - 3.3f) / (4.2f - 3.3f), 0.0f, 1.0f);
  }

  bag_.write(battery, topic("/battery"), stamp_ns);
}

void TopicWriter::emit_rc(int64_t stamp_ns, const CsvRow & row)
{
  if (!columns_.has_rc()) {
    return;
  }
  if (!key_changed(last_rc_key_, row,
                   {columns_.rc_data[0], columns_.rc_data[1], columns_.rc_data[2], columns_.rc_data[3]})) {
    return;
  }

  const double roll = row.get(columns_.rc_data[0], 1500.0);
  const double pitch = row.get(columns_.rc_data[1], 1500.0);
  const double yaw = row.get(columns_.rc_data[2], 1500.0);
  const double throttle = row.get(columns_.rc_data[3], 1000.0);

  sensor_msgs::msg::Joy joy;
  set_stamp(joy.header.stamp, stamp_ns);
  joy.header.frame_id = options_.frame_base_link;
  // axes: [roll, pitch, yaw, throttle], each normalised to [-1, 1].
  joy.axes = {
    static_cast<float>(std::clamp((roll - 1500.0) / 500.0, -1.0, 1.0)),
    static_cast<float>(std::clamp((pitch - 1500.0) / 500.0, -1.0, 1.0)),
    static_cast<float>(std::clamp((yaw - 1500.0) / 500.0, -1.0, 1.0)),
    static_cast<float>(std::clamp((throttle - 1000.0) / 1000.0, 0.0, 1.0)),
  };
  // buttons: [armed, receiver signal received]
  joy.buttons = {
    static_cast<int32_t>(contains(split_flag_names(row.text(columns_.flight_mode_flags)), "ARM")),
    static_cast<int32_t>(row.get(columns_.rx_signal_received) != 0.0),
  };
  bag_.write(joy, topic("/rc"), stamp_ns);

  std_msgs::msg::UInt16MultiArray raw;
  raw.data.resize(4);
  for (int i = 0; i < 4; ++i) {
    raw.data[static_cast<size_t>(i)] = static_cast<uint16_t>(std::max(0.0, row.get(columns_.rc_data[i])));
  }
  bag_.write(raw, topic("/rc_raw"), stamp_ns);
}

void TopicWriter::emit_scalars(int64_t stamp_ns, const CsvRow & row)
{
  if (!key_changed(last_scalar_key_, row,
                   {columns_.baro_alt, columns_.airspeed, columns_.mag[0], columns_.mag[1], columns_.mag[2]})) {
    return;
  }
  if (columns_.baro_alt >= 0) {
    std_msgs::msg::Float32 message;
    message.data = static_cast<float>(row.get(columns_.baro_alt) * frames::kCmToM);
    bag_.write(message, topic("/baro"), stamp_ns);
  }
  if (columns_.airspeed >= 0) {
    std_msgs::msg::Float32 message;
    // INAV logs airspeed in cm/s and blackbox_decode passes it through as is.
    message.data = static_cast<float>(row.get(columns_.airspeed) * frames::kCmToM);
    bag_.write(message, topic("/airspeed"), stamp_ns);
  }
  if (columns_.mag[0] >= 0) {
    std_msgs::msg::Float64MultiArray message;
    message.data.resize(3);
    for (int i = 0; i < 3; ++i) {
      const double raw = row.get(columns_.mag[i]);
      // magADC is uncalibrated in the log, so the raw count is the honest value;
      // --mag-scale only rescales it for convenience.
      message.data[static_cast<size_t>(i)] = options_.mag_scale > 0.0 ? raw * options_.mag_scale : raw;
    }
    bag_.write(message, topic("/mag_raw"), stamp_ns);
  }
}

void TopicWriter::emit_plots(int64_t stamp_ns, const CsvRow & row)
{
  for (const PlotTopic & plot : plots_) {
    std_msgs::msg::Float64 message;
    message.data = row.get(plot.column) * plot.scale;
    bag_.write(message, plot.topic, stamp_ns);
  }
}

void TopicWriter::emit_markers(int64_t stamp_ns, const CsvRow & row)
{
  if (!home_marker_written_) {
    // navPos is measured from the navigation origin, which is also what INAV
    // arms from, so the origin is where the home marker belongs.
    visualization_msgs::msg::Marker marker;
    set_stamp(marker.header.stamp, stamp_ns);
    marker.header.frame_id = options_.frame_map;
    marker.ns = "inav_home";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.5;
    marker.color.r = 1.0f;
    marker.color.g = 0.2f;
    marker.color.b = 0.2f;
    marker.color.a = 1.0f;
    marker.text = "nav origin";
    visualization_msgs::msg::MarkerArray array;
    array.markers.push_back(marker);
    bag_.write(array, topic("/markers"), stamp_ns);
    home_marker_written_ = true;
  }

  if (anchor_set_ && home_is_gps_home_ && !gps_home_marker_written_) {
    visualization_msgs::msg::Marker marker;
    set_stamp(marker.header.stamp, stamp_ns);
    marker.header.frame_id = options_.frame_map;
    marker.ns = "inav_gps_home";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = home_in_nav_m_[0];
    marker.pose.position.y = home_in_nav_m_[1];
    marker.pose.position.z = home_nav_z_m_;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = 0.3;
    marker.scale.z = 2.0;
    marker.color.r = 0.2f;
    marker.color.g = 1.0f;
    marker.color.b = 0.2f;
    marker.color.a = 0.8f;
    marker.text = "GPS home";
    visualization_msgs::msg::MarkerArray array;
    array.markers.push_back(marker);
    bag_.write(array, topic("/markers"), stamp_ns);
    gps_home_marker_written_ = true;
  }

  // The nav target only moves when the mission does, so it is rate limited
  // rather than written on every row.
  if (last_marker_ns_ == 0 || stamp_ns - last_marker_ns_ >= period_ns(options_.plot_rate)) {
    last_marker_ns_ = stamp_ns;
    emit_nav_target(stamp_ns, row);
  }
}

void TopicWriter::emit_nav_target(int64_t stamp_ns, const CsvRow & row)
{
  if (columns_.nav_tgt_pos[0] < 0) {
    return;
  }
  const double north = row.get(columns_.nav_tgt_pos[0]);
  const double east = row.get(columns_.nav_tgt_pos[1]);
  if (north == 0.0 && east == 0.0) {
    return;
  }
  // navTgtPos is read as navPos's (North, East, Up) convention; measured
  // correlations against navPos are +0.74 / +0.82 / +0.80 on a real log.
  double enu[3];
  frames::nav_position_to_enu(north, east, row.get(columns_.nav_tgt_pos[2]), enu);

  visualization_msgs::msg::Marker marker;
  set_stamp(marker.header.stamp, stamp_ns);
  marker.header.frame_id = options_.frame_map;
  marker.ns = "inav_nav_target";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = enu[0];
  marker.pose.position.y = enu[1];
  marker.pose.position.z = enu[2];
  marker.pose.orientation.w = 1.0;
  marker.scale.x = marker.scale.y = marker.scale.z = 0.4;
  marker.color.r = 1.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;
  visualization_msgs::msg::MarkerArray array;
  array.markers.push_back(marker);
  bag_.write(array, topic("/markers"), stamp_ns);
}

void TopicWriter::accumulate_paths(int64_t stamp_ns, const CsvRow & row)
{
  if (!options_.write_paths) {
    return;
  }

  if (columns_.has_nav_position()) {
    double enu[3];
    nav_position(row, enu);
    geometry_msgs::msg::PoseStamped pose;
    set_stamp(pose.header.stamp, stamp_ns);
    pose.header.frame_id = options_.frame_map;
    pose.pose.position.x = enu[0];
    pose.pose.position.y = enu[1];
    pose.pose.position.z = enu[2];
    double xyzw[4];
    craft_orientation(row, xyzw);
    pose.pose.orientation.x = xyzw[0];
    pose.pose.orientation.y = xyzw[1];
    pose.pose.orientation.z = xyzw[2];
    pose.pose.orientation.w = xyzw[3];
    if (nav_path_.poses.empty()) {
      set_stamp(nav_path_.header.stamp, stamp_ns);
    }
    nav_path_.poses.push_back(std::move(pose));
  }

  if (anchor_set_ && columns_.has_gps_fix()) {
    const double lat = row.get(columns_.gps_lat);
    const double lon = row.get(columns_.gps_lon);
    if (lat != 0.0 || lon != 0.0) {
      double relative[2];
      frames::geodetic_to_local_enu(lat, lon, ref_lat_deg_, ref_lon_deg_, relative);
      geometry_msgs::msg::PoseStamped pose;
      set_stamp(pose.header.stamp, stamp_ns);
      pose.header.frame_id = options_.frame_map;
      pose.pose.position.x = home_in_nav_m_[0] + relative[0];
      pose.pose.position.y = home_in_nav_m_[1] + relative[1];
      pose.pose.position.z = home_nav_z_m_ + (row.get(columns_.gps_altitude) * frames::kCmToM - home_alt_m_);
      pose.pose.orientation.w = 1.0;
      if (gps_path_.poses.empty()) {
        set_stamp(gps_path_.header.stamp, stamp_ns);
      }
      gps_path_.poses.push_back(std::move(pose));
    }
  }
}

void TopicWriter::finish_log()
{
  // Static transforms, so rviz2 does not have to guess where the sensors sit.
  if (columns_.has_attitude()) {
    tf2_msgs::msg::TFMessage statics;
    const char * children[] = {"imu_link", "gps_link"};
    for (const char * child : children) {
      geometry_msgs::msg::TransformStamped transform;
      set_stamp(transform.header.stamp, last_stamp_ns_);
      transform.header.frame_id = options_.frame_base_link;
      transform.child_frame_id = child;
      transform.transform.rotation.w = 1.0;
      statics.transforms.push_back(transform);
    }
    bag_.write(statics, "/tf_static", last_stamp_ns_);
  }

  if (!options_.write_paths) {
    return;
  }
  if (!nav_path_.poses.empty() && !nav_path_written_) {
    bag_.write(nav_path_, topic("/path/nav"), last_stamp_ns_);
    nav_path_written_ = true;
  }
  if (!gps_path_.poses.empty() && !gps_path_written_) {
    bag_.write(gps_path_, topic("/path/gps"), last_stamp_ns_);
    gps_path_written_ = true;
  }
}

void TopicWriter::report_skipped(std::ostream & out) const
{
  if (skipped_groups_.empty()) {
    return;
  }
  out << "  topics this log cannot feed:";
  for (const std::string & group : skipped_groups_) {
    out << "\n    - " << group;
  }
  out << "\n";
}

}  // namespace inav_blackbox_ros2
