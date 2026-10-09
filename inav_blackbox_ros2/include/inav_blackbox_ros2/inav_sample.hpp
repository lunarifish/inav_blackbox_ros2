// Copyright 2026 lunarifish
// SPDX-License-Identifier: MIT
//
// Column indices for the blackbox fields this tool consumes.
//
// blackbox_decode emits a different column set per log: an INAV 9 flight log
// carries attitude / navPos / axisRate, a Betaflight log carries gyroUnfilt /
// setpoint / baroAlt instead, and an older INAV log may carry neither. So every
// index here is -1 when the log lacks that column, and the topic that needs it
// is simply not published.

#ifndef INAV_BLACKBOX_ROS2__INAV_SAMPLE_HPP_
#define INAV_BLACKBOX_ROS2__INAV_SAMPLE_HPP_

#include <initializer_list>

#include "inav_blackbox_ros2/csv_table.hpp"

namespace inav_blackbox_ros2
{

/// Resolved column indices. -1 means "this log has no such column".
struct InavColumns
{
  int time = -1;
  int loop_iteration = -1;

  int attitude[3] = {-1, -1, -1};
  int gyro[3] = {-1, -1, -1};
  int gyro_unfilt[3] = {-1, -1, -1};
  int acc[3] = {-1, -1, -1};
  int acc_vib = -1;

  int rc_data[4] = {-1, -1, -1, -1};
  int rc_command[4] = {-1, -1, -1, -1};
  int setpoint[4] = {-1, -1, -1, -1};
  int motor[8] = {-1, -1, -1, -1, -1, -1, -1, -1};
  int axis_rate[3] = {-1, -1, -1};
  int axis_p[3] = {-1, -1, -1};
  int axis_i[3] = {-1, -1, -1};
  int axis_d[3] = {-1, -1, -1};
  int axis_f[3] = {-1, -1, -1};

  int vbat = -1;
  int amperage = -1;
  int energy_cumulative = -1;
  int sag_compensated_vbat = -1;
  int power_supply_impedance = -1;

  int mag[3] = {-1, -1, -1};
  int baro_alt = -1;
  int airspeed = -1;
  int rssi = -1;
  int imu_temperature = -1;
  int baro_temperature = -1;
  int esc_rpm = -1;
  int esc_temperature = -1;

  int nav_state = -1;
  int nav_flags = -1;
  int nav_eph = -1;
  int nav_epv = -1;
  int nav_pos[3] = {-1, -1, -1};
  int nav_vel[3] = {-1, -1, -1};
  int nav_tgt_pos[3] = {-1, -1, -1};
  int nav_tgt_vel[3] = {-1, -1, -1};
  int nav_tgt_hdg = -1;

  int gps_fix_type = -1;
  int gps_num_sat = -1;
  int gps_lat = -1;
  int gps_lon = -1;
  int gps_altitude = -1;
  int gps_speed = -1;
  int gps_ground_course = -1;
  int gps_hdop = -1;
  int gps_eph = -1;
  int gps_epv = -1;
  int gps_velned[3] = {-1, -1, -1};
  int gps_home_lat = -1;
  int gps_home_lon = -1;

  int flight_mode_flags = -1;
  int active_flight_mode_flags = -1;
  int state_flags = -1;
  int failsafe_phase = -1;
  int active_wp_number = -1;
  int rx_signal_received = -1;
  int rx_flight_channels_valid = -1;
  int rx_update_rate = -1;
  int hw_health_status = -1;
  int wind[3] = {-1, -1, -1};
  int msp_override_flags = -1;

  // Capability flags, derived from the indices above.
  bool has_attitude() const { return attitude[0] >= 0; }
  bool has_imu() const { return gyro[0] >= 0 && acc[0] >= 0; }
  bool has_nav_position() const { return nav_pos[0] >= 0 && nav_pos[1] >= 0; }
  bool has_nav_velocity() const { return nav_vel[0] >= 0 && nav_vel[1] >= 0; }
  bool has_gps_fix() const { return gps_lat >= 0 && gps_lon >= 0; }
  bool has_gps_velocity() const { return gps_velned[0] >= 0; }
  bool has_rc() const { return rc_data[0] >= 0; }
  bool has_status() const { return flight_mode_flags >= 0 || state_flags >= 0 || failsafe_phase >= 0; }
};

/// Resolves `columns` against a decoded log's header.
InavColumns resolve_columns(const CsvTable & table);

}  // namespace inav_blackbox_ros2

#endif  // INAV_BLACKBOX_ROS2__INAV_SAMPLE_HPP_
