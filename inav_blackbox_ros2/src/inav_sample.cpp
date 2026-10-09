// Copyright 2026 lunarifish
// SPDX-License-Identifier: MIT

#include "inav_blackbox_ros2/inav_sample.hpp"

#include <string>

namespace inav_blackbox_ros2
{

namespace
{

/// Index of the first of `names` that this log actually contains, else -1.
/// Several fields were renamed across firmware versions, e.g. INAV's "BaroAlt"
/// and Betaflight's "baroAlt", or "vbat" and "vbatLatest".
int first_present(const CsvTable & table, std::initializer_list<const char *> names)
{
  for (const char * name : names) {
    const int index = table.index(name);
    if (index >= 0) {
      return index;
    }
  }
  return -1;
}

/// Resolves a family of array columns: `base` names either "foo" (then the
/// columns are "foo[i]") or a single column (then `count` is 1).
void resolve_array(
  const CsvTable & table, const char * base, int * out, int count, bool indexed = true)
{
  for (int i = 0; i < count; ++i) {
    out[i] = indexed ? table.index(std::string(base) + "[" + std::to_string(i) + "]") : table.index(base);
  }
}

}  // namespace

InavColumns resolve_columns(const CsvTable & table)
{
  InavColumns c;

  c.time = first_present(table, {"time"});
  c.loop_iteration = first_present(table, {"loopIteration"});

  resolve_array(table, "attitude", c.attitude, 3);
  resolve_array(table, "gyroADC", c.gyro, 3);
  resolve_array(table, "gyroUnfilt", c.gyro_unfilt, 3);
  resolve_array(table, "accSmooth", c.acc, 3);
  c.acc_vib = first_present(table, {"accVib"});

  resolve_array(table, "rcData", c.rc_data, 4);
  resolve_array(table, "rcCommand", c.rc_command, 4);
  resolve_array(table, "setpoint", c.setpoint, 4);
  resolve_array(table, "motor", c.motor, 8);
  resolve_array(table, "axisRate", c.axis_rate, 3);
  resolve_array(table, "axisP", c.axis_p, 3);
  resolve_array(table, "axisI", c.axis_i, 3);
  resolve_array(table, "axisD", c.axis_d, 3);
  resolve_array(table, "axisF", c.axis_f, 3);

  c.vbat = first_present(table, {"vbat", "vbatLatest"});
  c.amperage = first_present(table, {"amperage", "amperageLatest"});
  c.energy_cumulative = first_present(table, {"energyCumulative"});
  c.sag_compensated_vbat = first_present(table, {"sagCompensatedVBat"});
  c.power_supply_impedance = first_present(table, {"powerSupplyImpedance"});

  resolve_array(table, "magADC", c.mag, 3);
  c.baro_alt = first_present(table, {"BaroAlt", "baroAlt"});
  c.airspeed = first_present(table, {"AirSpeed"});
  c.rssi = first_present(table, {"rssi"});
  c.imu_temperature = first_present(table, {"IMUTemperature"});
  c.baro_temperature = first_present(table, {"baroTemperature"});
  c.esc_rpm = first_present(table, {"escRPM"});
  c.esc_temperature = first_present(table, {"escTemperature"});

  c.nav_state = first_present(table, {"navState"});
  c.nav_flags = first_present(table, {"navFlags"});
  c.nav_eph = first_present(table, {"navEPH"});
  c.nav_epv = first_present(table, {"navEPV"});
  resolve_array(table, "navPos", c.nav_pos, 3);
  resolve_array(table, "navVel", c.nav_vel, 3);
  resolve_array(table, "navTgtPos", c.nav_tgt_pos, 3);
  resolve_array(table, "navTgtVel", c.nav_tgt_vel, 3);
  c.nav_tgt_hdg = first_present(table, {"navTgtHdg"});

  c.gps_fix_type = first_present(table, {"GPS_fixType"});
  c.gps_num_sat = first_present(table, {"GPS_numSat"});
  c.gps_lat = first_present(table, {"GPS_coord[0]"});
  c.gps_lon = first_present(table, {"GPS_coord[1]"});
  c.gps_altitude = first_present(table, {"GPS_altitude"});
  c.gps_speed = first_present(table, {"GPS_speed"});
  c.gps_ground_course = first_present(table, {"GPS_ground_course"});
  c.gps_hdop = first_present(table, {"GPS_hdop"});
  c.gps_eph = first_present(table, {"GPS_eph"});
  c.gps_epv = first_present(table, {"GPS_epv"});
  resolve_array(table, "GPS_velned", c.gps_velned, 3);
  c.gps_home_lat = first_present(table, {"GPS_home_lat"});
  c.gps_home_lon = first_present(table, {"GPS_home_lon"});

  c.flight_mode_flags = first_present(table, {"flightModeFlags"});
  c.active_flight_mode_flags = first_present(table, {"activeFlightModeFlags"});
  c.state_flags = first_present(table, {"stateFlags"});
  c.failsafe_phase = first_present(table, {"failsafePhase"});
  c.active_wp_number = first_present(table, {"activeWpNumber"});
  c.rx_signal_received = first_present(table, {"rxSignalReceived"});
  c.rx_flight_channels_valid = first_present(table, {"rxFlightChannelsValid"});
  c.rx_update_rate = first_present(table, {"rxUpdateRate"});
  c.hw_health_status = first_present(table, {"hwHealthStatus"});
  resolve_array(table, "wind", c.wind, 3);
  c.msp_override_flags = first_present(table, {"mspOverrideFlags"});

  return c;
}

}  // namespace inav_blackbox_ros2
