// Copyright 2026 lunarifish
// SPDX-License-Identifier: MIT

#include "inav_blackbox_ros2/inav_frames.hpp"

#include <cmath>

namespace inav_blackbox_ros2
{
namespace frames
{

namespace
{
/// MW_NAV_STATE_* from INAV's navigation.h (navSystemStatus_State_e).
const char * const kNavStateNames[] = {
  "NONE",           "RTH_START",      "RTH_ENROUTE",       "HOLD_INFINIT",
  "HOLD_TIMED",     "WP_ENROUTE",     "PROCESS_NEXT",      "DO_JUMP",
  "LAND_START",     "LAND_IN_PROGRESS", "LANDED",          "LAND_SETTLE",
  "LAND_START_DESCENT", "HOVER_ABOVE_HOME", "EMERGENCY_LANDING", "RTH_CLIMB",
};
}  // namespace

void nav_position_to_enu(double north_cm, double east_cm, double up_cm, double out_enu_m[3])
{
  out_enu_m[0] = east_cm * kCmToM;
  out_enu_m[1] = north_cm * kCmToM;
  out_enu_m[2] = up_cm * kCmToM;
}

void nav_velocity_to_enu(double north_cms, double east_cms, double up_cms, double out_enu_ms[3])
{
  out_enu_ms[0] = east_cms * kCmToM;
  out_enu_ms[1] = north_cms * kCmToM;
  out_enu_ms[2] = up_cms * kCmToM;
}

void ned_velocity_to_enu(double north_cms, double east_cms, double down_cms, double out_enu_ms[3])
{
  out_enu_ms[0] = east_cms * kCmToM;
  out_enu_ms[1] = north_cms * kCmToM;
  out_enu_ms[2] = -down_cms * kCmToM;
}

void attitude_to_quaternion(double roll_deg, double pitch_deg, double heading_deg, double out_xyzw[4])
{
  const double roll = roll_deg * kDegToRad;
  const double pitch = -pitch_deg * kDegToRad;
  const double yaw = (90.0 - heading_deg) * kDegToRad;

  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);

  // Z-Y-X intrinsic, i.e. R = Rz(yaw) * Ry(pitch) * Rx(roll).
  out_xyzw[0] = sr * cp * cy - cr * sp * sy;
  out_xyzw[1] = cr * sp * cy + sr * cp * sy;
  out_xyzw[2] = cr * cp * sy - sr * sp * cy;
  out_xyzw[3] = cr * cp * cy + sr * sp * sy;
}

void gyro_to_ros(double gx_dps, double gy_dps, double gz_dps, double out_rad_s[3])
{
  out_rad_s[0] = gx_dps * kDegPerSecToRad;
  out_rad_s[1] = -gy_dps * kDegPerSecToRad;
  out_rad_s[2] = gz_dps * kDegPerSecToRad;
}

void geodetic_to_local_enu(
  double lat_deg, double lon_deg, double lat_ref_deg, double lon_ref_deg, double out_en_m[2])
{
  const double lat_rad = lat_ref_deg * kDegToRad;
  out_en_m[0] = (lon_deg - lon_ref_deg) * kDegToRad * kEarthRadius * std::cos(lat_rad);
  out_en_m[1] = (lat_deg - lat_ref_deg) * kDegToRad * kEarthRadius;
}

std::string nav_state_name(uint8_t nav_state)
{
  const size_t count = sizeof(kNavStateNames) / sizeof(kNavStateNames[0]);
  if (nav_state >= count) {
    return "NAV_STATE_UNKNOWN";
  }
  return std::string("NAV_STATE_") + kNavStateNames[nav_state];
}

}  // namespace frames
}  // namespace inav_blackbox_ros2
