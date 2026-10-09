// Copyright 2026 lunarifish
// SPDX-License-Identifier: MIT
//
// INAV -> ROS (REP-103 / REP-105) frame and unit conversions.
//
// Every sign and unit below was established from the INAV 9.1 sources plus
// measurements on real logs; tools/check_conventions.py reproduces those
// measurements against any blackbox file. The evidence, in short:
//
//  * The logged IMU frame is FLU: x forward, y left, z up. accSmooth[2] reads
//    +9.8 m/s^2 at rest, corr(accSmooth[1], sin(roll)) > 0 and
//    corr(accSmooth[0], sin(pitch)) < 0. FLU is already REP-103, so linear
//    acceleration passes through with no sign change.
//
//  * INAV "yaw" is a 0..360 degree clockwise compass heading, while ROS yaw is
//    a counter-clockwise angle from East, hence yaw_ros = 90 deg - heading.
//    INAV pitch is positive nose-up, ROS pitch is positive nose-down about +y,
//    so it is negated. Roll needs no sign change.
//
//  * Because of that heading convention, body rates map to ROS as
//    (+gx, -gy, +gz). The yaw sign follows d(yaw_ros)/dt = -d(heading)/dt, which
//    matches corr(gyroADC[2], d(attitude[2])/dt) < 0 measured on real logs.
//
//  * Both the navigation position and the navigation velocity are
//    (North, East, Up) in cm and cm/s. Three measurements agree: the vertical
//    velocity matches GPS_velned[2] with a negative correlation (-0.85 on real
//    logs, and that column is NED), navPos[2] tracks the barometric altitude
//    (82-100% of the >1 m altitude changes move the same way, slope ~+0.5), and
//    d(navPos[2])/dt correlates *positively* with navVel[2] exactly like the
//    north and east axes do. So the Down sign belongs to GPS_velned[2] only.

#ifndef INAV_BLACKBOX_ROS2__INAV_FRAMES_HPP_
#define INAV_BLACKBOX_ROS2__INAV_FRAMES_HPP_

#include <cstdint>
#include <string>

namespace inav_blackbox_ros2
{
namespace frames
{

/// Attitude is logged in 0.1 degree units.
constexpr double kAttitudeLsb = 0.1;
/// Heights (BaroAlt, GPS_altitude) are logged in cm with --unit-height cm.
constexpr double kCmToM = 0.01;
constexpr double kDegToRad = 0.017453292519943295;
constexpr double kDegPerSecToRad = kDegToRad;
/// Mean Earth radius used for the local tangential projection.
constexpr double kEarthRadius = 6371000.0;

/// navPos is (North, East, Up) in cm relative to the navigation origin.
/// Writes {east, north, up} in metres.
void nav_position_to_enu(double north_cm, double east_cm, double up_cm, double out_enu_m[3]);

/// navVel is (North, East, Up) in cm/s, the same convention as navPos.
/// Writes {east, north, up} in m/s.
void nav_velocity_to_enu(double north_cms, double east_cms, double up_cms, double out_enu_ms[3]);

/// GPS_velned is (North, East, Down) in cm/s. Writes {east, north, up} in m/s.
void ned_velocity_to_enu(double north_cms, double east_cms, double down_cms, double out_enu_ms[3]);

/// Roll / pitch applied as-is and heading mapped to ROS yaw. All angles in
/// degrees; writes a unit quaternion as {x, y, z, w}.
void attitude_to_quaternion(double roll_deg, double pitch_deg, double heading_deg, double out_xyzw[4]);

/// Body rates in deg/s from the logged FLU frame to ROS rad/s.
void gyro_to_ros(double gx_dps, double gy_dps, double gz_dps, double out_rad_s[3]);

/// WGS-84 coordinates to a local east-north-up offset in metres from the
/// reference point (equirectangular projection, adequate over a few km).
void geodetic_to_local_enu(
  double lat_deg, double lon_deg, double lat_ref_deg, double lon_ref_deg, double out_en_m[2]);

/// INAV navState (MW_NAV_STATE_*) as a human readable name, or "NAV_STATE_UNKNOWN".
std::string nav_state_name(uint8_t nav_state);

}  // namespace frames
}  // namespace inav_blackbox_ros2

#endif  // INAV_BLACKBOX_ROS2__INAV_FRAMES_HPP_
