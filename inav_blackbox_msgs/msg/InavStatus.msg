# Flight-controller status decoded from an INAV blackbox S-frame (slow frame).
#
# INAV / Betaflight already decode the flag bitfields into "|"-separated name
# strings in the log (e.g. "ARM|ANGLE|AIRMODE"), so the name arrays below carry
# human-readable modes directly. The raw bitmasks are also published so that
# nothing is lost: blackbox_decode drops the flightModeFlags2 column entirely
# and emits active_flight_mode_flags without decoding it into names.
#
# Fields are 0 / empty when the underlying log column is absent (older firmware
# or a Betaflight log), so consumers must not assume every field is populated.

std_msgs/Header header

# --- Arming and flight modes -------------------------------------------------
# True when "ARM" appears in flight_mode_names.
bool armed
# flightModeFlags, e.g. ["ARM", "ANGLE", "AIRMODE", "PREARM"].
string[] flight_mode_names
# activeFlightModeFlags as a raw bitmask (the decoder does not name this one).
uint32 active_flight_mode_flags
# stateFlags, e.g. ["GPS_FIX_HOME", "GPS_FIX", "MULTIROTOR", "ALTITUDE_CONTROL"].
string[] state_flags_names
# failsafePhase, e.g. "IDLE", "RX_LOSS_DETECTED", "LANDING".
string failsafe_phase

# --- Navigation --------------------------------------------------------------
# MW_NAV_STATE_* value and its name (e.g. 3 / "NAV_STATE_WP").
uint8 nav_state
string nav_state_name
uint16 nav_flags
# Horizontal / vertical position error estimate, in cm.
float32 nav_eph
float32 nav_epv
# Currently active waypoint, 0 when there is none.
uint8 active_wp_number

# --- GPS ---------------------------------------------------------------------
# 0 = no GPS, 1 = no fix, 2 = 2D fix, 3 = 3D fix (INAV GPS_fixType).
uint8 gps_fix_type
uint8 gps_num_sat

# --- RC link -----------------------------------------------------------------
bool rx_signal_received
bool rx_flight_channels_valid
# RC frame rate reported by the receiver, in Hz.
float32 rx_update_rate

# --- Health and environment --------------------------------------------------
uint32 hw_health_status
# Wind estimate in cm/s, in INAV's navigation frame (North, East, Down).
float32[3] wind
float32 imu_temperature
float32 baro_temperature
float32 esc_rpm
float32 esc_temperature
# Battery supply impedance in milliohms and the voltage the FC would report at
# zero current draw, in volts.
float32 power_supply_impedance
float32 sag_compensated_vbat
