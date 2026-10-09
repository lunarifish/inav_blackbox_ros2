// Copyright 2026 lunarifish
// SPDX-License-Identifier: MIT
//
// Locates and drives the bundled iNavFlight/blackbox-tools decoder.
//
// Two things about that tool are worth remembering:
//   * With --stdout and an explicit --index it streams one decoded CSV for that
//     log and writes nothing to disk. Without --index it prints a table of the
//     logs to stderr and exits with 255 -- again writing nothing. Both of those
//     are what this wrapper relies on; never invoke it without --stdout, as it
//     would drop .csv/.event/.gpx files next to the input.
//   * Its CSV columns do not line up with the "H Field" lists in the log header
//     (it adds energyCumulative and drops flightModeFlags2), so callers must map
//     columns by name rather than by position.

#ifndef INAV_BLACKBOX_ROS2__BLACKBOX_DECODER_HPP_
#define INAV_BLACKBOX_ROS2__BLACKBOX_DECODER_HPP_

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace inav_blackbox_ros2
{

/// One log inside a blackbox file; a file usually holds many.
struct LogInfo
{
  int index = 1;                   ///< 1-based index accepted by --index
  uint64_t offset = 0;             ///< byte offset of the log within the file
  uint64_t size = 0;               ///< size of the log in bytes
  std::string start_datetime;      ///< "H Log start datetime", UTC ISO-8601
  std::string firmware_type;       ///< "H Firmware type", e.g. "Cleanflight"
  std::string firmware_revision;   ///< e.g. "INAV 9.1.0 (15317503) SPEEDYBEEF405AIO"

  /// True for the firmware this tool was written against. Betaflight logs
  /// decode fine but carry a different field set, so most topics stay silent.
  bool is_inav() const { return firmware_revision.find("INAV") != std::string::npos; }
};

/// A stream of decoded CSV lines, with a header line first.
class LineSource
{
public:
  virtual ~LineSource() = default;
  /// Reads the next line without its terminator; false at end of stream.
  virtual bool getline(std::string & line) = 0;
  /// Waits for the producer to finish and returns its exit status (0 is good).
  virtual int finish() = 0;
};

class BlackboxDecoder
{
public:
  explicit BlackboxDecoder(std::string executable) : executable_(std::move(executable)) {}

  /// Resolution order: an explicit path, $INAV_BLACKBOX_DECODE, the copy
  /// installed in this package's share directory, then plain "blackbox_decode"
  /// left to the OS to find on $PATH.
  static std::string resolve_executable(const std::string & explicit_path = std::string());

  const std::string & executable() const { return executable_; }

  /// Lists the logs in `path` by parsing the decoder's own table, then reads
  /// each log's ASCII header block for its start time and firmware.
  std::vector<LogInfo> list_logs(const std::string & path) const;

  /// Starts decoding log `index` of `path` to a CSV stream. The decoder's
  /// progress messages are inherited, so they show up on the terminal.
  std::unique_ptr<LineSource> decode(const std::string & path, int index) const;

  /// The --unit-* and merging flags every decode uses. Pinning the units keeps
  /// the conversions in inav_frames.hpp valid regardless of the decoder's
  /// defaults.
  static std::vector<std::string> unit_arguments();

private:
  std::string executable_;
};

/// Reads an already decoded CSV from disk, as written by --stdout redirection.
std::unique_ptr<LineSource> open_csv_file(const std::string & path);

/// Parses the ASCII "H key:value" block that starts a log at `offset`.
std::map<std::string, std::string> read_log_header(const std::string & path, uint64_t offset);

}  // namespace inav_blackbox_ros2

#endif  // INAV_BLACKBOX_ROS2__BLACKBOX_DECODER_HPP_
