// Copyright 2026 lunarifish
// SPDX-License-Identifier: MIT
//
// Helpers for the CSV that blackbox_decode writes with --stdout.

#ifndef INAV_BLACKBOX_ROS2__CSV_TABLE_HPP_
#define INAV_BLACKBOX_ROS2__CSV_TABLE_HPP_

#include <string>
#include <unordered_map>
#include <vector>

namespace inav_blackbox_ros2
{

/// Strips the " (unit)" suffix blackbox_decode appends to a column name when the
/// matching --unit-* flag is given: "gyroADC[0] (deg/s)" -> "gyroADC[0]",
/// "BaroAlt (cm)" -> "BaroAlt", "vbat (V)" -> "vbat".
std::string normalize_column_name(const std::string & column);

/// Splits one CSV line on commas. blackbox_decode never quotes or escapes.
std::vector<std::string> split_csv_line(const std::string & line);

/// Splits a decoded flag column such as "ARM|ANGLE|AIRMODE" into its names.
/// An empty input yields an empty vector.
std::vector<std::string> split_flag_names(const std::string & text);

/// True when `cell` looks like a plain number rather than a flag-name list.
/// blackbox_decode writes decoded flags as names and undecoded ones as numbers.
bool looks_numeric(const std::string & cell);

/// Name -> column index lookup for the header line of one decoded log.
class CsvTable
{
public:
  explicit CsvTable(const std::vector<std::string> & header);

  /// Column index of `name`, or -1 when this log has no such column. The search
  /// is by normalised name, so callers pass "gyroADC[0]" and "BaroAlt".
  int index(const std::string & name) const;
  bool has(const std::string & name) const { return index(name) >= 0; }
  int size() const { return static_cast<int>(header_.size()); }
  const std::vector<std::string> & header() const { return header_; }

private:
  std::vector<std::string> header_;
  std::unordered_map<std::string, int> index_by_name_;
};

/// One decoded CSV data row. Absent or empty cells read as the fallback.
class CsvRow
{
public:
  CsvRow() = default;
  explicit CsvRow(std::vector<std::string> fields) : fields_(std::move(fields)) {}
  void reset(std::vector<std::string> fields) { fields_ = std::move(fields); }

  size_t size() const { return fields_.size(); }
  double get(int column, double fallback = 0.0) const;
  /// Raw text of a cell, or an empty string when the column is absent.
  const std::string & text(int column) const;

private:
  std::vector<std::string> fields_;
};

}  // namespace inav_blackbox_ros2

#endif  // INAV_BLACKBOX_ROS2__CSV_TABLE_HPP_
