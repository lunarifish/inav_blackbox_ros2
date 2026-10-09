// Copyright 2026 lunarifish
// SPDX-License-Identifier: MIT

#include "inav_blackbox_ros2/csv_table.hpp"

#include <cstdlib>
#include <cstring>

namespace inav_blackbox_ros2
{

namespace
{
const std::string kEmpty;
}  // namespace

std::string normalize_column_name(const std::string & column)
{
  // Only a trailing " (...)" is a unit suffix; the '[i]' of array columns stays.
  if (column.empty() || column.back() != ')') {
    return column;
  }
  const size_t open = column.rfind(" (");
  if (open == std::string::npos) {
    return column;
  }
  return column.substr(0, open);
}

std::vector<std::string> split_csv_line(const std::string & line)
{
  std::vector<std::string> fields;
  size_t start = 0;
  while (true) {
    const size_t comma = line.find(',', start);
    if (comma == std::string::npos) {
      fields.push_back(line.substr(start));
      break;
    }
    fields.push_back(line.substr(start, comma - start));
    start = comma + 1;
  }
  // A trailing newline / carriage return belongs to no field.
  if (!fields.empty()) {
    std::string & last = fields.back();
    while (!last.empty() && (last.back() == '\n' || last.back() == '\r')) {
      last.pop_back();
    }
  }
  return fields;
}

std::vector<std::string> split_flag_names(const std::string & text)
{
  std::vector<std::string> names;
  if (text.empty()) {
    return names;
  }
  size_t start = 0;
  while (true) {
    const size_t bar = text.find('|', start);
    const std::string name = text.substr(start, bar == std::string::npos ? std::string::npos : bar - start);
    if (!name.empty()) {
      names.push_back(name);
    }
    if (bar == std::string::npos) {
      break;
    }
    start = bar + 1;
  }
  return names;
}

bool looks_numeric(const std::string & cell)
{
  if (cell.empty()) {
    return false;
  }
  char * end = nullptr;
  std::strtod(cell.c_str(), &end);
  return end != nullptr && *end == '\0';
}

CsvTable::CsvTable(const std::vector<std::string> & header)
{
  header_.reserve(header.size());
  for (size_t i = 0; i < header.size(); ++i) {
    header_.push_back(normalize_column_name(header[i]));
    // Columns that differ only by unit collide; the first one wins.
    index_by_name_.emplace(header_.back(), static_cast<int>(i));
  }
}

int CsvTable::index(const std::string & name) const
{
  const auto it = index_by_name_.find(name);
  return it == index_by_name_.end() ? -1 : it->second;
}

double CsvRow::get(int column, double fallback) const
{
  if (column < 0 || static_cast<size_t>(column) >= fields_.size()) {
    return fallback;
  }
  const std::string & cell = fields_[static_cast<size_t>(column)];
  if (cell.empty()) {
    return fallback;
  }
  char * end = nullptr;
  const double value = std::strtod(cell.c_str(), &end);
  if (end == cell.c_str() || (end != nullptr && *end != '\0')) {
    return fallback;
  }
  return value;
}

const std::string & CsvRow::text(int column) const
{
  if (column < 0 || static_cast<size_t>(column) >= fields_.size()) {
    return kEmpty;
  }
  return fields_[static_cast<size_t>(column)];
}

}  // namespace inav_blackbox_ros2
