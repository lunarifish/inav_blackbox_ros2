// Copyright 2026 lunarifish
// SPDX-License-Identifier: MIT
//
// inav_blackbox_to_bag: converts an INAV (or Betaflight) blackbox log into a
// rosbag2 MCAP file, so rviz2, Foxglove Studio and PlotJuggler can be used to
// debug the flight instead of squinting at CSV.

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rcutils/logging.h>

#include "inav_blackbox_ros2/bag_writer.hpp"
#include "inav_blackbox_ros2/blackbox_decoder.hpp"
#include "inav_blackbox_ros2/csv_table.hpp"
#include "inav_blackbox_ros2/inav_sample.hpp"
#include "inav_blackbox_ros2/topic_writer.hpp"

namespace inav_blackbox_ros2
{

namespace
{

struct Options
{
  std::string output;
  std::string csv;
  std::string start_time;
  std::string decoder;
  std::string storage = "mcap";
  int index = 1;
  int decimate = 1;
  int battery_cells = 0;
  double tf_rate = 50.0;
  double plot_rate = 50.0;
  double mag_scale = 0.0;
  bool list = false;
  bool all = false;
  bool relative = false;
  bool nav_alt_from_baro = false;
  bool write_paths = true;
  bool write_plots = true;
  bool write_imu = true;
  bool verbose = false;
};

void print_usage(std::ostream & out)
{
  out <<
    "Convert an INAV blackbox log into a rosbag2 bag (MCAP) for rviz2 / Foxglove Studio.\n"
    "\n"
    "usage: inav_blackbox_to_bag [options] <log.TXT|log.bbl>\n"
    "\n"
    "  -o, --output <dir>      Bag directory to write (default <log>.log<N>.bag)\n"
    "  -i, --index <n>         Which log inside the file to convert (default 1)\n"
    "      --all               Convert every log in the file, one bag each\n"
    "  -l, --list              List the logs in the file and exit\n"
    "      --relative          Start timestamps at 0 instead of the log's UTC time\n"
    "      --start-time <iso>  Override the log's start time (ISO-8601, e.g.\n"
    "                          2026-10-08T11:29:09.722+00:00)\n"
    "      --decimate <n>      Keep every nth main frame for the fast topics (default 1)\n"
    "      --tf-rate <hz>      Rate limit for /tf (default 50)\n"
    "      --plot-rate <hz>    Rate limit for the /plot topics (default 50)\n"
    "      --nav-alt-from-baro Use BaroAlt as the height of the nav track\n"
    "      --csv <file>        Convert an already decoded CSV instead of running the\n"
    "                          decoder (its timestamps need --start-time or --relative)\n"
    "      --decoder <path>    blackbox_decode to use (default: $INAV_BLACKBOX_DECODE,\n"
    "                          then this package's share/bin, then $PATH)\n"
    "      --mag-scale <s>     Microtesla per magADC count (default: raw counts)\n"
    "      --battery-cells <n> Cells used for the battery percentage (default: derived)\n"
    "      --no-paths          Skip the nav_msgs/Path topics\n"
    "      --no-plot           Skip the scalar plot topics\n"
    "      --no-imu            Skip sensor_msgs/Imu\n"
    "      --storage <id>      rosbag2 storage plugin (default mcap)\n"
    "  -v, --verbose           More detail, including the topic list\n"
    "  -h, --help              This text\n"
    "\n"
    "Timestamps are absolute UTC by default, so replay with:\n"
    "  ros2 bag play --clock <bag>\n"
    "  ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true\n";
}

std::string require_value(int argc, char ** argv, int & i, const char * flag)
{
  if (i + 1 >= argc) {
    throw std::runtime_error(std::string(flag) + " needs a value");
  }
  return argv[++i];
}

int to_int(const std::string & text, const char * flag)
{
  try {
    return std::stoi(text);
  } catch (const std::exception &) {
    throw std::runtime_error(std::string(flag) + " needs a whole number, got '" + text + "'");
  }
}

double to_double(const std::string & text, const char * flag)
{
  try {
    return std::stod(text);
  } catch (const std::exception &) {
    throw std::runtime_error(std::string(flag) + " needs a number, got '" + text + "'");
  }
}

/// Returns false when the caller asked for --help and nothing should be done.
bool parse_arguments(int argc, char ** argv, Options & options, std::string & input, std::ostream & out)
{
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "-h" || arg == "--help") {
      print_usage(out);
      return false;
    } else if (arg == "-o" || arg == "--output") {
      options.output = require_value(argc, argv, i, "--output");
    } else if (arg == "-i" || arg == "--index") {
      options.index = to_int(require_value(argc, argv, i, "--index"), "--index");
    } else if (arg == "--all") {
      options.all = true;
    } else if (arg == "-l" || arg == "--list") {
      options.list = true;
    } else if (arg == "--relative") {
      options.relative = true;
    } else if (arg == "--start-time") {
      options.start_time = require_value(argc, argv, i, "--start-time");
    } else if (arg == "--decimate") {
      options.decimate = to_int(require_value(argc, argv, i, "--decimate"), "--decimate");
    } else if (arg == "--tf-rate") {
      options.tf_rate = to_double(require_value(argc, argv, i, "--tf-rate"), "--tf-rate");
    } else if (arg == "--plot-rate") {
      options.plot_rate = to_double(require_value(argc, argv, i, "--plot-rate"), "--plot-rate");
    } else if (arg == "--nav-alt-from-baro") {
      options.nav_alt_from_baro = true;
    } else if (arg == "--csv") {
      options.csv = require_value(argc, argv, i, "--csv");
    } else if (arg == "--decoder") {
      options.decoder = require_value(argc, argv, i, "--decoder");
    } else if (arg == "--mag-scale") {
      options.mag_scale = to_double(require_value(argc, argv, i, "--mag-scale"), "--mag-scale");
    } else if (arg == "--battery-cells") {
      options.battery_cells = to_int(require_value(argc, argv, i, "--battery-cells"), "--battery-cells");
    } else if (arg == "--no-paths") {
      options.write_paths = false;
    } else if (arg == "--no-plot") {
      options.write_plots = false;
    } else if (arg == "--no-imu") {
      options.write_imu = false;
    } else if (arg == "--storage") {
      options.storage = require_value(argc, argv, i, "--storage");
    } else if (arg == "-v" || arg == "--verbose") {
      options.verbose = true;
    } else if (!arg.empty() && arg[0] == '-') {
      throw std::runtime_error("unknown option '" + arg + "' (try --help)");
    } else if (input.empty()) {
      input = arg;
    } else {
      throw std::runtime_error("more than one input file given ('" + input + "' and '" + arg + "')");
    }
  }

  if (input.empty() && options.csv.empty()) {
    throw std::runtime_error("no input file given (try --help)");
  }
  if (options.index < 1) {
    throw std::runtime_error("--index counts from 1");
  }
  if (options.decimate < 1) {
    throw std::runtime_error("--decimate counts from 1");
  }
  if (options.all && !options.csv.empty()) {
    throw std::runtime_error("--all and --csv cannot be combined");
  }
  return true;
}

/// Days since 1970-01-01 for a civil date (Howard Hinnant's algorithm), which
/// avoids depending on the non-standard timegm().
int64_t days_from_civil(int64_t year, unsigned month, unsigned day)
{
  year -= month <= 2;
  const int64_t era = (year >= 0 ? year : year - 399) / 400;
  const unsigned year_of_era = static_cast<unsigned>(year - era * 400);
  const unsigned day_of_year = (153 * (month + (month > 2 ? -3 : 9)) + 2) / 5 + day - 1;
  const unsigned day_of_era = year_of_era * 365 + year_of_era / 4 - year_of_era / 100 + day_of_year;
  return era * 146097 + static_cast<int64_t>(day_of_era) - 719468;
}

/// Parses the "H Log start datetime" form, e.g. 2026-10-08T11:29:09.722+00:00,
/// into nanoseconds since the Unix epoch.
bool parse_iso8601_utc(const std::string & text, int64_t & out_ns)
{
  int year = 0;
  int month = 0;
  int day = 0;
  int hour = 0;
  int minute = 0;
  int second = 0;
  int consumed = 0;
  const int fields = std::sscanf(
    text.c_str(), "%4d-%2d-%2dT%2d:%2d:%2d%n", &year, &month, &day, &hour, &minute, &second, &consumed);
  if (fields != 6 || month < 1 || month > 12 || day < 1 || day > 31) {
    return false;
  }

  std::string rest = text.substr(static_cast<size_t>(consumed));
  int64_t fraction_ns = 0;
  if (!rest.empty() && rest[0] == '.') {
    size_t i = 1;
    int digits = 0;
    while (i < rest.size() && std::isdigit(static_cast<unsigned char>(rest[i]))) {
      if (digits < 9) {
        fraction_ns = fraction_ns * 10 + (rest[i] - '0');
        ++digits;
      }
      ++i;
    }
    while (digits < 9) {
      fraction_ns *= 10;
      ++digits;
    }
    rest.erase(0, i);
  }

  // The decoder writes UTC offsets; fold them back to UTC.
  int64_t offset_seconds = 0;
  if (!rest.empty() && (rest[0] == '+' || rest[0] == '-')) {
    int offset_hour = 0;
    int offset_minute = 0;
    if (std::sscanf(rest.c_str() + 1, "%2d:%2d", &offset_hour, &offset_minute) == 2) {
      offset_seconds = (offset_hour * 3600LL + offset_minute * 60LL) * (rest[0] == '-' ? -1 : 1);
    }
  } else if (!rest.empty() && rest[0] != 'Z' && rest[0] != 'z') {
    return false;
  }

  const int64_t days = days_from_civil(year, static_cast<unsigned>(month), static_cast<unsigned>(day));
  const int64_t seconds = days * 86400 + hour * 3600 + minute * 60 + second - offset_seconds;
  out_ns = seconds * 1000000000LL + fraction_ns;
  return true;
}

std::string file_stem(const std::string & path)
{
  size_t begin = path.find_last_of('/');
  begin = begin == std::string::npos ? 0 : begin + 1;
  size_t end = path.find_last_of('.');
  if (end == std::string::npos || end < begin) {
    end = path.size();
  }
  return path.substr(begin, end - begin);
}

std::string join_path(const std::string & directory, const std::string & name)
{
  if (directory.empty() || directory.back() == '/') {
    return directory + name;
  }
  return directory + "/" + name;
}

/// The log's own clock is only trustworthy if it ever got set; a flight
/// controller without an RTC writes 0000-01-01, which would put the bag in the
/// year zero.
constexpr int64_t kEarliestSaneEpochNs = 946684800LL * 1000000000LL;  // 2000-01-01T00:00:00Z

int64_t resolve_start_epoch_ns(const Options & options, const LogInfo * info)
{
  if (options.relative) {
    return 0;
  }
  if (!options.start_time.empty()) {
    int64_t nanoseconds = 0;
    if (!parse_iso8601_utc(options.start_time, nanoseconds)) {
      throw std::runtime_error("cannot read --start-time '" + options.start_time + "' as an ISO-8601 timestamp");
    }
    return nanoseconds;
  }
  if (info != nullptr && !info->start_datetime.empty()) {
    int64_t nanoseconds = 0;
    if (!parse_iso8601_utc(info->start_datetime, nanoseconds)) {
      std::cerr << "warning: cannot read the log's start time ('" << info->start_datetime
                << "'); writing relative timestamps instead\n";
    } else if (nanoseconds < kEarliestSaneEpochNs) {
      std::cerr << "warning: the log says it started at " << info->start_datetime
                << ", i.e. its clock was never set; writing relative timestamps instead\n";
    } else {
      return nanoseconds;
    }
  }
  return 0;
}

struct Summary
{
  std::string output;
  std::string firmware;
  std::string note;
  int index = 1;
  uint64_t rows = 0;
  uint64_t messages = 0;
  int64_t duration_us = 0;
  std::vector<std::string> topics;
  std::string skipped;
};

/// Streams one decoded log into one bag. `source` is expected to sit at the CSV
/// header line.
Summary convert_stream(
  const Options & options, std::unique_ptr<LineSource> source, const std::string & output, int index,
  int64_t start_epoch_ns, const std::string & firmware)
{
  Summary summary;
  summary.output = output;
  summary.index = index;
  summary.firmware = firmware;

  std::string line;
  if (!source->getline(line)) {
    throw std::runtime_error("the decoded CSV is empty; nothing to write");
  }
  const CsvTable table(split_csv_line(line));
  if (options.verbose) {
    std::cout << "  csv columns: " << table.size() << "\n";
  }

  const InavColumns columns = resolve_columns(table);
  const int time_column = columns.time;

  BagWriter bag(output, options.storage);

  TopicWriterOptions writer_options;
  writer_options.decimate = options.decimate;
  writer_options.tf_rate = options.tf_rate;
  writer_options.plot_rate = options.plot_rate;
  writer_options.nav_alt_from_baro = options.nav_alt_from_baro;
  writer_options.write_paths = options.write_paths;
  writer_options.write_plots = options.write_plots;
  writer_options.write_imu = options.write_imu;
  writer_options.mag_scale = options.mag_scale;
  writer_options.battery_cells = options.battery_cells;

  TopicWriter writer(bag, writer_options);
  writer.start_log(table, columns, start_epoch_ns);

  CsvRow row;
  int64_t first_time_us = 0;
  int64_t last_time_us = 0;
  while (source->getline(line)) {
    if (line.empty()) {
      continue;
    }
    row.reset(split_csv_line(line));
    writer.add_row(row);
    if (time_column >= 0) {
      const int64_t time_us = static_cast<int64_t>(row.get(time_column));
      if (summary.rows == 0) {
        first_time_us = time_us;
      }
      last_time_us = time_us;
    }
    ++summary.rows;
  }
  writer.finish_log();

  const int status = source->finish();
  if (status != 0) {
    std::cerr << "warning: the decoder exited with status " << status << "; the bag may be incomplete\n";
  }

  summary.duration_us = summary.rows > 0 ? last_time_us - first_time_us : 0;
  summary.messages = bag.message_count();
  for (const auto & entry : bag.topics()) {
    summary.topics.push_back(entry.first + "  [" + entry.second + "]");
  }
  std::ostringstream skipped;
  writer.report_skipped(skipped);
  summary.skipped = skipped.str();
  return summary;
}

void report(std::ostream & out, const Summary & summary, bool verbose)
{
  out << summary.output << "\n";
  out << "  log index  : " << summary.index << "\n";
  if (!summary.firmware.empty()) {
    out << "  firmware   : " << summary.firmware << "\n";
  }
  out << "  decoded    : " << summary.rows << " rows, " << std::fixed << std::setprecision(3)
      << static_cast<double>(summary.duration_us) / 1e6 << " s\n";
  out << "  bag        : " << summary.messages << " messages on " << summary.topics.size() << " topics\n";
  if (verbose) {
    for (const std::string & topic : summary.topics) {
      out << "    - " << topic << "\n";
    }
  }
  out << summary.skipped;
  if (!summary.note.empty()) {
    out << "  note       : " << summary.note << "\n";
  }
  out.flush();
}

void report_logs(std::ostream & out, const std::string & path, const std::vector<LogInfo> & logs)
{
  out << path << ": " << logs.size() << (logs.size() == 1 ? " log\n" : " logs\n");
  for (const LogInfo & info : logs) {
    out << "  " << std::setw(3) << info.index << "  " << std::setw(12) << info.size << " bytes  "
        << (info.start_datetime.empty() ? std::string("(no start datetime)") : info.start_datetime) << "  "
        << (info.firmware_revision.empty() ? std::string("(unknown firmware)") : info.firmware_revision);
    if (!info.is_inav()) {
      out << "  [not INAV: most topics will stay empty]";
    }
    out << "\n";
  }
}

int run(const Options & options, const std::string & input, std::ostream & out, std::ostream & err)
{
  std::vector<Summary> summaries;

  if (!options.csv.empty()) {
    // A bare CSV has no log header, so the start time has to come from the
    // command line (or --relative).
    const int64_t start_ns = resolve_start_epoch_ns(options, nullptr);
    if (start_ns == 0 && !options.relative) {
      err << "note: a CSV carries no start time; writing relative timestamps (use --start-time to set one)\n";
    }
    const std::string output =
      options.output.empty() ? file_stem(options.csv) + ".log1.bag" : options.output;
    summaries.push_back(
      convert_stream(options, open_csv_file(options.csv), output, 1, start_ns, std::string()));
  } else {
    const BlackboxDecoder decoder(BlackboxDecoder::resolve_executable(options.decoder));
    if (options.verbose) {
      out << "decoder: " << decoder.executable() << "\n";
    }

    const std::vector<LogInfo> logs = decoder.list_logs(input);
    if (options.list) {
      report_logs(out, input, logs);
      return 0;
    }

    if (options.all) {
      if (options.output.empty()) {
        throw std::runtime_error("--all needs --output <directory> to put the bags in");
      }
      for (const LogInfo & info : logs) {
        const std::string output =
          join_path(options.output, file_stem(input) + ".log" + std::to_string(info.index) + ".bag");
        if (options.verbose) {
          out << "log " << info.index << " -> " << output << "\n";
        }
        summaries.push_back(convert_stream(
          options, decoder.decode(input, info.index), output, info.index,
          resolve_start_epoch_ns(options, &info), info.firmware_revision));
      }
    } else {
      const auto wanted = std::find_if(
        logs.begin(), logs.end(), [&options](const LogInfo & info) { return info.index == options.index; });
      if (wanted == logs.end()) {
        throw std::runtime_error(
          "this file has no log " + std::to_string(options.index) + " (it has " + std::to_string(logs.size()) +
          "); see --list");
      }
      const std::string output = options.output.empty()
        ? file_stem(input) + ".log" + std::to_string(wanted->index) + ".bag"
        : options.output;
      Summary summary = convert_stream(
        options, decoder.decode(input, wanted->index), output, wanted->index,
        resolve_start_epoch_ns(options, &*wanted), wanted->firmware_revision);
      if (!wanted->is_inav()) {
        summary.note =
          "this is not an INAV log, so the INAV-only topics (attitude, nav, RC, status) are absent; "
          "the vendor-neutral ones (gyro, motors, plots, battery, baro) are still there";
      }
      summaries.push_back(std::move(summary));
    }
  }

  for (const Summary & summary : summaries) {
    report(out, summary, options.verbose);
  }
  if (summaries.size() > 1) {
    out << "wrote " << summaries.size() << " bags\n";
  }
  out << "\nReplay with:\n  ros2 bag play --clock " << summaries.front().output << "\n"
      << "  ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true   # Fixed Frame: map\n";
  return 0;
}

}  // namespace

/// Entry point, kept out of the anonymous namespace only so main() can call it.
int run_main(int argc, char ** argv)
{
  // rosbag2 chatters about flushing its cache when the bag closes, which buries
  // the summary; warnings and errors still come through.
  rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_WARN);

  Options options;
  std::string input;
  try {
    if (!parse_arguments(argc, argv, options, input, std::cout)) {
      return 0;
    }
    return run(options, input, std::cout, std::cerr);
  } catch (const std::exception & error) {
    std::cerr << "inav_blackbox_to_bag: " << error.what() << "\n";
    return 1;
  }
}

}  // namespace inav_blackbox_ros2

int main(int argc, char ** argv)
{
  return inav_blackbox_ros2::run_main(argc, argv);
}
