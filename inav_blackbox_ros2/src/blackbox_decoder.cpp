// Copyright 2026 lunarifish
// SPDX-License-Identifier: MIT

#include "inav_blackbox_ros2/blackbox_decoder.hpp"

#include <fcntl.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>

namespace inav_blackbox_ros2
{

namespace
{

constexpr size_t kHeaderBlockLimit = 512 * 1024;

/// A child process whose stdout (or stderr) is read back line by line.
class PipeSource : public LineSource
{
public:
  PipeSource(std::vector<std::string> argv, bool capture_stderr)
  {
    int fds[2];
    if (pipe(fds) != 0) {
      throw std::runtime_error(std::string("pipe() failed: ") + std::strerror(errno));
    }

    child_ = fork();
    if (child_ < 0) {
      close(fds[0]);
      close(fds[1]);
      throw std::runtime_error(std::string("fork() failed: ") + std::strerror(errno));
    }

    if (child_ == 0) {
      // Child: route the captured stream into the pipe and drop the other one.
      close(fds[0]);
      if (dup2(fds[1], capture_stderr ? STDERR_FILENO : STDOUT_FILENO) < 0) {
        _exit(127);
      }
      const int devnull = open("/dev/null", O_WRONLY);
      if (devnull >= 0) {
        dup2(devnull, capture_stderr ? STDOUT_FILENO : STDERR_FILENO);
      }
      close(fds[1]);
      std::vector<char *> raw;
      raw.reserve(argv.size() + 1);
      for (const std::string & arg : argv) {
        raw.push_back(const_cast<char *>(arg.c_str()));
      }
      raw.push_back(nullptr);
      execvp(raw[0], raw.data());
      fprintf(stderr, "inav_blackbox_ros2: cannot execute '%s': %s\n", raw[0], strerror(errno));
      _exit(127);
    }

    close(fds[1]);
    stream_ = fdopen(fds[0], "r");
    if (stream_ == nullptr) {
      close(fds[0]);
      throw std::runtime_error("fdopen() failed");
    }
  }

  ~PipeSource() override
  {
    if (stream_ != nullptr) {
      fclose(stream_);
    }
    finish();
  }

  bool getline(std::string & line) override
  {
    char * buffer = nullptr;
    size_t capacity = 0;
    const ssize_t length = ::getline(&buffer, &capacity, stream_);
    if (length < 0) {
      free(buffer);
      return false;
    }
    line.assign(buffer, static_cast<size_t>(length));
    free(buffer);
    while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) {
      line.pop_back();
    }
    return true;
  }

  int finish() override
  {
    if (reaped_) {
      return status_;
    }
    if (stream_ != nullptr) {
      fclose(stream_);
      stream_ = nullptr;
    }
    int raw_status = 0;
    while (waitpid(child_, &raw_status, 0) < 0 && errno == EINTR) {
    }
    reaped_ = true;
    status_ = WIFEXITED(raw_status) ? WEXITSTATUS(raw_status) : -1;
    return status_;
  }

private:
  pid_t child_ = -1;
  FILE * stream_ = nullptr;
  int status_ = -1;
  bool reaped_ = false;
};

/// Reads a CSV already on disk.
class FileSource : public LineSource
{
public:
  explicit FileSource(const std::string & path)
  {
    stream_ = fopen(path.c_str(), "r");
    if (stream_ == nullptr) {
      throw std::runtime_error("cannot open CSV '" + path + "': " + std::strerror(errno));
    }
  }

  ~FileSource() override
  {
    if (stream_ != nullptr) {
      fclose(stream_);
    }
  }

  bool getline(std::string & line) override
  {
    char * buffer = nullptr;
    size_t capacity = 0;
    const ssize_t length = ::getline(&buffer, &capacity, stream_);
    if (length < 0) {
      free(buffer);
      return false;
    }
    line.assign(buffer, static_cast<size_t>(length));
    free(buffer);
    while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) {
      line.pop_back();
    }
    return true;
  }

  int finish() override
  {
    if (stream_ != nullptr) {
      fclose(stream_);
      stream_ = nullptr;
    }
    return 0;
  }

private:
  FILE * stream_ = nullptr;
};

/// Runs `argv` to completion and returns its exit status plus captured stderr.
std::pair<int, std::string> run_capturing_stderr(const std::vector<std::string> & argv)
{
  PipeSource process(argv, /*capture_stderr=*/true);
  std::string output;
  std::string line;
  while (process.getline(line)) {
    output += line;
    output += '\n';
  }
  return {process.finish(), output};
}

/// Parses the "Index  Start offset  Size (bytes)" table the decoder prints when
/// it is asked for a multi-log file without --index.
std::vector<std::pair<int, std::pair<uint64_t, uint64_t>>> parse_log_table(const std::string & text)
{
  std::vector<std::pair<int, std::pair<uint64_t, uint64_t>>> rows;
  size_t position = 0;
  while (position <= text.size()) {
    const size_t end = text.find('\n', position);
    const std::string line = text.substr(position, end == std::string::npos ? std::string::npos : end - position);
    if (end == std::string::npos) {
      position = text.size() + 1;
    } else {
      position = end + 1;
    }
    int index = 0;
    unsigned long long offset = 0;
    unsigned long long size = 0;
    if (std::sscanf(line.c_str(), "%d %llu %llu", &index, &offset, &size) == 3 && index > 0) {
      rows.push_back({index, {offset, size}});
    }
  }
  return rows;
}

}  // namespace

std::string BlackboxDecoder::resolve_executable(const std::string & explicit_path)
{
  if (!explicit_path.empty()) {
    return explicit_path;
  }
  if (const char * from_env = std::getenv("INAV_BLACKBOX_DECODE")) {
    if (from_env[0] != '\0') {
      return from_env;
    }
  }
  try {
    const std::string share = ament_index_cpp::get_package_share_directory("inav_blackbox_ros2");
    const std::string candidate = share + "/bin/blackbox_decode";
    if (access(candidate.c_str(), X_OK) == 0) {
      return candidate;
    }
  } catch (const std::exception &) {
    // Not installed (e.g. running from a build tree): fall back to $PATH below.
  }
  return "blackbox_decode";
}

std::vector<std::string> BlackboxDecoder::unit_arguments()
{
  return {
    "--merge-gps",
    "--unit-frame-time", "us",
    "--unit-rotation", "deg/s",
    "--unit-acceleration", "m/s2",
    "--unit-height", "cm",
    "--unit-vbat", "V",
    "--unit-amperage", "A",
    "--unit-gps-speed", "m/s",
    "--unit-flags", "flags",
  };
}

std::vector<LogInfo> BlackboxDecoder::list_logs(const std::string & path) const
{
  std::vector<std::string> argv{executable_, "--stdout"};
  const std::vector<std::string> units = unit_arguments();
  argv.insert(argv.end(), units.begin(), units.end());
  argv.push_back(path);

  const auto [status, output] = run_capturing_stderr(argv);
  const auto rows = parse_log_table(output);
  if (rows.empty()) {
    std::string detail = output;
    while (!detail.empty() && (detail.back() == '\n' || detail.back() == ' ')) {
      detail.pop_back();
    }
    throw std::runtime_error(
      "could not list the logs in '" + path + "' (decoder exit status " + std::to_string(status) +
      ")" + (detail.empty() ? std::string() : ": " + detail));
  }

  std::vector<LogInfo> logs;
  logs.reserve(rows.size());
  for (const auto & [index, offset_and_size] : rows) {
    LogInfo info;
    info.index = index;
    info.offset = offset_and_size.first;
    info.size = offset_and_size.second;
    const std::map<std::string, std::string> header = read_log_header(path, info.offset);
    if (const auto it = header.find("Log start datetime"); it != header.end()) {
      info.start_datetime = it->second;
    }
    if (const auto it = header.find("Firmware type"); it != header.end()) {
      info.firmware_type = it->second;
    }
    if (const auto it = header.find("Firmware revision"); it != header.end()) {
      info.firmware_revision = it->second;
    }
    logs.push_back(std::move(info));
  }
  return logs;
}

std::unique_ptr<LineSource> BlackboxDecoder::decode(const std::string & path, int index) const
{
  std::vector<std::string> argv{executable_, "--index", std::to_string(index), "--stdout"};
  const std::vector<std::string> units = unit_arguments();
  argv.insert(argv.end(), units.begin(), units.end());
  argv.push_back(path);
  // stderr is inherited so the decoder's per-log statistics stay visible.
  return std::make_unique<PipeSource>(argv, /*capture_stderr=*/false);
}

std::unique_ptr<LineSource> open_csv_file(const std::string & path)
{
  return std::make_unique<FileSource>(path);
}

std::map<std::string, std::string> read_log_header(const std::string & path, uint64_t offset)
{
  std::map<std::string, std::string> header;
  FILE * file = fopen(path.c_str(), "rb");
  if (file == nullptr) {
    return header;
  }
  if (fseek(file, static_cast<long>(offset), SEEK_SET) != 0) {
    fclose(file);
    return header;
  }

  std::string block;
  block.reserve(kHeaderBlockLimit);
  char buffer[4096];
  while (block.size() < kHeaderBlockLimit) {
    const size_t read = fread(buffer, 1, sizeof(buffer), file);
    if (read == 0) {
      break;
    }
    block.append(buffer, read);
    // The header block ends as soon as the binary frame data starts, so stop as
    // soon as a line does not look like "H key:value".
    const size_t last_newline = block.rfind('\n');
    if (last_newline == std::string::npos) {
      continue;
    }
    const size_t previous_newline = last_newline == 0 ? std::string::npos : block.rfind('\n', last_newline - 1);
    const size_t line_start = previous_newline == std::string::npos ? 0 : previous_newline + 1;
    const std::string last_line = block.substr(line_start, last_newline - line_start);
    if (!last_line.empty() && last_line.compare(0, 2, "H ") != 0) {
      break;
    }
  }
  fclose(file);

  size_t position = 0;
  while (position < block.size()) {
    const size_t end = block.find('\n', position);
    if (end == std::string::npos) {
      break;
    }
    std::string line = block.substr(position, end - position);
    position = end + 1;
    while (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line.size() < 3 || line.compare(0, 2, "H ") != 0) {
      break;
    }
    const size_t colon = line.find(':', 2);
    if (colon == std::string::npos) {
      continue;
    }
    header.emplace(line.substr(2, colon - 2), line.substr(colon + 1));
  }
  return header;
}

}  // namespace inav_blackbox_ros2
