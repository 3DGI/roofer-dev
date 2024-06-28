/**
 * spdlog logging backend implementation.
 * Logs messages to stdout, stderr, file.
 * The log file is a JSON file.
 */
#ifdef RF_USE_LOGGER_SPDLOG

#include <roofer/logger/logger.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <cassert>

namespace roofer::logger {
  spdlog::level::level_enum cast_level(LogLevel level) {
    switch (level) {
      case LogLevel::off:
        return spdlog::level::off;
      case LogLevel::trace:
        return spdlog::level::trace;
      case LogLevel::debug:
        return spdlog::level::debug;
      case LogLevel::info:
        return spdlog::level::info;
      case LogLevel::warning:
        return spdlog::level::warn;
      case LogLevel::error:
        return spdlog::level::err;
      case LogLevel::critical:
        return spdlog::level::critical;
    }
    return spdlog::level::off;
  }

  struct Logger::logger_impl {
    LogLevel level = LogLevel::default_level;

    std::shared_ptr<
        spdlog::sinks::ansicolor_stdout_sink<spdlog::details::console_mutex>>
        stdout_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    std::shared_ptr<
        spdlog::sinks::ansicolor_stderr_sink<spdlog::details::console_mutex>>
        stderr_sink = std::make_shared<spdlog::sinks::stderr_color_sink_mt>();
    std::shared_ptr<spdlog::sinks::basic_file_sink<std::mutex>> file_sink =
        std::make_shared<spdlog::sinks::basic_file_sink_mt>(logfile_path_,
                                                            true);

    spdlog::logger logger_stdout =
        spdlog::logger("stdout", {stdout_sink, file_sink});
    spdlog::logger logger_stderr =
        spdlog::logger("stderr", {stderr_sink, file_sink});

    logger_impl() {
      auto spdlog_level = cast_level(level);
      stdout_sink->set_level(spdlog_level);
      stderr_sink->set_level(spdlog_level);
      file_sink->set_level(spdlog_level);
      // Set up json logging in the logfile.
      file_sink->set_pattern("{\n \"log\": [");
      logger_stdout.log(spdlog_level, "");
      std::string jsonpattern = {
          R"({"time": "%Y-%m-%dT%H:%M:%S.%f%z", "name": "%n", "level": "%^%l%$", "process": %P, "thread": %t, "message": "%v"},)"};
      file_sink->set_pattern(jsonpattern);
    }

    ~logger_impl() {
      // Finalize the json logfile.
      std::string jsonlastlogpattern = {
          R"({"time": "%Y-%m-%dT%H:%M:%S.%f%z", "name": "%n", "level": "%^%l%$", "process": %P, "thread": %t, "message": "%v"})"};
      file_sink->set_pattern(jsonlastlogpattern);
      logger_stdout.log(cast_level(level), "Finished.");
      file_sink->set_pattern("]\n}");
      logger_stdout.log(cast_level(level), "");
      spdlog::drop_all();
    }

    void set_level(LogLevel new_level) {
      // We set the "level" member too, for the sake of completeness, althought
      // we only need to set the levels on the sinks for spdlog.
      level = new_level;
      auto spdlog_level = cast_level(new_level);
      stdout_sink->set_level(spdlog_level);
      stderr_sink->set_level(spdlog_level);
      file_sink->set_level(spdlog_level);
      logger_stdout.set_level(spdlog_level);
      logger_stderr.set_level(spdlog_level);
    }
  };

  void Logger::set_level(LogLevel level) {
    assert(impl_);
    impl_->set_level(level);
  }

  Logger &Logger::get_logger() {
    static Logger singleton;
    if (!singleton.impl_) {
      auto impl = std::make_shared<Logger::logger_impl>();
      singleton.impl_ = impl;
    }
    return singleton;
  }

  void Logger::log(LogLevel level, std::string_view message) {
    assert(impl_);
    switch (level) {
      case LogLevel::off:
        return;
      case LogLevel::trace:
        impl_->logger_stdout.trace(message);
        return;
      case LogLevel::debug:
        impl_->logger_stdout.debug(message);
        return;
      case LogLevel::info:
        impl_->logger_stdout.info(message);
        return;
      case LogLevel::warning:
        impl_->logger_stdout.warn(message);
        return;
      case LogLevel::error:
        impl_->logger_stderr.error(message);
        return;
      case LogLevel::critical:
        impl_->logger_stderr.critical(message);
        return;
    }
  }

}  // namespace roofer::logger
#endif
