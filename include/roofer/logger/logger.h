/**
 * Logger library for roofer.
 *
 * Implements a generic logger facade that can use different logging backends.
 * The logging backend is selected compile time with the USE_LOGGER_* option.
 * Currently available backends:
 *  - internal (default)
 *  - spdlog (enable with USE_LOGGER_SPDLOG)
 *
 * Each implementation is thread-safe.
 *
 * References:
 * - https://hnrck.io/post/singleton-design-pattern/
 * -
 * https://github.com/PacktPublishing/Multi-Paradigm-Programming-with-Modern-Cpp-daytime/blob/master/src/helpers/logger.h
 * */
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>

#include "fmt/format.h"

namespace roofer::logger {

  enum class LogLevel : std::uint8_t {
    off = 0,
    debug,
    info,
    default_level = info,
    warning,
    error,
    critical,
  };

  class Logger final {
   public:
    ~Logger() = default;

    // Copy is cheap, because of the shared implementation.
    Logger(const Logger &) = default;
    Logger &operator=(const Logger &) = default;

    // Move would leave the object in a default state which does not make sense
    // here.
    Logger(Logger &&) noexcept = delete;
    Logger &operator=(Logger &&) noexcept = delete;

    /**
     * @brief Set the minimum level for the logger implementation. Messages
     * with a level below will be ignored.
     */
    void set_level(LogLevel level);

    /** @brief Returns a reference to the single logger instance. */
    static Logger &get_logger();

    template <typename... Args>
    void debug(fmt::format_string<Args...> fmt, Args &&...args) {
      log(LogLevel::debug, fmt::vformat(fmt, fmt::make_format_args(args...)));
    }

    template <typename... Args>
    void info(fmt::format_string<Args...> fmt, Args &&...args) {
      log(LogLevel::info, fmt::vformat(fmt, fmt::make_format_args(args...)));
    }

    template <typename... Args>
    void warning(fmt::format_string<Args...> fmt, Args &&...args) {
      log(LogLevel::warning, fmt::vformat(fmt, fmt::make_format_args(args...)));
    }

    template <typename... Args>
    void error(fmt::format_string<Args...> fmt, Args &&...args) {
      log(LogLevel::error, fmt::vformat(fmt, fmt::make_format_args(args...)));
    }

    template <typename... Args>
    void critical(fmt::format_string<Args...> fmt, Args &&...args) {
      log(LogLevel::critical,
          fmt::vformat(fmt, fmt::make_format_args(args...)));
    }

   private:
    inline static const std::string logfile_path_{"roofer.log"};

    Logger() = default;

    struct logger_impl;
    std::shared_ptr<logger_impl> impl_;

    void log(LogLevel level, std::string_view message);
  };

}  // namespace roofer::logger
