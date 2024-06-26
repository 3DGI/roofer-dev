/*
 * Copyright (c) 2024 Balázs Dukai.
 *
 * Internal logger implementation.
 * Message writer thread implementation taken from
 * https://github.com/PacktPublishing/Multi-Paradigm-Programming-with-Modern-Cpp-daytime
 */
#include "config.hpp"

#if !defined(USE_LOGGER_SPDLOG) && !defined(USE_LOGGER_RERUN)

#include <cassert>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>

#include "logger.h"

namespace {
  /** @brief Convert log level into string */
  std::string string_from_log_level(Logger::LogLevel level) {
    std::array<std::string, 6> names = {"OFF",     "DEBUG", "INFO",
                                        "WARNING", "ERROR", "CRITICAL"};
    auto ilevel = static_cast<int>(level);
    return names[ilevel];
  }

  /** @brief Get current time, for printing into the log */
  std::string get_now_string() {
    using system_clock = std::chrono::system_clock;
    auto now = system_clock::now();
    return std::format("{:%F %T}", now);
  }
}  // namespace

namespace Logger {
  struct Logger::logger_impl {
    LogLevel level = LogLevel::default_level;

    std::mutex messages_mutex;  // protects the messages queue
    std::queue<std::string> messages;

    std::thread writer_thread;
    std::atomic<bool> active = true;
    std::condition_variable message_pending;

    logger_impl()
        : writer_thread([this]() {
            while (active.load(std::memory_order_relaxed)) {
              write_pending_message();
            }
          }) {}

    ~logger_impl() {
      active.store(false, std::memory_order_relaxed);
      message_pending.notify_one();
      writer_thread.join();
    }

    void write_pending_message() {
      std::unique_lock lock{messages_mutex};
      message_pending.wait(lock);

      if (messages.empty()) return;

      // Temporary queue so that we can quickly move the pending messages off
      // main queue and release the lock on the main queue.
      std::queue<std::string> pending_messages{std::move(messages)};
      lock.unlock();

      while (!pending_messages.empty()) {
        std::string m{std::move(pending_messages.front())};
        pending_messages.pop();

        std::cout << m << '\n';
        // Writing to any other stream eg. file can happen here.
      }
    }

    void push_message(LogLevel log_level, std::string_view message) {
      std::stringstream stream;
      stream << "[" << get_now_string() << "]\t"
             << string_from_log_level(log_level) << '\t' << message;

      {
        std::scoped_lock lock{messages_mutex};
        messages.push(stream.str());
      }
      message_pending.notify_one();
    }
  };

  void Logger::set_level(LogLevel level) {
    assert(impl_);
    impl_->level = level;
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
    if (impl_->level > level) {
      return;
    }
    impl_->push_message(level, message);
  }

}  // namespace Logger
#endif
