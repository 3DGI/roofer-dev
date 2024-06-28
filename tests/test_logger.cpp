#include <roofer/logger/logger.h>

#include <catch2/catch_test_macros.hpp>

TEST_CASE("logger") {
  auto& logger = roofer::logger::Logger::get_logger();
  logger.set_level(roofer::logger::LogLevel::trace);
  logger.trace("trace", 42);
  logger.debug("debug");
  logger.info("info");
  logger.error("error");
  logger.critical("critical");
}
