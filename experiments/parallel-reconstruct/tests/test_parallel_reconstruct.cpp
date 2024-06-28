#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <coroutine>
#include <format>
#include <future>
#include <nlohmann/json.hpp>
#include <queue>
#include <span>
#include <stdexcept>

#include "JsonWriter.h"
#include "crop.h"
#include "libfork/core.hpp"
#include "libfork/schedule.hpp"
#include "reconstruct.h"

using json = nlohmann::json;

const uint EMIT_TRACE_AT = 10;

// Ref:
// https://github.com/ConorWilliams/libfork?tab=readme-ov-file#the-cactus-stack
inline constexpr auto reconstruct_parallel =
    [](auto reconstruct_parallel,
       std::span<Points> cropped_buildings) -> lf::task<std::span<Points>> {
  // Allocate space for results, outputs is a std::span<Points>
  auto [outputs] = co_await lf::co_new<Points>(cropped_buildings.size());

  for (std::size_t i = 0; i < cropped_buildings.size(); ++i) {
    co_await lf::fork(&outputs[i], reconstruct_one_building_libfork)(
        cropped_buildings[i]);
  }

  co_await lf::join;  // Wait for all tasks to complete.

  co_return outputs;
};

int main(int argc, char* argv[]) {
  auto args = std::span(argv, size_t(argc));
  uint nr_laz_files = 3;
  nr_laz_files = std::stoi(args[1]);
  uint nr_points_per_laz = 2000;
  nr_points_per_laz = std::stoi(args[2]);

  std::string jsonpattern = {
      "{\"time\": \"%Y-%m-%dT%H:%M:%S.%f%z\", \"name\": \"%n\", \"process\": "
      "%P, "
      "\"thread\": %t, "
      "\"count\": %v}"};
  spdlog::set_pattern(jsonpattern);
  spdlog::set_level(spdlog::level::trace);

  // Init loggers
  auto logs_dir = "logs/parallel_reconstruct";
  auto logger_read_pc = spdlog::basic_logger_mt(
      "read_pc", std::format("{}/log_read_pc.json", logs_dir));
  auto logger_crop = spdlog::basic_logger_mt(
      "crop", std::format("{}/log_crop.json", logs_dir));
  auto logger_reconstruct = spdlog::basic_logger_mt(
      "reconstruct", std::format("{}/log_reconstruct.json", logs_dir));
  auto logger_write = spdlog::basic_logger_mt(
      "write", std::format("{}/log_write.json", logs_dir));
  auto logger_coro = spdlog::stderr_color_mt("coro");
  //  auto logger_coro = spdlog::basic_logger_mt(
  //    "coro", std::format("{}/log_coro.json", logs_dir));

  // Crop
  std::queue<Points> queue_cropped_buildings;
  crop(nr_points_per_laz, nr_laz_files, queue_cropped_buildings);
  // hack
  std::vector<Points> vec_cropped_buildings;
  vec_cropped_buildings.reserve(queue_cropped_buildings.size());
  for (; !queue_cropped_buildings.empty(); queue_cropped_buildings.pop()) {
    vec_cropped_buildings.push_back(queue_cropped_buildings.front());
  }

  // Reconstruct
  std::queue<Points> queue_reconstructed_buildings;
  auto count_buildings = 0;
  logger_reconstruct->trace(count_buildings);

  lf::lazy_pool pool(4);  // 4 worker threads
  auto reconstructed_buildings =
      lf::sync_wait(pool, reconstruct_parallel, vec_cropped_buildings);
  logger_reconstruct->trace(reconstructed_buildings.size());

  // Write
  auto count_written = 0;
  auto json_writer = JsonWriter();
  for (auto model : reconstructed_buildings) {
    json_writer.write(model, std::format("output/parallel_reconstruct/{}.json",
                                         count_written));
    if (count_written % EMIT_TRACE_AT == 0) {
      logger_write->trace(count_written);
    }
    count_written++;
  }
  logger_write->trace(count_written);

  return 0;
}
