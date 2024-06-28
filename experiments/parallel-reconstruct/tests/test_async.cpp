#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <atomic>
#include <coroutine>
#include <deque>
#include <format>
#include <span>
#include <stdexcept>
#include <thread>

#include "JsonWriter.h"
#include "crop.h"
#include "libfork/core.hpp"
#include "libfork/schedule.hpp"
#include "reconstruct.h"

const uint EMIT_TRACE_AT = 10;

// Ref:
// https://github.com/ConorWilliams/libfork?tab=readme-ov-file#the-cactus-stack
inline constexpr auto reconstruct_parallel =
    [](auto reconstruct_parallel,
       std::span<Points> cropped_buildings) -> lf::task<std::vector<Points>> {
  // Allocate space for results, outputs is a std::span<Points>
  auto [outputs] = co_await lf::co_new<Points>(cropped_buildings.size());

  for (std::size_t i = 0; i < cropped_buildings.size(); ++i) {
    co_await lf::fork(&outputs[i], reconstruct_one_building_libfork)(
        cropped_buildings[i]);
  }

  co_await lf::join;  // Wait for all tasks to complete.

  std::vector<Points> o{};
  o.assign(outputs.begin(), outputs.end());

  co_return o;
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
  auto logs_dir = "logs/async";
  auto logger_read_pc = spdlog::basic_logger_mt(
      "read_pc", std::format("{}/log_read_pc.json", logs_dir));
  auto logger_crop = spdlog::basic_logger_mt(
      "crop", std::format("{}/log_crop.json", logs_dir));
  auto logger_reconstruct = spdlog::basic_logger_mt(
      "reconstruct", std::format("{}/log_reconstruct.json", logs_dir));
  auto logger_write = spdlog::basic_logger_mt(
      "write", std::format("{}/log_write.json", logs_dir));
  auto logger_coro = spdlog::stderr_color_mt("coro");

  // -- Multithreading setup

  // This should be something like:
  // unsigned int nthreads = std::thread::hardware_concurrency();
  // lf::lazy_pool pool(nthreads - 3);
  // -3, because we need one thread for loops of crop, reconstruct, serialize
  lf::lazy_pool pool(4);  // 4 worker threads

  std::mutex cropped_mutex;  // protects the cropped queue
  std::deque<std::vector<Points>> deque_cropped_laz;
  std::condition_variable cropped_pending;

  std::mutex reconstructed_mutex;  // protects the reconstructed queue
  std::deque<std::vector<Points>> deque_reconstructed_buildings;
  std::condition_variable reconstructed_pending;

  std::atomic<bool> reconstruction_running{true};

  // end setup --

  // Yields the cropped points per laz file.
  auto cropped_generator = crop_coro_batch(nr_points_per_laz, nr_laz_files);
  std::thread cropper([&]() {
    while (!cropped_generator.mCoroHdl.done()) {
      auto cropped_per_laz = cropped_generator.mCoroHdl.promise()._valueOut;
      {
        std::scoped_lock lock{cropped_mutex};
        deque_cropped_laz.push_back(cropped_per_laz);
      }
      cropped_pending.notify_one();
      cropped_generator.mCoroHdl.resume();
    }
  });
  // When crop yields the data of one laz, it suspends, add the data to the
  // queue When crop is finished, it needs to signal somehow that it's finished
  // Reconstruct_batch is initially suspended
  // There needs to be a container/buffer between crop and reconstruct so that
  // the two can operate at different speeds.
  // When data is available in the queue, resume reconstruct_batch
  // When reconstruct_batch finished, it suspends

  // Reconstruct
  std::atomic<uint> count_buildings{0};
  logger_reconstruct->trace(count_buildings.load());
  std::thread reconstructor([&]() {
    while (!cropped_generator.mCoroHdl.done()) {
      std::unique_lock lock{cropped_mutex};
      cropped_pending.wait(lock);
      if (deque_cropped_laz.empty()) continue;
      // Temporary queue so we can quickly move off items of the producer queue
      // and process them independently.
      auto pending_cropped{std::move(deque_cropped_laz)};
      deque_cropped_laz.clear();
      lock.unlock();

      while (!pending_cropped.empty()) {
        logger_coro->debug("popped one item from deque_cropped_laz");
        auto reconstructed_one_laz =
            lf::sync_wait(pool, reconstruct_parallel, pending_cropped.front());
        {
          std::scoped_lock lock_reconstructed{reconstructed_mutex};
          deque_reconstructed_buildings.push_back(reconstructed_one_laz);
        }
        reconstructed_pending.notify_one();
        pending_cropped.pop_front();

        // just for tracing
        for (auto i = 0; i < reconstructed_one_laz.size(); i++) {
          count_buildings++;
          if (count_buildings % EMIT_TRACE_AT == 0) {
            logger_reconstruct->trace(count_buildings.load());
          }
        }
      }
    }
    reconstruction_running.store(false);
  });

  // Serialize
  std::atomic<uint> count_written{0};
  auto json_writer = JsonWriter();
  logger_write->trace(count_written.load());
  std::thread serializer([&]() {
    while (reconstruction_running.load()) {
      std::unique_lock lock{reconstructed_mutex};
      reconstructed_pending.wait(lock);
      if (deque_reconstructed_buildings.empty()) continue;
      auto pending_reconstructed{std::move(deque_reconstructed_buildings)};
      deque_reconstructed_buildings.clear();
      lock.unlock();

      while (!pending_reconstructed.empty()) {
        auto models_one_laz = pending_reconstructed.front();
        for (auto model : models_one_laz) {
          json_writer.write(
              model, std::format("output/async/{}.json", count_written.load()));
          // just tracing
          count_written++;
          if (count_written % EMIT_TRACE_AT == 0) {
            logger_write->trace(count_written.load());
          }
        }
        pending_reconstructed.pop_front();
      }
    }
  });

  cropper.join();
  reconstructor.join();
  serializer.join();

  logger_reconstruct->trace(count_buildings.load());
  logger_write->trace(count_written.load());

  return 0;
}
