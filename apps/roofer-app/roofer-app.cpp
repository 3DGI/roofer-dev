#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <filesystem>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
namespace fs = std::filesystem;

// common
#include <roofer/logger/logger.h>

#include <roofer/common/datastructures.hpp>

// crop
#include <roofer/io/PointCloudReader.hpp>
#include <roofer/io/PointCloudWriter.hpp>
#include <roofer/io/RasterWriter.hpp>
#include <roofer/io/StreamCropper.hpp>
#include <roofer/io/VectorReader.hpp>
#include <roofer/io/VectorWriter.hpp>
#include <roofer/misc/NodataCircleComputer.hpp>
#include <roofer/misc/PointcloudRasteriser.hpp>
#include <roofer/misc/Vector2DOps.hpp>
#include <roofer/misc/projHelper.hpp>
#include <roofer/misc/select_pointcloud.hpp>

// reconstruct
// #include <roofer/io/PointCloudReader.hpp>
// #include <roofer/io/VectorReader.hpp>
#include <roofer/misc/PC2MeshDistCalculator.hpp>
// #include <roofer/misc/projHelper.hpp>
#include <roofer/reconstruction/AlphaShaper.hpp>
#include <roofer/reconstruction/ArrangementBuilder.hpp>
#include <roofer/reconstruction/ArrangementDissolver.hpp>
#include <roofer/reconstruction/ArrangementExtruder.hpp>
#include <roofer/reconstruction/ArrangementOptimiser.hpp>
#include <roofer/reconstruction/ArrangementSnapper.hpp>
#include <roofer/reconstruction/LineDetector.hpp>
#include <roofer/reconstruction/LineRegulariser.hpp>
#include <roofer/reconstruction/MeshTriangulator.hpp>
#include <roofer/reconstruction/PlaneDetector.hpp>
#include <roofer/reconstruction/PlaneIntersector.hpp>
#include <roofer/reconstruction/SegmentRasteriser.hpp>
#include <roofer/reconstruction/SimplePolygonExtruder.hpp>

// serialisation
#include <roofer/io/CityJsonWriter.hpp>

#include "argh.h"
#include "git.h"
#undef LF_USE_HWLOC
#include "libfork/core.hpp"
#include "libfork/schedule.hpp"
#include "toml.hpp"
// roofer crop
// roofer reconstruct
// roofer tile

using fileExtent = std::pair<std::string, roofer::TBox<double>>;

struct InputPointcloud {
  std::string path;
  std::string name;
  int quality;
  int date = 0;
  int bld_class = 6;
  int grnd_class = 2;
  bool force_low_lod = false;
  bool select_only_for_date = false;

  roofer::vec1f nodata_radii;
  roofer::vec1f nodata_fractions;
  roofer::vec1f pt_densities;
  roofer::vec1b is_mutated;
  std::vector<roofer::LinearRing> nodata_circles;
  std::vector<roofer::PointCollection> building_clouds;
  std::vector<roofer::ImageMap> building_rasters;
  roofer::vec1f ground_elevations;
  roofer::vec1i acquisition_years;

  std::unique_ptr<roofer::misc::RTreeInterface> rtree;
  std::vector<fileExtent> file_extents;
};

struct BuildingObject {
  roofer::PointCollection pointcloud;
  roofer::LinearRing footprint;

  std::unordered_map<int, roofer::Mesh> multisolids_lod12;
  std::unordered_map<int, roofer::Mesh> multisolids_lod13;
  std::unordered_map<int, roofer::Mesh> multisolids_lod22;

  size_t attribute_index;

  // set in crop
  std::string jsonl_path;
  float h_ground;
  bool skip;  // kas_warenhuis / low_lod

  // set in reconstruction
  std::string roof_type;
  float roof_elevation_50p;
  float roof_elevation_70p;
  float roof_elevation_min;
  float roof_elevation_max;
  float rmse_lod12;
  float rmse_lod13;
  float rmse_lod22;
  bool was_skipped;  // b3_reconstructie_onvolledig;
};

struct BuildingTile {
  std::deque<BuildingObject> buildings;
  roofer::AttributeVecMap attributes;
  // offset
  std::unique_ptr<roofer::misc::projHelperInterface> proj_helper;
  // extent
  roofer::TBox<double> extent;
};

struct RooferConfig {
  // footprint source parameters
  std::string source_footprints;
  std::string building_bid_attribute;
  std::string low_lod_attribute = "kas_warenhuis";
  std::string year_of_construction_attribute;

  // crop parameters
  float max_point_density = 20;
  float cellsize = 0.5;
  int low_lod_area = 69000;
  float max_point_density_low_lod = 5;
  float tilesize_x = 1000;
  float tilesize_y = 1000;

  bool write_crop_outputs = false;
  bool output_all = false;
  bool write_rasters = false;
  bool write_metadata = false;
  bool write_index = false;

  // general parameters
  std::optional<roofer::TBox<double>> region_of_interest;
  std::string output_crs;

  // crop output
  std::string building_toml_file_spec;
  std::string building_las_file_spec;
  std::string building_gpkg_file_spec;
  std::string building_raster_file_spec;
  std::string building_jsonl_file_spec;
  std::string jsonl_list_file_spec;
  std::string index_file_spec;
  std::string metadata_json_file_spec;
  std::string crop_output_path;

  // reconstruct
  //...
};

#include "crop_tile.hpp"
#include "reconstruct_building.hpp"

void print_help(std::string program_name) {
  // see http://docopt.org/
  std::cout << "Usage:" << "\n";
  std::cout << "   " << program_name;
  std::cout << " -c <file>" << "\n";
  std::cout << "Options:" << "\n";
  std::cout << "   -h, --help                   Show this help message."
            << "\n";
  std::cout << "   -V, --version                Show version." << "\n";
  std::cout << "   -v, --verbose                Be more verbose." << "\n";
  std::cout << "   -t, --trace                  Trace the progress. Implies "
               "--verbose."
            << "\n";
  std::cout << "   -c <file>, --config <file>   Config file." << "\n";
  std::cout << "   -r, --rasters                Output rasterised building "
               "pointclouds."
            << "\n";
  std::cout << "   -m, --metadata               Output metadata.json file."
            << "\n";
  std::cout << "   -i, --index                  Output index.gpkg file."
            << "\n";
  std::cout << "   -a, --all                    Output files for each "
               "candidate point cloud instead of only the optimal candidate."
            << "\n";
}

void print_version() {
  std::cout << fmt::format(
      "roofer {} ({}{}{})\n", git_Describe(),
      std::strcmp(git_Branch(), "main") ? ""
                                        : fmt::format("{}, ", git_Branch()),
      git_AnyUncommittedChanges() ? "dirty, " : "", git_CommitDate());
}

void read_config(const std::string& config_path, RooferConfig& cfg,
                 std::vector<InputPointcloud>& input_pointclouds) {
  auto& logger = roofer::logger::Logger::get_logger();
  toml::table config;
  config = toml::parse_file(config_path);

  auto tml_source_footprints =
      config["input"]["footprint"]["path"].value<std::string>();
  if (tml_source_footprints.has_value())
    cfg.source_footprints = *tml_source_footprints;

  auto id_attribute_ =
      config["input"]["footprint"]["id_attribute"].value<std::string>();
  if (id_attribute_.has_value()) cfg.building_bid_attribute = *id_attribute_;
  auto low_lod_attribute_ =
      config["input"]["low_lod_attribute"].value<std::string>();
  if (low_lod_attribute_.has_value())
    cfg.low_lod_attribute = *low_lod_attribute_;
  auto year_of_construction_attribute_ =
      config["input"]["year_of_construction_attribute"].value<std::string>();
  if (year_of_construction_attribute_.has_value())
    cfg.year_of_construction_attribute = *year_of_construction_attribute_;

  auto tml_pointclouds = config["input"]["pointclouds"];
  if (toml::array* arr = tml_pointclouds.as_array()) {
    // visitation with for_each() helps deal with heterogeneous data
    for (auto& el : *arr) {
      toml::table* tb = el.as_table();
      auto& pc = input_pointclouds.emplace_back();

      if (auto n = (*tb)["name"].value<std::string>(); n.has_value()) {
        pc.name = *n;
      }
      if (auto n = (*tb)["quality"].value<int>(); n.has_value()) {
        pc.quality = *n;
      }
      if (auto n = (*tb)["date"].value<int>(); n.has_value()) {
        pc.date = *n;
      }
      if (auto n = (*tb)["force_low_lod"].value<bool>(); n.has_value()) {
        pc.force_low_lod = *n;
      }
      if (auto n = (*tb)["select_only_for_date"].value<bool>(); n.has_value()) {
        pc.select_only_for_date = *n;
      }

      if (auto n = (*tb)["building_class"].value<int>(); n.has_value()) {
        pc.bld_class = *n;
      }
      if (auto n = (*tb)["ground_class"].value<int>(); n.has_value()) {
        pc.grnd_class = *n;
      }

      auto tml_path = (*tb)["path"].value<std::string>();
      if (tml_path.has_value()) {
        pc.path = *tml_path;
      }
    };
  }

  auto max_point_density_ =
      config["parameters"]["max_point_density"].value<float>();
  if (max_point_density_.has_value())
    cfg.max_point_density = *max_point_density_;

  auto tilesize_x_ = config["parameters"]["tilesize_x"].value<float>();
  if (tilesize_x_.has_value()) cfg.tilesize_x = *tilesize_x_;
  auto tilesize_y_ = config["parameters"]["tilesize_y"].value<float>();
  if (tilesize_y_.has_value()) cfg.tilesize_y = *tilesize_y_;

  auto cellsize_ = config["parameters"]["cellsize"].value<float>();
  if (cellsize_.has_value()) cfg.cellsize = *cellsize_;

  auto low_lod_area_ = config["parameters"]["low_lod_area"].value<int>();
  if (low_lod_area_.has_value()) cfg.low_lod_area = *low_lod_area_;

  if (toml::array* region_of_interest_ =
          config["parameters"]["region_of_interest"].as_array()) {
    if (region_of_interest_->size() == 4 &&
        (region_of_interest_->is_homogeneous(toml::node_type::floating_point) ||
         region_of_interest_->is_homogeneous(toml::node_type::integer))) {
      cfg.region_of_interest =
          roofer::TBox<double>{*region_of_interest_->get(0)->value<double>(),
                               *region_of_interest_->get(1)->value<double>(),
                               *region_of_interest_->get(2)->value<double>(),
                               *region_of_interest_->get(3)->value<double>()};
    } else {
      logger.error("Failed to read parameter.region_of_interest");
    }
  }
  auto building_toml_file_spec_ =
      config["output"]["building_toml_file"].value<std::string>();
  if (building_toml_file_spec_.has_value())
    cfg.building_toml_file_spec = *building_toml_file_spec_;

  auto building_las_file_spec_ =
      config["output"]["building_las_file"].value<std::string>();
  if (building_las_file_spec_.has_value())
    cfg.building_las_file_spec = *building_las_file_spec_;

  auto building_gpkg_file_spec_ =
      config["output"]["building_gpkg_file"].value<std::string>();
  if (building_gpkg_file_spec_.has_value())
    cfg.building_gpkg_file_spec = *building_gpkg_file_spec_;

  auto building_raster_file_spec_ =
      config["output"]["building_raster_file"].value<std::string>();
  if (building_raster_file_spec_.has_value())
    cfg.building_raster_file_spec = *building_raster_file_spec_;

  if (cfg.write_metadata) {
    auto metadata_json_file_spec_ =
        config["output"]["metadata_json_file"].value<std::string>();
    if (metadata_json_file_spec_.has_value())
      cfg.metadata_json_file_spec = *metadata_json_file_spec_;
  }

  auto building_jsonl_file_spec_ =
      config["output"]["building_jsonl_file"].value<std::string>();
  if (building_jsonl_file_spec_.has_value())
    cfg.building_jsonl_file_spec = *building_jsonl_file_spec_;

  auto index_file_spec_ = config["output"]["index_file"].value<std::string>();
  if (index_file_spec_.has_value()) cfg.index_file_spec = *index_file_spec_;

  auto jsonl_list_file_spec_ =
      config["output"]["jsonl_list_file"].value<std::string>();
  if (jsonl_list_file_spec_.has_value())
    cfg.jsonl_list_file_spec = *jsonl_list_file_spec_;

  auto output_path_ = config["output"]["path"].value<std::string>();
  if (output_path_.has_value()) cfg.crop_output_path = *output_path_;

  auto output_crs_ = config["output"]["crs"].value<std::string>();
  if (output_crs_.has_value()) cfg.output_crs = *output_crs_;
}

void get_las_extents(InputPointcloud& ipc) {
  ;
  auto pj = roofer::misc::createProjHelper();
  for (auto& fp :
       roofer::find_filepaths(ipc.path, {".las", ".LAS", ".laz", ".LAZ"})) {
    auto PointReader = roofer::io::createPointCloudReaderLASlib(*pj);
    PointReader->open(fp);
    ipc.file_extents.push_back(std::make_pair(fp, PointReader->getExtent()));
  }
}

std::vector<roofer::TBox<double>> create_tiles(roofer::TBox<double>& roi,
                                               double tilesize_x,
                                               double tilesize_y) {
  size_t dimx_ = (roi.max()[0] - roi.min()[0]) / tilesize_x + 1;
  size_t dimy_ = (roi.max()[1] - roi.min()[1]) / tilesize_y + 1;
  std::vector<roofer::TBox<double>> tiles;

  for (size_t col = 0; col < dimx_; ++col) {
    for (size_t row = 0; row < dimy_; ++row) {
      tiles.emplace_back(roofer::TBox<double>{
          roi.min()[0] + col * tilesize_x, roi.min()[1] + row * tilesize_y, 0.,
          roi.min()[0] + (col + 1) * tilesize_x,
          roi.min()[1] + (row + 1) * tilesize_y, 0.});
    }
  }
  return tiles;
}

// Coroutine wrapper for reconstructing a single building.
// Because, libfork::fork requires an awaitable function.
inline constexpr auto reconstruct_building_coro =
    [](auto reconstruct_building_coro,
       BuildingObject building_object) -> lf::task<BuildingObject> {
  reconstruct_building(building_object);
  co_return building_object;
};

// Parallel reconstruction of a single BuildingTile.
// Uses the fork-join method, with the `libfork` library.
// Coroutines should take arguments by value.
// Ref:
// https://github.com/ConorWilliams/libfork?tab=readme-ov-file#the-cactus-stack
inline constexpr auto reconstruct_tile_parallel =
    [](auto reconstruct_tile_parallel,
       BuildingTile building_tile) -> lf::task<BuildingTile> {
  // Allocate space for results, outputs is a std::span<Points>
  auto [outputs] =
      co_await lf::co_new<BuildingObject>(building_tile.buildings.size());

  for (std::size_t i = 0; i < building_tile.buildings.size(); ++i) {
    co_await lf::fork(&outputs[i],
                      reconstruct_building_coro)(building_tile.buildings[i]);
  }
  co_await lf::join;  // Wait for all tasks to complete.

  building_tile.buildings.assign(outputs.begin(), outputs.end());
  co_return building_tile;
};

// Overrides for heap allocation counting
// Ref.: https://www.youtube.com/watch?v=sLlGEUO_EGE
namespace {
  struct HeapAllocationCounter {
    uint32_t total_allocated = 0;
    uint32_t total_freed = 0;
    [[nodiscard]] uint32_t current_usage() const {
      return total_allocated - total_freed;
    };
  };
  HeapAllocationCounter s_HeapAllocationCounter;
}  // namespace

void* operator new(size_t size) {
  s_HeapAllocationCounter.total_allocated += size;
  return malloc(size);
}
void operator delete(void* memory, size_t size) {
  s_HeapAllocationCounter.total_freed += size;
  free(memory);
};

int main(int argc, const char* argv[]) {
  auto cmdl = argh::parser({"-c", "--config"});

  cmdl.parse(argc, argv);
  std::string program_name = cmdl[0];
  auto& logger = roofer::logger::Logger::get_logger();

  // read cmdl options
  RooferConfig roofer_cfg;

  roofer_cfg.output_all = cmdl[{"-a", "--all"}];
  roofer_cfg.write_rasters = cmdl[{"-r", "--rasters"}];
  roofer_cfg.write_metadata = cmdl[{"-m", "--metadata"}];
  roofer_cfg.write_index = cmdl[{"-i", "--index"}];

  if (cmdl[{"-h", "--help"}]) {
    print_help(program_name);
    return EXIT_SUCCESS;
  }
  if (cmdl[{"-V", "--version"}]) {
    print_version();
    return EXIT_SUCCESS;
  }
  if (cmdl[{"-v", "--verbose"}]) {
    logger.set_level(roofer::logger::LogLevel::debug);
  } else {
    logger.set_level(roofer::logger::LogLevel::warning);
  }
  // Enabling tracing overwrites the log level
  if (cmdl[{"-t", "--trace"}]) {
    logger.set_level(roofer::logger::LogLevel::trace);
  }

  // Read configuration
  std::string config_path;
  std::vector<InputPointcloud> input_pointclouds;
  if (cmdl({"-c", "--config"}) >> config_path) {
    if (!fs::exists(config_path)) {
      logger.error("No such config file: {}", config_path);
      print_help(program_name);
      return EXIT_FAILURE;
    }
    logger.info("Reading configuration from file {}", config_path);
    try {
      read_config(config_path, roofer_cfg, input_pointclouds);
    } catch (const std::exception& e) {
      logger.error("Unable to parse config file {}.\n{}", config_path,
                   e.what());
      return EXIT_FAILURE;
    }
  } else {
    logger.error("No config file specified");
    return EXIT_FAILURE;
  }

  // precomputation for tiling
  std::deque<BuildingTile> building_tiles;

  for (auto& ipc : input_pointclouds) {
    get_las_extents(ipc);
    ipc.rtree = roofer::misc::createRTreeGEOS();
    for (auto& item : ipc.file_extents) {
      ipc.rtree->insert(item.second, &item);
    }
  }

  // Compute batch tile regions
  // we just create one tile for now
  {
    auto pj = roofer::misc::createProjHelper();
    auto VectorReader = roofer::io::createVectorReaderOGR(*pj);
    VectorReader->open(roofer_cfg.source_footprints);
    logger.info("region_of_interest.has_value()? {}",
                roofer_cfg.region_of_interest.has_value());
    logger.info("Reading footprints from {}", roofer_cfg.source_footprints);

    roofer::TBox<double> roi;
    if (roofer_cfg.region_of_interest.has_value()) {
      // VectorReader->region_of_interest = *roofer_cfg.region_of_interest;
      roi = *roofer_cfg.region_of_interest;
    } else {
      roi = VectorReader->layer_extent;
    }

    // actual tiling
    auto tile_extents =
        create_tiles(roi, roofer_cfg.tilesize_x, roofer_cfg.tilesize_y);

    for (auto& tile : tile_extents) {
      auto& building_tile = building_tiles.emplace_back();
      building_tile.extent = tile;
      building_tile.proj_helper = roofer::misc::createProjHelper();
    }
  }

  // Multithreading setup
  size_t nthreads = std::thread::hardware_concurrency();
  // -3, because we need one thread for crop, reconstruct, serialize each
  lf::lazy_pool pool(nthreads - 3);

  std::mutex cropped_mutex;  // protects the cropped queue
  std::deque<BuildingTile> deque_cropped;
  std::condition_variable cropped_pending;

  std::mutex reconstructed_mutex;  // protects the reconstructed queue
  std::deque<BuildingTile> deque_reconstructed;
  std::condition_variable reconstructed_pending;

  std::atomic crop_running{true};
  std::atomic reconstruction_running{true};

  // Counters for tracing
  std::atomic<size_t> cropped_count = 0;
  std::atomic<size_t> reconstructed_count = 0;
  std::atomic<size_t> serialized_count = 0;

  logger.trace("heap", s_HeapAllocationCounter.current_usage());
  // Process tiles
  std::thread cropper([&]() {
    logger.trace("crop", cropped_count);
    for (auto& building_tile : building_tiles) {
      // crop each tile
      crop_tile(building_tile.extent,  // tile extent
                input_pointclouds,     // input pointclouds
                building_tile,         // output building data
                roofer_cfg);           // configuration parameters
      {
        std::scoped_lock lock{cropped_mutex};
        cropped_count += building_tile.buildings.size();
        deque_cropped.push_back(std::move(building_tile));
      }
      logger.trace("crop", cropped_count);
      logger.trace("heap", s_HeapAllocationCounter.current_usage());
      cropped_pending.notify_one();
    }
    crop_running.store(false);
  });

  std::thread reconstructor([&]() {
    logger.trace("reconstruct", reconstructed_count);
    while (crop_running.load()) {
      std::unique_lock lock{cropped_mutex};
      cropped_pending.wait(lock);
      if (deque_cropped.empty()) continue;
      // Temporary queue so we can quickly move off items of the producer queue
      // and process them independently.
      auto pending_cropped{std::move(deque_cropped)};
      deque_cropped.clear();
      lock.unlock();

      while (!pending_cropped.empty()) {
        auto& building_tile = pending_cropped.front();
        auto reconstructed_tile = lf::sync_wait(pool, reconstruct_tile_parallel,
                                                std::move(building_tile));
        {
          std::scoped_lock lock_reconstructed{reconstructed_mutex};
          reconstructed_count += reconstructed_tile.buildings.size();
          deque_reconstructed.push_back(std::move(reconstructed_tile));
        }
        logger.trace("reconstruct", reconstructed_count);
        logger.trace("heap", s_HeapAllocationCounter.current_usage());
        reconstructed_pending.notify_one();
        pending_cropped.pop_front();
      }
    }
    reconstruction_running.store(false);
  });

  std::thread serializer([&]() {
    logger.trace("serialize", serialized_count);
    while (reconstruction_running.load()) {
      std::unique_lock lock{reconstructed_mutex};
      reconstructed_pending.wait(lock);
      if (deque_reconstructed.empty()) continue;
      auto pending_reconstructed{std::move(deque_reconstructed)};
      deque_reconstructed.clear();
      lock.unlock();

      while (!pending_reconstructed.empty()) {
        auto& building_tile = pending_reconstructed.front();
        // output reconstructed buildings
        auto CityJsonWriter =
            roofer::io::createCityJsonWriter(*building_tile.proj_helper);
        for (auto& building : building_tile.buildings) {

          CityJsonWriter->write(building.jsonl_path, building.footprint,
                                &building.multisolids_lod12, &building.multisolids_lod13,
                                &building.multisolids_lod22, building_tile.attributes, building.attribute_index);
          logger.info("Completed CityJsonWriter to {}", building.jsonl_path);
        }

        // buildings are finishes processing so they can be cleared
        serialized_count += building_tile.buildings.size();
        logger.trace("serialize", serialized_count);
        logger.trace("heap", s_HeapAllocationCounter.current_usage());
        building_tile.buildings.clear();
        pending_reconstructed.pop_front();
      }
    }
  });
  logger.trace("heap", s_HeapAllocationCounter.current_usage());

  cropper.join();
  reconstructor.join();
  serializer.join();
}
