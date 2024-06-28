#include <roofer/logger/logger.h>

#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
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
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "argh.h"
#include "fmt/format.h"
#include "git.h"
#include "toml.hpp"

namespace fs = std::filesystem;

void print_help(std::string program_name) {
  // see http://docopt.org/
  std::cout << "Usage:" << "\n";
  std::cout << "   " << program_name;
  std::cout << " -c <file>" << "\n";
  std::cout << "Options:" << "\n";
  // std::cout << "   -v, --version                Print version information\n";
  std::cout << "   -h, --help                   Show this help message" << "\n";
  std::cout << "   -V, --version                Show version" << "\n";
  std::cout << "   -v, --verbose                Be more verbose" << "\n";
  std::cout << "   -c <file>, --config <file>   Config file" << "\n";
  std::cout << "   -r, --rasters                Output rasterised building "
               "pointclouds"
            << "\n";
  std::cout << "   -m, --metadata               Output metadata.json file"
            << "\n";
  std::cout << "   -i, --index                  Output index.gpkg file" << "\n";
  std::cout << "   -a, --all                    Output files for each "
               "candidate point cloud instead of only the optimal candidate"
            << "\n";
}

void print_version() {
  std::cout << fmt::format(
      "roofer {} ({}{}{})\n", git_Describe(),
      std::strcmp(git_Branch(), "main") ? ""
                                        : fmt::format("{}, ", git_Branch()),
      git_AnyUncommittedChanges() ? "dirty, " : "", git_CommitDate());
}

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
};

int main(int argc, const char* argv[]) {
  // auto cmdl = argh::parser({ "-f", "--footprint", "-p", "--pointcloud" });
  auto cmdl = argh::parser({"-c", "--config"});

  cmdl.parse(argc, argv);
  std::string program_name = cmdl[0];

  std::string
      path_footprint;  // =
                       // "/mnt/Data/LocalData/Kadaster/true_ortho_experimenten/2021_LAZ_Leiden_Almere/DenHaag/bag_83000_455000.gpkg";

  bool output_all = cmdl[{"-a", "--all"}];
  bool write_rasters = cmdl[{"-r", "--rasters"}];
  bool write_metadata = cmdl[{"-m", "--metadata"}];
  bool write_index = cmdl[{"-i", "--index"}];
  bool verbose = cmdl[{"-v", "--verbose"}];
  bool version = cmdl[{"-V", "--version"}];

  if (cmdl[{"-h", "--help"}]) {
    print_help(program_name);
    return EXIT_SUCCESS;
  }
  if (version) {
    print_version();
    return EXIT_SUCCESS;
  }

  auto& logger = roofer::logger::Logger::get_logger();

  if (verbose) {
    logger.set_level(roofer::logger::LogLevel::debug);
  } else {
    logger.set_level(roofer::logger::LogLevel::warning);
  }

  std::vector<InputPointcloud> input_pointclouds;

  // TOML config parsing
  // pointclouds, footprints
  float max_point_density = 20;
  float max_point_density_low_lod = 5;
  float cellsize = 0.5;
  float low_lod_area = 69000.0;
  std::optional<roofer::TBox<double>> region_of_interest;
  std::string year_of_construction_attribute = "oorspronkelijkbouwjaar";
  std::string low_lod_attribute = "kas_warenhuis";
  std::string output_crs = "";
  std::string config_path;
  std::string building_toml_file_spec;
  std::string building_las_file_spec;
  std::string building_gpkg_file_spec;
  std::string building_raster_file_spec;
  std::string building_jsonl_file_spec;
  std::string jsonl_list_file_spec;
  std::string index_file_spec;
  std::string metadata_json_file_spec;
  std::string output_path;
  std::string building_bid_attribute;
  toml::table config;
  if (cmdl({"-c", "--config"}) >> config_path) {
    if (!fs::exists(config_path)) {
      logger.error("No such config file: {}", config_path);
      print_help(program_name);
      return EXIT_FAILURE;
    }
    logger.info("Reading configuration from file {}", config_path);
    try {
      config = toml::parse_file(config_path);
    } catch (const std::exception& e) {
      logger.error("Unable to parse config file {}.\n{}", config_path,
                   e.what());
      return EXIT_FAILURE;
    }

    auto tml_path_footprint =
        config["input"]["footprint"]["path"].value<std::string>();
    if (tml_path_footprint.has_value()) path_footprint = *tml_path_footprint;

    auto id_attribute_ =
        config["input"]["footprint"]["id_attribute"].value<std::string>();
    if (id_attribute_.has_value()) building_bid_attribute = *id_attribute_;
    auto low_lod_attribute_ =
        config["input"]["low_lod_attribute"].value<std::string>();
    if (low_lod_attribute_.has_value()) low_lod_attribute = *low_lod_attribute_;
    auto year_of_construction_attribute_ =
        config["input"]["year_of_construction_attribute"].value<std::string>();
    if (year_of_construction_attribute_.has_value())
      year_of_construction_attribute = *year_of_construction_attribute_;

    auto tml_pointclouds = config["input"]["pointclouds"];
    if (toml::array* arr = tml_pointclouds.as_array()) {
      // visitation with for_each() helps deal with heterogeneous data
      for (auto& el : *arr) {
        toml::table* tb = el.as_table();
        InputPointcloud pc;

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
        if (auto n = (*tb)["select_only_for_date"].value<bool>();
            n.has_value()) {
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
        input_pointclouds.push_back(pc);
      };
    }

    auto max_point_density_ =
        config["parameters"]["max_point_density"].value<float>();
    if (max_point_density_.has_value()) max_point_density = *max_point_density_;

    auto cellsize_ = config["parameters"]["cellsize"].value<float>();
    if (cellsize_.has_value()) cellsize = *cellsize_;

    auto low_lod_area_ = config["parameters"]["low_lod_area"].value<int>();
    if (low_lod_area_.has_value()) low_lod_area = *low_lod_area_;

    if (toml::array* region_of_interest_ =
            config["parameters"]["region_of_interest"].as_array()) {
      if (region_of_interest_->size() == 4 &&
          (region_of_interest_->is_homogeneous(
               toml::node_type::floating_point) ||
           region_of_interest_->is_homogeneous(toml::node_type::integer))) {
        region_of_interest = {*region_of_interest_->get(0)->value<double>(),
                              *region_of_interest_->get(1)->value<double>(),
                              0,
                              *region_of_interest_->get(2)->value<double>(),
                              *region_of_interest_->get(3)->value<double>(),
                              0};
      } else {
        logger.error("Failed to read parameter.region_of_interest");
      }
    }
    // logger.info("Region of interest: {}", fmt::format("{}",
    // region_of_interest));

    auto building_toml_file_spec_ =
        config["output"]["building_toml_file"].value<std::string>();
    if (building_toml_file_spec_.has_value())
      building_toml_file_spec = *building_toml_file_spec_;

    auto building_las_file_spec_ =
        config["output"]["building_las_file"].value<std::string>();
    if (building_las_file_spec_.has_value())
      building_las_file_spec = *building_las_file_spec_;

    auto building_gpkg_file_spec_ =
        config["output"]["building_gpkg_file"].value<std::string>();
    if (building_gpkg_file_spec_.has_value())
      building_gpkg_file_spec = *building_gpkg_file_spec_;

    auto building_raster_file_spec_ =
        config["output"]["building_raster_file"].value<std::string>();
    if (building_raster_file_spec_.has_value())
      building_raster_file_spec = *building_raster_file_spec_;

    if (write_metadata) {
      auto metadata_json_file_spec_ =
          config["output"]["metadata_json_file"].value<std::string>();
      if (metadata_json_file_spec_.has_value())
        metadata_json_file_spec = *metadata_json_file_spec_;
    }

    auto building_jsonl_file_spec_ =
        config["output"]["building_jsonl_file"].value<std::string>();
    if (building_jsonl_file_spec_.has_value())
      building_jsonl_file_spec = *building_jsonl_file_spec_;

    auto index_file_spec_ = config["output"]["index_file"].value<std::string>();
    if (index_file_spec_.has_value()) index_file_spec = *index_file_spec_;

    auto jsonl_list_file_spec_ =
        config["output"]["jsonl_list_file"].value<std::string>();
    if (jsonl_list_file_spec_.has_value())
      jsonl_list_file_spec = *jsonl_list_file_spec_;

    auto output_path_ = config["output"]["path"].value<std::string>();
    if (output_path_.has_value()) output_path = *output_path_;

    auto output_crs_ = config["output"]["crs"].value<std::string>();
    if (output_crs_.has_value()) output_crs = *output_crs_;

  } else {
    logger.error("No config file specified");
    return EXIT_FAILURE;
  }

  auto pj = roofer::misc::createProjHelper();
  auto VectorReader = roofer::io::createVectorReaderOGR(*pj);
  auto VectorWriter = roofer::io::createVectorWriterOGR(*pj);
  VectorWriter->srs = output_crs;
  auto RasterWriter = roofer::io::createRasterWriterGDAL(*pj);
  auto PointCloudCropper = roofer::io::createPointCloudCropper(*pj);
  auto VectorOps = roofer::misc::createVector2DOpsGEOS();
  auto LASWriter = roofer::io::createLASWriter(*pj);

  VectorReader->open(path_footprint);
  logger.info("region_of_interest.has_value()? {}",
              region_of_interest.has_value());
  if (region_of_interest.has_value())
    VectorReader->region_of_interest = *region_of_interest;
  logger.info("Reading footprints from {}", path_footprint);
  std::vector<roofer::LinearRing> footprints;
  roofer::AttributeVecMap attributes;
  VectorReader->readPolygons(footprints, &attributes);

  const unsigned N_fp = footprints.size();

  // check if low_lod_attribute exists, if not then create it
  if (!attributes.get_if<bool>(low_lod_attribute)) {
    auto& vec = attributes.insert_vec<bool>(low_lod_attribute);
    vec.resize(N_fp, false);
  }
  auto low_lod_vec = attributes.get_if<bool>(low_lod_attribute);
  for (size_t i = 0; i < N_fp; ++i) {
    // need dereference operator here for dereferencing pointer and getting
    // std::option value
    (*low_lod_vec)[i] = *(*low_lod_vec)[i] ||
                        std::fabs(footprints[i].signed_area()) > low_lod_area;
  }

  // get yoc attribute vector (nullptr if it does not exist)
  bool use_acquisition_year = true;
  auto yoc_vec = attributes.get_if<int>(year_of_construction_attribute);
  if (!yoc_vec) {
    use_acquisition_year = false;
    logger.warning(
        "year_of_construction_attribute '{}' not found in input footprints",
        year_of_construction_attribute);
  }

  // simplify + buffer footprints
  logger.info("Simplifying and buffering footprints...");
  VectorOps->simplify_polygons(footprints);
  auto buffered_footprints = footprints;
  VectorOps->buffer_polygons(buffered_footprints);

  roofer::Box polygon_extent;
  for (auto& buf_ring : buffered_footprints) {
    polygon_extent.add(buf_ring.box());
  }

  // Crop all pointclouds
  for (auto& ipc : input_pointclouds) {
    logger.info("Cropping pointcloud {}...", ipc.name);

    PointCloudCropper->process(ipc.path, footprints, buffered_footprints,
                               ipc.building_clouds, ipc.ground_elevations,
                               ipc.acquisition_years, polygon_extent,
                               {.ground_class = ipc.grnd_class,
                                .building_class = ipc.bld_class,
                                .use_acquisition_year = use_acquisition_year});
    if (ipc.date != 0) {
      logger.info("Overriding acquisition year from config file");
      std::fill(ipc.acquisition_years.begin(), ipc.acquisition_years.end(),
                ipc.date);
    }
  }

  // compute rasters
  // thin
  // compute nodata maxcircle
  for (auto& ipc : input_pointclouds) {
    logger.info("Analysing pointcloud {}...", ipc.name);
    ipc.nodata_radii.resize(N_fp);
    ipc.building_rasters.resize(N_fp);
    ipc.nodata_fractions.resize(N_fp);
    ipc.pt_densities.resize(N_fp);
    if (write_index) ipc.nodata_circles.resize(N_fp);

    // auto& r_nodata = attributes.insert_vec<float>("r_nodata_"+ipc.name);
    roofer::arr2f nodata_c;
    for (unsigned i = 0; i < N_fp; ++i) {
      roofer::misc::RasterisePointcloud(ipc.building_clouds[i], footprints[i],
                                        ipc.building_rasters[i], cellsize);
      ipc.nodata_fractions[i] =
          roofer::misc::computeNoDataFraction(ipc.building_rasters[i]);
      ipc.pt_densities[i] =
          roofer::misc::computePointDensity(ipc.building_rasters[i]);

      auto target_density = max_point_density;
      bool low_lod = *(*low_lod_vec)[i];
      if (low_lod) {
        target_density = max_point_density_low_lod;
        logger.info(
            "Applying extra thinning and skipping nodata circle calculation "
            "[low_lod_attribute = {}]",
            low_lod);
      }

      roofer::misc::gridthinPointcloud(ipc.building_clouds[i],
                                       ipc.building_rasters[i]["cnt"],
                                       target_density);

      if (low_lod) {
        ipc.nodata_radii[i] = 0;
      } else {
        roofer::misc::compute_nodata_circle(ipc.building_clouds[i],
                                            footprints[i], &ipc.nodata_radii[i],
                                            &nodata_c);
        // if (write_index) {
        //   roofer::draw_circle(
        //     ipc.nodata_circles[i],
        //     ipc.nodata_radii[i],
        //     nodata_c
        //   );
        // }
      }
    }
  }

  // add raster stats attributes from PointCloudCropper to footprint attributes
  for (auto& ipc : input_pointclouds) {
    auto& nodata_r = attributes.insert_vec<float>("nodata_r_" + ipc.name);
    auto& nodata_frac = attributes.insert_vec<float>("nodata_frac_" + ipc.name);
    auto& pt_density = attributes.insert_vec<float>("pt_density_" + ipc.name);
    nodata_r.reserve(N_fp);
    nodata_frac.reserve(N_fp);
    pt_density.reserve(N_fp);
    for (unsigned i = 0; i < N_fp; ++i) {
      nodata_r.push_back(ipc.nodata_radii[i]);
      nodata_frac.push_back(ipc.nodata_fractions[i]);
      pt_density.push_back(ipc.pt_densities[i]);
    }
  }

  // compute is_mutated attribute for first 2 pointclouds and add to footprint
  // attributes
  roofer::misc::selectPointCloudConfig select_pc_cfg;
  if (input_pointclouds.size() > 1) {
    auto& is_mutated =
        attributes.insert_vec<bool>("is_mutated_" + input_pointclouds[0].name +
                                    "_" + input_pointclouds[1].name);
    is_mutated.reserve(N_fp);
    for (unsigned i = 0; i < N_fp; ++i) {
      is_mutated[i] =
          roofer::misc::isMutated(input_pointclouds[0].building_rasters[i],
                                  input_pointclouds[1].building_rasters[i],
                                  select_pc_cfg.threshold_mutation_fraction,
                                  select_pc_cfg.threshold_mutation_difference);
    }
  }

  // select pointcloud and write out geoflow config + pointcloud / fp for each
  // building
  logger.info("Selecting and writing pointclouds");
  auto bid_vec = attributes.get_if<std::string>(building_bid_attribute);
  auto& pc_select = attributes.insert_vec<std::string>("pc_select");
  auto& pc_source = attributes.insert_vec<std::string>("pc_source");
  auto& pc_year = attributes.insert_vec<int>("pc_year");
  std::unordered_map<std::string, roofer::vec1s> jsonl_paths;
  std::string bid;
  bool only_write_selected = !output_all;
  for (unsigned i = 0; i < N_fp; ++i) {
    if (bid_vec) {
      bid = (*bid_vec)[i].value();
    } else {
      bid = std::to_string(i);
    }
    // spdlog::debug("bid={}", bid);

    std::vector<roofer::misc::CandidatePointCloud> candidates;
    candidates.reserve(input_pointclouds.size());
    std::vector<roofer::misc::CandidatePointCloud> candidates_just_for_data;
    {
      int j = 0;
      for (auto& ipc : input_pointclouds) {
        auto cpc = roofer::misc::CandidatePointCloud{
            ipc.nodata_radii[i],
            ipc.nodata_fractions[i],
            ipc.building_rasters[i],
            yoc_vec ? (*yoc_vec)[i].value_or(-1) : -1,
            ipc.name,
            ipc.quality,
            ipc.acquisition_years[i],
            j++};
        if (ipc.select_only_for_date) {
          candidates_just_for_data.push_back(cpc);
        } else {
          candidates.push_back(cpc);
        }
        jsonl_paths.insert({ipc.name, roofer::vec1s{}});
      }
      jsonl_paths.insert({"", roofer::vec1s{}});
    }

    roofer::misc::PointCloudSelectResult sresult =
        roofer::misc::selectPointCloud(candidates, select_pc_cfg);
    const roofer::misc::CandidatePointCloud* selected =
        sresult.selected_pointcloud;

    // this is a sanity check and should never happen
    if (!selected) {
      logger.error("Unable to select pointcloud");
      exit(1);
    }

    // check if yoc_attribute indicates this building to be built after selected
    // PC
    int yoc = yoc_vec ? (*yoc_vec)[i].value_or(-1) : -1;
    if (yoc != -1 && yoc > selected->date) {
      // force selection of latest pointcloud
      selected = getLatestPointCloud(candidates);
      sresult.explanation = roofer::misc::PointCloudSelectExplanation::_LATEST;
      // overrule if there was a more recent pointcloud with
      // select_only_for_date = true
      if (candidates_just_for_data.size()) {
        if (candidates_just_for_data[0].date > selected->date) {
          selected = &candidates_just_for_data[0];
          // sresult.explanation = roofer::PointCloudSelectExplanation::_LATEST;
        }
      }
    }

    if (sresult.explanation ==
        roofer::misc::PointCloudSelectExplanation::PREFERRED_AND_LATEST)
      pc_select.push_back("PREFERRED_AND_LATEST");
    else if (sresult.explanation ==
             roofer::misc::PointCloudSelectExplanation::PREFERRED_NOT_LATEST)
      pc_select.push_back("PREFERRED_NOT_LATEST");
    else if (sresult.explanation ==
             roofer::misc::PointCloudSelectExplanation::LATEST_WITH_MUTATION)
      pc_select.push_back("LATEST_WITH_MUTATION");
    else if (sresult.explanation == roofer::misc::PointCloudSelectExplanation::
                                        _HIGHEST_YET_INSUFFICIENT_COVERAGE)
      pc_select.push_back("_HIGHEST_YET_INSUFFICIENT_COVERAGE");
    else if (sresult.explanation ==
             roofer::misc::PointCloudSelectExplanation::_LATEST) {
      pc_select.push_back("_LATEST");
      // // clear pc
      // ipc[selected->index].building_clouds[i].clear();
    } else
      pc_select.push_back("NONE");

    pc_source.push_back(selected->name);
    pc_year.push_back(selected->date);
    if (input_pointclouds[selected->index].force_low_lod) {
      (*low_lod_vec)[i] = true;
    }
    {
      // fs::create_directories(fs::path(fname).parent_path());
      std::string fp_path =
          fmt::format(fmt::runtime(building_gpkg_file_spec),
                      fmt::arg("bid", bid), fmt::arg("path", output_path));
      VectorWriter->writePolygons(fp_path, footprints, attributes, i, i + 1);

      size_t j = 0;
      for (auto& ipc : input_pointclouds) {
        if ((selected->index != j) && (only_write_selected)) {
          ++j;
          continue;
        };

        std::string pc_path = fmt::format(
            fmt::runtime(building_las_file_spec), fmt::arg("bid", bid),
            fmt::arg("pc_name", ipc.name), fmt::arg("path", output_path));
        std::string raster_path = fmt::format(
            fmt::runtime(building_raster_file_spec), fmt::arg("bid", bid),
            fmt::arg("pc_name", ipc.name), fmt::arg("path", output_path));
        std::string jsonl_path = fmt::format(
            fmt::runtime(building_jsonl_file_spec), fmt::arg("bid", bid),
            fmt::arg("pc_name", ipc.name), fmt::arg("path", output_path));

        if (write_rasters) {
          RasterWriter->writeBands(raster_path, ipc.building_rasters[i]);
        }

        LASWriter->write_pointcloud(input_pointclouds[j].building_clouds[i],
                                    pc_path);

        // Correct ground height for offset, NB this ignores crs transformation
        double h_ground =
            input_pointclouds[j].ground_elevations[i] + (*pj->data_offset)[2];

        auto gf_config = toml::table{
            {"INPUT_FOOTPRINT", fp_path},
            // {"INPUT_POINTCLOUD", sresult.explanation ==
            // roofer::PointCloudSelectExplanation::_LATEST_BUT_OUTDATED ? "" :
            // pc_path},
            {"INPUT_POINTCLOUD", pc_path},
            {"BID", bid},
            {"GROUND_ELEVATION", h_ground},
            {"OUTPUT_JSONL", jsonl_path},

            {"GF_PROCESS_OFFSET_OVERRIDE", true},
            {"GF_PROCESS_OFFSET_X", (*pj->data_offset)[0]},
            {"GF_PROCESS_OFFSET_Y", (*pj->data_offset)[1]},
            {"GF_PROCESS_OFFSET_Z", (*pj->data_offset)[2]},
            {"skip_attribute_name", low_lod_attribute},
            {"id_attribute", building_bid_attribute},
        };

        if (write_metadata) {
          // gf_config.insert("GF_PROCESS_OFFSET_OVERRIDE", true);
          gf_config.insert("CITYJSON_TRANSLATE_X", (*pj->data_offset)[0]);
          gf_config.insert("CITYJSON_TRANSLATE_Y", (*pj->data_offset)[1]);
          gf_config.insert("CITYJSON_TRANSLATE_Z", (*pj->data_offset)[2]);
          gf_config.insert("CITYJSON_SCALE_X", 0.001);
          gf_config.insert("CITYJSON_SCALE_Y", 0.001);
          gf_config.insert("CITYJSON_SCALE_Z", 0.001);
        }
        auto tbl_gfparams =
            config["output"]["reconstruction_parameters"].as_table();
        gf_config.insert(tbl_gfparams->begin(), tbl_gfparams->end());

        if (!only_write_selected) {
          std::ofstream ofs;
          std::string config_path = fmt::format(
              fmt::runtime(building_toml_file_spec), fmt::arg("bid", bid),
              fmt::arg("pc_name", ipc.name), fmt::arg("path", output_path));
          ofs.open(config_path);
          ofs << gf_config;
          ofs.close();

          jsonl_paths[ipc.name].push_back(jsonl_path);
        }
        if (selected->index == j) {
          // set optimal jsonl path
          std::string jsonl_path = fmt::format(
              fmt::runtime(building_jsonl_file_spec), fmt::arg("bid", bid),
              fmt::arg("pc_name", ""), fmt::arg("path", output_path));
          gf_config.insert_or_assign("OUTPUT_JSONL", jsonl_path);
          jsonl_paths[""].push_back(jsonl_path);

          // write optimal config
          std::ofstream ofs;
          std::string config_path = fmt::format(
              fmt::runtime(building_toml_file_spec), fmt::arg("bid", bid),
              fmt::arg("pc_name", ""), fmt::arg("path", output_path));
          ofs.open(config_path);
          ofs << gf_config;
          ofs.close();
        }
        ++j;
      }
    };
    // write config
  }

  // Write index output
  if (write_index) {
    std::string index_file = fmt::format(fmt::runtime(index_file_spec),
                                         fmt::arg("path", output_path));
    VectorWriter->writePolygons(index_file, footprints, attributes);

    // write nodata circles
    // for (auto& ipc : input_pointclouds) {
    //   VectorWriter->writePolygons(index_file+"_"+ipc.name+"_nodatacircle.gpkg",
    //   ipc.nodata_circles, attributes);
    // }
  }

  // write the txt containing paths to all jsonl features to be written by
  // reconstruct
  {
    for (auto& [name, pathsvec] : jsonl_paths) {
      if (pathsvec.size() != 0) {
        std::string jsonl_list_file = fmt::format(
            fmt::runtime(jsonl_list_file_spec), fmt::arg("path", output_path),
            fmt::arg("pc_name", name));
        std::ofstream ofs;
        ofs.open(jsonl_list_file);
        for (auto& jsonl_p : pathsvec) {
          ofs << jsonl_p << "\n";
        }
        ofs.close();
      }
    }
  }

  // Write metadata.json for json features
  if (write_metadata) {
    auto md_scale = roofer::arr3d{0.001, 0.001, 0.001};
    auto md_trans = *pj->data_offset;

    auto metadatajson = toml::table{
        {"type", "CityJSON"},
        {"version", "2.0"},
        {"CityObjects", toml::table{}},
        {"vertices", toml::array{}},
        {"transform",
         toml::table{
             {"scale", toml::array{md_scale[0], md_scale[1], md_scale[2]}},
             {"translate", toml::array{md_trans[0], md_trans[1], md_trans[2]}},
         }},
        {"metadata",
         toml::table{{"referenceSystem",
                      "https://www.opengis.net/def/crs/EPSG/0/7415"}}}};
    // serializing as JSON using toml::json_formatter:
    std::string metadata_json_file = fmt::format(
        fmt::runtime(metadata_json_file_spec), fmt::arg("path", output_path));

    // minimize json
    std::stringstream ss;
    ss << toml::json_formatter{metadatajson};
    auto s = ss.str();
    s.erase(std::remove(s.begin(), s.end(), '\n'), s.cend());
    s.erase(std::remove(s.begin(), s.end(), ' '), s.cend());

    std::ofstream ofs;
    ofs.open(metadata_json_file);
    ofs << s;
    ofs.close();
  }
  return EXIT_SUCCESS;
  ;
}
