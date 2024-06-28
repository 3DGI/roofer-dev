
#include <cstddef>
#include <filesystem>
#include <string>

#include "argh.h"
#include "toml.hpp"
#ifdef RF_USE_RERUN
#include <rerun.hpp>
#endif

#include <roofer/io/PointCloudReader.hpp>
#include <roofer/io/VectorReader.hpp>
#include <roofer/misc/PC2MeshDistCalculator.hpp>
#include <roofer/misc/projHelper.hpp>
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
#ifdef RF_USE_VAL3DITY
#include <roofer/misc/Val3dator.hpp>
#endif
#include <roofer/logger/logger.h>

#include <roofer/io/CityJsonWriter.hpp>

#include "git.h"

namespace fs = std::filesystem;

#ifdef RF_USE_RERUN
// Adapters so we can log eigen vectors as rerun positions:
template <>
struct rerun::CollectionAdapter<rerun::Position3D, roofer::PointCollection> {
  /// Borrow for non-temporary.
  Collection<rerun::Position3D> operator()(
      const roofer::PointCollection& container) {
    return Collection<rerun::Position3D>::borrow(container.data(),
                                                 container.size());
  }

  // Do a full copy for temporaries (otherwise the data might be deleted when
  // the temporary is destroyed).
  Collection<rerun::Position3D> operator()(
      roofer::PointCollection&& container) {
    std::vector<rerun::Position3D> positions(container.size());
    memcpy(positions.data(), container.data(),
           container.size() * sizeof(roofer::arr3f));
    return Collection<rerun::Position3D>::take_ownership(std::move(positions));
  }
};
template <>
struct rerun::CollectionAdapter<rerun::Position3D, roofer::TriangleCollection> {
  /// Borrow for non-temporary.
  Collection<rerun::Position3D> operator()(
      const roofer::TriangleCollection& container) {
    return Collection<rerun::Position3D>::borrow(container[0].data(),
                                                 container.vertex_count());
  }

  // Do a full copy for temporaries (otherwise the data might be deleted when
  // the temporary is destroyed).
  Collection<rerun::Position3D> operator()(
      roofer::TriangleCollection&& container) {
    std::vector<rerun::Position3D> positions(container.size());
    memcpy(positions.data(), container[0].data(),
           container.vertex_count() * sizeof(roofer::arr3f));
    return Collection<rerun::Position3D>::take_ownership(std::move(positions));
  }
};
#endif

enum LOD { LOD12 = 12, LOD13 = 13, LOD22 = 22 };

std::unordered_map<int, roofer::Mesh> extrude(
    roofer::Arrangement_2 arrangement, float floor_elevation,
#ifdef RF_USE_VAL3DITY
    std::vector<std::optional<std::string> >& attr_val3dity,
#endif
    std::vector<std::optional<float> >& attr_rmse,
    roofer::reconstruction::SegmentRasteriserInterface* SegmentRasteriser,
    roofer::reconstruction::PlaneDetectorInterface* PlaneDetector, LOD lod) {
  bool dissolve_step_edges = false;
  bool dissolve_all_interior = false;
  bool extrude_LoD2 = true;
  if (lod == LOD12) {
    dissolve_all_interior = true, extrude_LoD2 = false;
  } else if (lod == LOD13) {
    dissolve_step_edges = true, extrude_LoD2 = false;
  }
#ifdef RF_USE_RERUN
  const auto& rec = rerun::RecordingStream::current();
#endif
  std::string worldname = fmt::format("world/lod{}/", (int)lod);

  auto& logger = roofer::logger::Logger::get_logger();

  auto ArrangementDissolver =
      roofer::reconstruction::createArrangementDissolver();
  ArrangementDissolver->compute(
      arrangement, SegmentRasteriser->heightfield,
      {.dissolve_step_edges = dissolve_step_edges,
       .dissolve_all_interior = dissolve_all_interior});
  logger.info("Completed ArrangementDissolver");
  logger.info("Roof partition has {} faces", arrangement.number_of_faces());
#ifdef RF_USE_RERUN
  rec.log(
      worldname + "ArrangementDissolver",
      rerun::LineStrips3D(roofer::reconstruction::arr2polygons(arrangement)));
#endif
  auto ArrangementSnapper = roofer::reconstruction::createArrangementSnapper();
  ArrangementSnapper->compute(arrangement);
  logger.info("Completed ArrangementSnapper");
#ifdef RF_USE_RERUN
// rec.log(worldname+"ArrangementSnapper", rerun::LineStrips3D(
// roofer::reconstruction::arr2polygons(arrangement) ));
#endif

  auto ArrangementExtruder =
      roofer::reconstruction::createArrangementExtruder();
  ArrangementExtruder->compute(arrangement, floor_elevation,
                               {.LoD2 = extrude_LoD2});
  logger.info("Completed ArrangementExtruder");
#ifdef RF_USE_RERUN
  rec.log(worldname + "ArrangementExtruder",
          rerun::LineStrips3D(ArrangementExtruder->faces)
              .with_class_ids(ArrangementExtruder->labels));
#endif

  auto MeshTriangulator =
      roofer::reconstruction::createMeshTriangulatorLegacy();
  MeshTriangulator->compute(ArrangementExtruder->multisolid);
  logger.info("Completed MeshTriangulator");
#ifdef RF_USE_RERUN
  rec.log(worldname + "MeshTriangulator",
          rerun::Mesh3D(MeshTriangulator->triangles)
              .with_vertex_normals(MeshTriangulator->normals)
              .with_class_ids(MeshTriangulator->ring_ids));
#endif

  auto PC2MeshDistCalculator = roofer::misc::createPC2MeshDistCalculator();
  PC2MeshDistCalculator->compute(PlaneDetector->pts_per_roofplane,
                                 MeshTriangulator->multitrianglecol,
                                 MeshTriangulator->ring_ids);
  attr_rmse.push_back(PC2MeshDistCalculator->rms_error);
  logger.info("Completed PC2MeshDistCalculator. RMSE={}",
              PC2MeshDistCalculator->rms_error);
#ifdef RF_USE_RERUN
// rec.log(worldname+"PC2MeshDistCalculator",
// rerun::Mesh3D(PC2MeshDistCalculator->triangles).with_vertex_normals(MeshTriangulator->normals).with_class_ids(MeshTriangulator->ring_ids));
#endif

#ifdef RF_USE_VAL3DITY
  auto Val3dator = roofer::reconstruction::createVal3dator();
  Val3dator->compute(ArrangementExtruder->multisolid);
  attr_val3dity.push_back(Val3dator->errors.front());
  logger.info("Completed Val3dator. Errors={}", Val3dator->errors.front());
#endif

  return ArrangementExtruder->multisolid;
}

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
}

void print_version() {
  std::cout << fmt::format(
      "roofer {} ({}{}{})\n", git_Describe(),
      std::strcmp(git_Branch(), "main") ? ""
                                        : fmt::format("{}, ", git_Branch()),
      git_AnyUncommittedChanges() ? "dirty, " : "", git_CommitDate());
}

// TODO
// [x] add cmd options for config and output
// [x] read toml config
// [x] parse toml config
// [x] handle kas_warenhuis==true case
// [x] LoD12, LoD13
// [x] write generated attributes to output as well (eg. val3dity_codes, rmse
// etc...) [x] handle no planes in input pc case. Write empty output?

int main(int argc, const char* argv[]) {
  auto cmdl = argh::parser({"-c", "--config"});

  cmdl.parse(argc, argv);
  std::string program_name = cmdl[0];

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

  std::string config_path;
  std::string path_pointcloud =
      "output/wippolder/objects/503100000030812/crop/"
      "503100000030812_pointcloud.las";
  std::string path_footprint =
      "output/wippolder/objects/503100000030812/crop/503100000030812.gpkg";
  std::string path_output_jsonl = "output/output.city.jsonl";
  std::string crs_process = "EPSG:7415";
  std::string crs_output = "EPSG:7415";
  std::string skip_attribute_name = "kas_warenhuis";
  std::string building_bid_attribute = "CCA";
  float offset_x = 85373.406000000003;
  float offset_y = 447090.51799999998;
  float offset_z = 0;
  float CITYJSON_TRANSLATE_X = offset_x;
  float CITYJSON_TRANSLATE_Y = offset_y;
  float CITYJSON_TRANSLATE_Z = offset_z;
  float CITYJSON_SCALE_X = 0.01;
  float CITYJSON_SCALE_Y = 0.01;
  float CITYJSON_SCALE_Z = 0.01;
  float floor_elevation = -0.16899998486042023;
  size_t fp_i = 0;

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

    auto tml_path_footprint = config["INPUT_FOOTPRINT"].value<std::string>();
    if (tml_path_footprint.has_value()) path_footprint = *tml_path_footprint;

    auto tml_path_pointcloud = config["INPUT_POINTCLOUD"].value<std::string>();
    if (tml_path_pointcloud.has_value()) path_pointcloud = *tml_path_pointcloud;

    auto tml_building_bid_attribute =
        config["id_attribute"].value<std::string>();
    if (tml_building_bid_attribute.has_value())
      building_bid_attribute = *tml_building_bid_attribute;

    auto tml_path_output_jsonl = config["OUTPUT_JSONL"].value<std::string>();
    if (tml_path_output_jsonl.has_value())
      path_output_jsonl = *tml_path_output_jsonl;

    auto tml_crs_process = config["GF_PROCESS_CRS"].value<std::string>();
    if (tml_crs_process.has_value()) crs_process = *tml_crs_process;

    auto tml_crs_output = config["GF_OUTPUT_CRS"].value<std::string>();
    if (tml_crs_output.has_value()) crs_output = *tml_crs_output;

    auto tml_floor_elevation = config["GROUND_ELEVATION"].value<float>();
    if (tml_floor_elevation.has_value()) floor_elevation = *tml_floor_elevation;

    auto tml_offset_x = config["GF_PROCESS_OFFSET_X"].value<float>();
    if (tml_offset_x.has_value()) offset_x = *tml_offset_x;

    auto tml_offset_y = config["GF_PROCESS_OFFSET_Y"].value<float>();
    if (tml_offset_y.has_value()) offset_y = *tml_offset_y;

    auto tml_offset_z = config["GF_PROCESS_OFFSET_Z"].value<float>();
    if (tml_offset_z.has_value()) offset_z = *tml_offset_z;

    auto tml_CITYJSON_TRANSLATE_X =
        config["CITYJSON_TRANSLATE_X"].value<float>();
    if (tml_CITYJSON_TRANSLATE_X.has_value())
      CITYJSON_TRANSLATE_X = *tml_CITYJSON_TRANSLATE_X;

    auto tml_CITYJSON_TRANSLATE_Y =
        config["CITYJSON_TRANSLATE_Y"].value<float>();
    if (tml_CITYJSON_TRANSLATE_Y.has_value())
      CITYJSON_TRANSLATE_Y = *tml_CITYJSON_TRANSLATE_Y;

    auto tml_CITYJSON_TRANSLATE_Z =
        config["CITYJSON_TRANSLATE_Z"].value<float>();
    if (tml_CITYJSON_TRANSLATE_Z.has_value())
      CITYJSON_TRANSLATE_Z = *tml_CITYJSON_TRANSLATE_Z;

    auto tml_CITYJSON_SCALE_X = config["CITYJSON_SCALE_X"].value<float>();
    if (tml_CITYJSON_SCALE_X.has_value())
      CITYJSON_SCALE_X = *tml_CITYJSON_SCALE_X;

    auto tml_CITYJSON_SCALE_Y = config["CITYJSON_SCALE_Y"].value<float>();
    if (tml_CITYJSON_SCALE_Y.has_value())
      CITYJSON_SCALE_Y = *tml_CITYJSON_SCALE_Y;

    auto tml_CITYJSON_SCALE_Z = config["CITYJSON_SCALE_Z"].value<float>();
    if (tml_CITYJSON_SCALE_Z.has_value())
      CITYJSON_SCALE_Z = *tml_CITYJSON_SCALE_Z;

    auto tml_skip_attribute_name = config["skip_attribute_name"].value<float>();
    if (tml_skip_attribute_name.has_value())
      skip_attribute_name = *tml_skip_attribute_name;

  } else {
    logger.error("No config file specified\n");
    return EXIT_FAILURE;
  }

  // Create Writer. TODO: check if we can write to output file prior to doing
  // reconstruction?
  auto pj = roofer::misc::createProjHelper();
  auto CityJsonWriter = roofer::io::createCityJsonWriter(*pj);
  CityJsonWriter->CRS_ = crs_output;
  CityJsonWriter->identifier_attribute = building_bid_attribute;
  CityJsonWriter->translate_x_ = CITYJSON_TRANSLATE_X;
  CityJsonWriter->translate_y_ = CITYJSON_TRANSLATE_Y;
  CityJsonWriter->translate_z_ = CITYJSON_TRANSLATE_Z;
  CityJsonWriter->scale_x_ = CITYJSON_SCALE_X;
  CityJsonWriter->scale_y_ = CITYJSON_SCALE_Y;
  CityJsonWriter->scale_z_ = CITYJSON_SCALE_Z;

  // read inputs
  pj->set_process_crs(crs_process.c_str());
  roofer::arr3d offset = {offset_x, offset_y, offset_z};
  pj->set_data_offset(offset);
  auto PointReader = roofer::io::createPointCloudReaderLASlib(*pj);
  auto VectorReader = roofer::io::createVectorReaderOGR(*pj);

  VectorReader->open(path_footprint);
  std::vector<roofer::LinearRing> footprints;
  roofer::AttributeVecMap attributes;
  VectorReader->readPolygons(footprints, &attributes);

  PointReader->open(path_pointcloud);
  logger.info("Reading pointcloud from {}", path_pointcloud);
  roofer::vec1i classification;
  roofer::PointCollection points, points_ground, points_roof;
  PointReader->readPointCloud(points, &classification);
  logger.info("Read {} points", points.size());

  // split into ground and roof points
  for (size_t i = 0; i < points.size(); ++i) {
    if (2 == classification[i]) {
      points_ground.push_back(points[i]);
    } else if (6 == classification[i]) {
      points_roof.push_back(points[i]);
    }
  }
  logger.info("{} ground points and {} roof points", points_ground.size(),
              points_roof.size());

#ifdef RF_USE_RERUN
  // Create a new `RecordingStream` which sends data over TCP to the viewer
  // process.
  const auto rec = rerun::RecordingStream("Roofer rerun test");
  // Try to spawn a new viewer instance.
  rec.spawn().exit_on_failure();
  rec.set_global();
#endif

#ifdef RF_USE_RERUN
  rec.log("world/raw_points",
          rerun::Collection{rerun::components::AnnotationContext{
              rerun::datatypes::AnnotationInfo(
                  6, "BUILDING", rerun::datatypes::Rgba32(255, 0, 0)),
              rerun::datatypes::AnnotationInfo(2, "GROUND"),
              rerun::datatypes::AnnotationInfo(1, "UNCLASSIFIED"),
          }});
  rec.log("world/raw_points",
          rerun::Points3D(points).with_class_ids(classification));
#endif

  auto PlaneDetector = roofer::reconstruction::createPlaneDetector();
  PlaneDetector->detect(points_roof);
  logger.info("Completed PlaneDetector (roof), found {} roofplanes",
              PlaneDetector->pts_per_roofplane.size());

  auto& attr_roof_type = attributes.insert_vec<std::string>("b3_dak_type");
  attr_roof_type.push_back(PlaneDetector->roof_type);
  auto& attr_roof_elevation_50p = attributes.insert_vec<float>("b3_h_dak_50p");
  attr_roof_elevation_50p.push_back(PlaneDetector->roof_elevation_50p);
  auto& attr_roof_elevation_70p = attributes.insert_vec<float>("b3_h_dak_70p");
  attr_roof_elevation_70p.push_back(PlaneDetector->roof_elevation_70p);
  auto& attr_roof_elevation_min = attributes.insert_vec<float>("b3_h_dak_min");
  attr_roof_elevation_min.push_back(PlaneDetector->roof_elevation_min);
  auto& attr_roof_elevation_max = attributes.insert_vec<float>("b3_h_dak_max");
  attr_roof_elevation_max.push_back(PlaneDetector->roof_elevation_max);

  auto& attr_skip = attributes.insert_vec<bool>("b3_reconstructie_onvolledig");

  bool pointcloud_insufficient = PlaneDetector->roof_type == "no points" ||
                                 PlaneDetector->roof_type == "no planes";
  if (pointcloud_insufficient) {
    attr_skip.push_back(true);
    CityJsonWriter->write(path_output_jsonl, footprints[fp_i], nullptr, nullptr,
                          nullptr, attributes, fp_i);
  }

  auto PlaneDetector_ground = roofer::reconstruction::createPlaneDetector();
  PlaneDetector_ground->detect(points_ground);
  logger.info("Completed PlaneDetector (ground), found {} groundplanes",
              PlaneDetector_ground->pts_per_roofplane.size());

#ifdef RF_USE_RERUN
  rec.log("world/segmented_points",
          rerun::Collection{rerun::components::AnnotationContext{
              rerun::datatypes::AnnotationInfo(
                  0, "no plane", rerun::datatypes::Rgba32(30, 30, 30))}});
  rec.log("world/segmented_points",
          rerun::Points3D(points_roof).with_class_ids(PlaneDetector->plane_id));
#endif

  // check skip_attribute
  bool skip = false;
  if (auto vec = attributes.get_if<bool>(skip_attribute_name)) {
    if ((*vec)[fp_i].has_value()) {
      skip = (*vec)[fp_i].value();
    }
  }
  // skip = skip || no_planes;
  logger.info("Skip = {}", skip);
  attr_skip.push_back(skip);

  if (skip) {
    auto SimplePolygonExtruder =
        roofer::reconstruction::createSimplePolygonExtruder();
    SimplePolygonExtruder->compute(footprints[fp_i], floor_elevation,
                                   PlaneDetector->roof_elevation_70p);
    CityJsonWriter->write(path_output_jsonl, footprints[fp_i], &SimplePolygonExtruder->multisolid,
                          &SimplePolygonExtruder->multisolid, &SimplePolygonExtruder->multisolid, attributes, fp_i);
    logger.info("Completed CityJsonWriter to {}", path_output_jsonl);
  } else {
    auto AlphaShaper = roofer::reconstruction::createAlphaShaper();
    AlphaShaper->compute(PlaneDetector->pts_per_roofplane);
    logger.info("Completed AlphaShaper (roof), found {} rings, {} labels",
                AlphaShaper->alpha_rings.size(),
                AlphaShaper->roofplane_ids.size());
#ifdef RF_USE_RERUN
    rec.log("world/alpha_rings_roof",
            rerun::LineStrips3D(AlphaShaper->alpha_rings)
                .with_class_ids(AlphaShaper->roofplane_ids));
#endif

    auto AlphaShaper_ground = roofer::reconstruction::createAlphaShaper();
    AlphaShaper_ground->compute(PlaneDetector_ground->pts_per_roofplane);
    logger.info("Completed AlphaShaper (ground), found {} rings, {} labels",
                AlphaShaper_ground->alpha_rings.size(),
                AlphaShaper_ground->roofplane_ids.size());
#ifdef RF_USE_RERUN
    rec.log("world/alpha_rings_ground",
            rerun::LineStrips3D(AlphaShaper_ground->alpha_rings)
                .with_class_ids(AlphaShaper_ground->roofplane_ids));
#endif

    auto LineDetector = roofer::reconstruction::createLineDetector();
    LineDetector->detect(AlphaShaper->alpha_rings, AlphaShaper->roofplane_ids,
                         PlaneDetector->pts_per_roofplane);
    logger.info("Completed LineDetector");
#ifdef RF_USE_RERUN
    rec.log("world/boundary_lines",
            rerun::LineStrips3D(LineDetector->edge_segments));
#endif

    auto PlaneIntersector = roofer::reconstruction::createPlaneIntersector();
    PlaneIntersector->compute(PlaneDetector->pts_per_roofplane,
                              PlaneDetector->plane_adjacencies);
    logger.info("Completed PlaneIntersector");
#ifdef RF_USE_RERUN
    rec.log("world/intersection_lines",
            rerun::LineStrips3D(PlaneIntersector->segments));
#endif

    auto LineRegulariser = roofer::reconstruction::createLineRegulariser();
    LineRegulariser->compute(LineDetector->edge_segments,
                             PlaneIntersector->segments);
    logger.info("Completed LineRegulariser");
#ifdef RF_USE_RERUN
    rec.log("world/regularised_lines",
            rerun::LineStrips3D(LineRegulariser->regularised_edges));
#endif

    auto SegmentRasteriser = roofer::reconstruction::createSegmentRasteriser();
    SegmentRasteriser->compute(AlphaShaper->alpha_triangles,
                               AlphaShaper_ground->alpha_triangles);
    logger.info("Completed SegmentRasteriser");

    auto heightfield_copy = SegmentRasteriser->heightfield;
    heightfield_copy.set_nodata(0);
#ifdef RF_USE_RERUN
    rec.log("world/heightfield",
            rerun::DepthImage({heightfield_copy.dimy_, heightfield_copy.dimx_},
                              *heightfield_copy.vals_));
#endif

    roofer::Arrangement_2 arrangement;
    auto ArrangementBuilder =
        roofer::reconstruction::createArrangementBuilder();
    ArrangementBuilder->compute(arrangement, footprints[fp_i],
                                LineRegulariser->exact_regularised_edges);
    logger.info("Completed ArrangementBuilder");
    logger.info("Roof partition has {} faces", arrangement.number_of_faces());
#ifdef RF_USE_RERUN
    rec.log(
        "world/initial_partition",
        rerun::LineStrips3D(roofer::reconstruction::arr2polygons(arrangement)));
#endif

    auto ArrangementOptimiser =
        roofer::reconstruction::createArrangementOptimiser();
    ArrangementOptimiser->compute(arrangement, SegmentRasteriser->heightfield,
                                  PlaneDetector->pts_per_roofplane,
                                  PlaneDetector_ground->pts_per_roofplane);
    logger.info("Completed ArrangementOptimiser");
// rec.log("world/optimised_partition", rerun::LineStrips3D(
// roofer::reconstruction::arr2polygons(arrangement) ));

// LoDs
// attributes to be filled during reconstruction
#ifdef RF_USE_VAL3DITY
    auto& attr_val3dity_lod12 =
        attributes.insert_vec<std::string>("b3_val3dity_lod12");
#endif
    auto& attr_rmse_lod12 = attributes.insert_vec<float>("b3_rmse_lod12");
    auto multisolids_lod12 = extrude(arrangement, floor_elevation,
#ifdef RF_USE_VAL3DITY
                                     attr_val3dity_lod12,
#endif
                                     attr_rmse_lod12, SegmentRasteriser.get(),
                                     PlaneDetector.get(), LOD12);
#ifdef RF_USE_VAL3DITY
    auto& attr_val3dity_lod13 =
        attributes.insert_vec<std::string>("b3_val3dity_lod13");
#endif
    auto& attr_rmse_lod13 = attributes.insert_vec<float>("b3_rmse_lod13");
    auto multisolids_lod13 = extrude(arrangement, floor_elevation,
#ifdef RF_USE_VAL3DITY
                                     attr_val3dity_lod13,
#endif
                                     attr_rmse_lod13, SegmentRasteriser.get(),
                                     PlaneDetector.get(), LOD13);
#ifdef RF_USE_VAL3DITY
    auto& attr_val3dity_lod22 =
        attributes.insert_vec<std::string>("b3_val3dity_lod22");
#endif
    auto& attr_rmse_lod22 = attributes.insert_vec<float>("b3_rmse_lod22");
    auto multisolids_lod22 = extrude(arrangement, floor_elevation,
#ifdef RF_USE_VAL3DITY
                                     attr_val3dity_lod22,
#endif
                                     attr_rmse_lod22, SegmentRasteriser.get(),
                                     PlaneDetector.get(), LOD22);

    CityJsonWriter->write(path_output_jsonl, footprints[0], &multisolids_lod12,
                          &multisolids_lod13, &multisolids_lod22, attributes, fp_i);
    logger.info("Completed CityJsonWriter to {}", path_output_jsonl);
  }
  // end LoD2
}
