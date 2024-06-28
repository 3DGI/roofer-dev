void crop_tile(const roofer::TBox<double>& tile,
               std::vector<InputPointcloud>& input_pointclouds,
               BuildingTile& output_building_tile, RooferConfig& cfg) {
  auto& logger = roofer::logger::Logger::get_logger();

  auto& pj = output_building_tile.proj_helper;
  auto vector_reader = roofer::io::createVectorReaderOGR(*pj);
  auto vector_writer = roofer::io::createVectorWriterOGR(*pj);
  vector_writer->srs = cfg.output_crs;
  auto PointCloudCropper = roofer::io::createPointCloudCropper(*pj);
  auto RasterWriter = roofer::io::createRasterWriterGDAL(*pj);
  auto vector_ops = roofer::misc::createVector2DOpsGEOS();
  auto LASWriter = roofer::io::createLASWriter(*pj);

  // logger.info("region_of_interest.has_value()? {}",
  // region_of_interest.has_value()); if(region_of_interest.has_value())
  vector_reader->open(cfg.source_footprints);
  vector_reader->region_of_interest = tile;
  std::vector<roofer::LinearRing> footprints;
  roofer::AttributeVecMap attributes;
  vector_reader->readPolygons(footprints, &attributes);

  const unsigned N_fp = footprints.size();

  // check if low_lod_attribute exists, if not then create it
  if (!attributes.get_if<bool>(cfg.low_lod_attribute)) {
    auto& vec = attributes.insert_vec<bool>(cfg.low_lod_attribute);
    vec.resize(N_fp, false);
  }
  auto low_lod_vec = attributes.get_if<bool>(cfg.low_lod_attribute);
  for (size_t i = 0; i < N_fp; ++i) {
    // need dereference operator here for dereferencing pointer and getting
    // std::option value
    (*low_lod_vec)[i] =
        *(*low_lod_vec)[i] ||
        std::fabs(footprints[i].signed_area()) > cfg.low_lod_area;
  }

  // get yoc attribute vector (nullptr if it does not exist)
  bool use_acquisition_year = true;
  auto yoc_vec = attributes.get_if<int>(cfg.year_of_construction_attribute);
  if (!yoc_vec) {
    use_acquisition_year = false;
    logger.warning(
        "year_of_construction_attribute '{}' not found in input footprints",
        cfg.year_of_construction_attribute);
  }

  // simplify + buffer footprints
  logger.info("Simplifying and buffering footprints...");
  vector_ops->simplify_polygons(footprints);
  auto buffered_footprints = footprints;
  vector_ops->buffer_polygons(buffered_footprints);

  // compute true extent that includes all buffered footprints
  roofer::Box polygon_extent;
  for (auto& buf_ring : buffered_footprints) {
    polygon_extent.add(buf_ring.box());
  }

  // transform back to input coordinates
  roofer::TBox<double> polygon_extent_untransformed;
  polygon_extent_untransformed.add(pj->coord_transform_rev(
      polygon_extent.pmin[0], polygon_extent.pmin[1], polygon_extent.pmin[2]));
  polygon_extent_untransformed.add(pj->coord_transform_rev(
      polygon_extent.pmax[0], polygon_extent.pmax[1], polygon_extent.pmax[2]));

  // Crop all pointclouds
  for (auto& ipc : input_pointclouds) {
    logger.info("Cropping pointcloud {}...", ipc.name);

    auto intersecting_files = ipc.rtree->query(polygon_extent_untransformed);

    std::vector<std::string> lasfiles;
    for (auto* file_extent_ : intersecting_files) {
      auto* file_extent = static_cast<fileExtent*>(file_extent_);
      lasfiles.push_back(file_extent->first);
    }

    PointCloudCropper->process(lasfiles, footprints, buffered_footprints,
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
    if (cfg.write_crop_outputs && cfg.write_index)
      ipc.nodata_circles.resize(N_fp);

    // auto& r_nodata = attributes.insert_vec<float>("r_nodata_"+ipc.name);
    roofer::arr2f nodata_c;
    for (unsigned i = 0; i < N_fp; ++i) {
      roofer::misc::RasterisePointcloud(ipc.building_clouds[i], footprints[i],
                                        ipc.building_rasters[i], cfg.cellsize);
      ipc.nodata_fractions[i] =
          roofer::misc::computeNoDataFraction(ipc.building_rasters[i]);
      ipc.pt_densities[i] =
          roofer::misc::computePointDensity(ipc.building_rasters[i]);

      auto target_density = cfg.max_point_density;
      bool low_lod = *(*low_lod_vec)[i];
      if (low_lod) {
        target_density = cfg.max_point_density_low_lod;
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
        // if (cfg.write_crop_outputs && cfg.write_index) {
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
  auto bid_vec = attributes.get_if<std::string>(cfg.building_bid_attribute);
  auto& pc_select = attributes.insert_vec<std::string>("pc_select");
  auto& pc_source = attributes.insert_vec<std::string>("pc_source");
  auto& pc_year = attributes.insert_vec<int>("pc_year");
  std::unordered_map<std::string, roofer::vec1s> jsonl_paths;
  std::string bid;
  bool only_write_selected = !cfg.output_all;
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

    // output to BuildingTile
    {
      BuildingObject& building = output_building_tile.buildings.emplace_back();
      building.attribute_index = i;

      building.pointcloud =
          input_pointclouds[selected->index].building_clouds[i];
      building.footprint = footprints[i];
      building.h_ground =
          input_pointclouds[selected->index].ground_elevations[i] +
          (*pj->data_offset)[2];

      output_building_tile.attributes = attributes;
      building.jsonl_path = fmt::format(
          fmt::runtime(cfg.building_jsonl_file_spec), fmt::arg("bid", bid),
          fmt::arg("pc_name", input_pointclouds[selected->index].name),
          fmt::arg("path", cfg.crop_output_path));
    }

    if (cfg.write_crop_outputs) {
      if (input_pointclouds[selected->index].force_low_lod) {
        (*low_lod_vec)[i] = true;
      }
      {
        // fs::create_directories(fs::path(fname).parent_path());
        std::string fp_path = fmt::format(
            fmt::runtime(cfg.building_gpkg_file_spec), fmt::arg("bid", bid),
            fmt::arg("path", cfg.crop_output_path));
        vector_writer->writePolygons(fp_path, footprints, attributes, i, i + 1);

        size_t j = 0;
        for (auto& ipc : input_pointclouds) {
          if ((selected->index != j) && (only_write_selected)) {
            ++j;
            continue;
          };

          std::string pc_path =
              fmt::format(fmt::runtime(cfg.building_las_file_spec),
                          fmt::arg("bid", bid), fmt::arg("pc_name", ipc.name),
                          fmt::arg("path", cfg.crop_output_path));
          std::string raster_path =
              fmt::format(fmt::runtime(cfg.building_raster_file_spec),
                          fmt::arg("bid", bid), fmt::arg("pc_name", ipc.name),
                          fmt::arg("path", cfg.crop_output_path));
          std::string jsonl_path =
              fmt::format(fmt::runtime(cfg.building_jsonl_file_spec),
                          fmt::arg("bid", bid), fmt::arg("pc_name", ipc.name),
                          fmt::arg("path", cfg.crop_output_path));

          if (cfg.write_rasters) {
            RasterWriter->writeBands(raster_path, ipc.building_rasters[i]);
          }

          LASWriter->write_pointcloud(input_pointclouds[j].building_clouds[i],
                                      pc_path);

          // Correct ground height for offset, NB this ignores crs
          // transformation
          double h_ground =
              input_pointclouds[j].ground_elevations[i] + (*pj->data_offset)[2];

          auto gf_config = toml::table{
              {"INPUT_FOOTPRINT", fp_path},
              // {"INPUT_POINTCLOUD", sresult.explanation ==
              // roofer::PointCloudSelectExplanation::_LATEST_BUT_OUTDATED ? ""
              // : pc_path},
              {"INPUT_POINTCLOUD", pc_path},
              {"BID", bid},
              {"GROUND_ELEVATION", h_ground},
              {"OUTPUT_JSONL", jsonl_path},

              {"GF_PROCESS_OFFSET_OVERRIDE", true},
              {"GF_PROCESS_OFFSET_X", (*pj->data_offset)[0]},
              {"GF_PROCESS_OFFSET_Y", (*pj->data_offset)[1]},
              {"GF_PROCESS_OFFSET_Z", (*pj->data_offset)[2]},
              {"skip_attribute_name", cfg.low_lod_attribute},
              {"id_attribute", cfg.building_bid_attribute},
          };

          if (cfg.write_metadata) {
            // gf_config.insert("GF_PROCESS_OFFSET_OVERRIDE", true);
            gf_config.insert("CITYJSON_TRANSLATE_X", (*pj->data_offset)[0]);
            gf_config.insert("CITYJSON_TRANSLATE_Y", (*pj->data_offset)[1]);
            gf_config.insert("CITYJSON_TRANSLATE_Z", (*pj->data_offset)[2]);
            gf_config.insert("CITYJSON_SCALE_X", 0.001);
            gf_config.insert("CITYJSON_SCALE_Y", 0.001);
            gf_config.insert("CITYJSON_SCALE_Z", 0.001);
          }
          // auto tbl_gfparams =
          //     config["output"]["reconstruction_parameters"].as_table();
          // gf_config.insert(tbl_gfparams->begin(), tbl_gfparams->end());

          if (!only_write_selected) {
            std::ofstream ofs;
            std::string config_path =
                fmt::format(fmt::runtime(cfg.building_toml_file_spec),
                            fmt::arg("bid", bid), fmt::arg("pc_name", ipc.name),
                            fmt::arg("path", cfg.crop_output_path));
            ofs.open(config_path);
            ofs << gf_config;
            ofs.close();

            jsonl_paths[ipc.name].push_back(jsonl_path);
          }
          if (selected->index == j) {
            // set optimal jsonl path
            std::string jsonl_path =
                fmt::format(fmt::runtime(cfg.building_jsonl_file_spec),
                            fmt::arg("bid", bid), fmt::arg("pc_name", ""),
                            fmt::arg("path", cfg.crop_output_path));
            gf_config.insert_or_assign("OUTPUT_JSONL", jsonl_path);
            jsonl_paths[""].push_back(jsonl_path);

            // write optimal config
            std::ofstream ofs;
            std::string config_path =
                fmt::format(fmt::runtime(cfg.building_toml_file_spec),
                            fmt::arg("bid", bid), fmt::arg("pc_name", ""),
                            fmt::arg("path", cfg.crop_output_path));
            ofs.open(config_path);
            ofs << gf_config;
            ofs.close();
          }
          ++j;
        }
      };
      // write config
    }
  }

  if (cfg.write_crop_outputs) {
    // Write index output
    if (cfg.write_index) {
      std::string index_file =
          fmt::format(fmt::runtime(cfg.index_file_spec),
                      fmt::arg("path", cfg.crop_output_path));
      vector_writer->writePolygons(index_file, footprints, attributes);

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
          std::string jsonl_list_file =
              fmt::format(fmt::runtime(cfg.jsonl_list_file_spec),
                          fmt::arg("path", cfg.crop_output_path),
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
    if (cfg.write_metadata) {
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
               {"translate",
                toml::array{md_trans[0], md_trans[1], md_trans[2]}},
           }},
          {"metadata",
           toml::table{{"referenceSystem",
                        "https://www.opengis.net/def/crs/EPSG/0/7415"}}}};
      // serializing as JSON using toml::json_formatter:
      std::string metadata_json_file =
          fmt::format(fmt::runtime(cfg.metadata_json_file_spec),
                      fmt::arg("path", cfg.crop_output_path));

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
  }
  // clear input_pointclouds data
  for (auto& ipc : input_pointclouds) {
    ipc.nodata_radii.clear();
    ipc.nodata_fractions.clear();
    ipc.pt_densities.clear();
    ipc.is_mutated.clear();
    ipc.nodata_circles.clear();
    ipc.building_clouds.clear();
    ipc.building_rasters.clear();
    ipc.ground_elevations.clear();
    ipc.acquisition_years.clear();
  }
}
