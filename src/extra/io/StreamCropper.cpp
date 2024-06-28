#include <roofer/logger/logger.h>

#include <bitset>
#include <ctime>
#include <filesystem>
#include <lasreader.hpp>
#include <roofer/common/Raster.hpp>
#include <roofer/common/pip_util.hpp>
#include <roofer/io/StreamCropper.hpp>

namespace roofer::io {

  namespace fs = std::filesystem;

  class PointsInPolygonsCollector {
    std::vector<LinearRing>& polygons;
    std::vector<PointCollection>& point_clouds;
    vec1f& ground_elevations;
    vec1i& acquisition_years;

    // ground elevations
    std::vector<std::vector<arr3f>> ground_buffer_points;
    RasterTools::Raster pindex;
    std::vector<std::vector<size_t>> pindex_vals;
    std::vector<pGridSet> poly_grids, buf_poly_grids;
    std::vector<vec1f> z_ground;
    std::unordered_map<std::unique_ptr<arr3f>, std::vector<size_t>>
        points_overlap;  // point, [poly id's], these are points that intersect
                         // with multiple polygons

    int ground_class, building_class;
    bool handle_overlap_points;

   public:
    float min_ground_elevation = std::numeric_limits<float>::max();

    PointsInPolygonsCollector(std::vector<LinearRing>& polygons,
                              std::vector<LinearRing>& buf_polygons,
                              std::vector<PointCollection>& point_clouds,
                              vec1f& ground_elevations,
                              vec1i& acquisition_years,
                              const Box& completearea_bb, float& cellsize,
                              float& buffer, int ground_class = 2,
                              int building_class = 6,
                              bool handle_overlap_points = false  // ·
                              )
        : polygons(polygons),
          point_clouds(point_clouds),
          ground_elevations(ground_elevations),
          ground_class(ground_class),
          building_class(building_class),
          acquisition_years(acquisition_years),
          handle_overlap_points(handle_overlap_points) {
      // point_clouds_ground.resize(polygons.size());
      point_clouds.resize(polygons.size());
      z_ground.resize(polygons.size());
      ground_buffer_points.resize(polygons.size());
      acquisition_years.resize(polygons.size(), 0);

      for (size_t i = 0; i < point_clouds.size(); ++i) {
        point_clouds.at(i).attributes.insert_vec<int>("classification");
      }

      // make a vector of BOX2D for the set of input polygons
      // build point in polygon grids
      for (size_t i = 0; i < polygons.size(); ++i) {
        auto ring = polygons.at(i);
        auto buf_ring = buf_polygons.at(i);
        poly_grids.push_back(build_grid(ring));
        buf_poly_grids.push_back(build_grid(buf_ring));
      }

      // build an index grid for the polygons

      // build raster index (pindex) and store for each raster cell all the
      // polygons with intersecting bbox (pindex_vals)
      float minx = completearea_bb.min()[0] - buffer;
      float miny = completearea_bb.min()[1] - buffer;
      float maxx = completearea_bb.max()[0] + buffer;
      float maxy = completearea_bb.max()[1] + buffer;
      pindex = RasterTools::Raster(cellsize, minx, maxx, miny, maxy);
      pindex_vals.resize(pindex.dimx_ * pindex.dimy_);

      // populate pindex_vals
      for (size_t i = 0; i < buf_polygons.size(); ++i) {
        auto ring = buf_polygons.at(i);
        auto& b = ring.box();
        size_t r_min = pindex.getRow(b.min()[0], b.min()[1]);
        size_t c_min = pindex.getCol(b.min()[0], b.min()[1]);
        size_t r_max = static_cast<size_t>(ceil(b.size_y() / cellsize)) + r_min;
        size_t c_max = static_cast<size_t>(ceil(b.size_x() / cellsize)) + c_min;
        if (r_max >= pindex.dimy_) r_max = pindex.dimy_ - 1;
        if (c_max >= pindex.dimx_) c_max = pindex.dimx_ - 1;
        for (size_t r = r_min; r <= r_max; ++r) {
          for (size_t c = c_min; c <= c_max; ++c) {
            pindex_vals[r * pindex.dimx_ + c].push_back(i);
          }
        }
      }
    }

    ~PointsInPolygonsCollector() {
      for (int i = 0; i < poly_grids.size(); i++) {
        delete poly_grids[i];
        delete buf_poly_grids[i];
      }
    }

    /**
     * @brief Find polygon that intersects point and add the point to the
     * corresponding building point cloud
     */
    void add_point(arr3f point, int point_class, int acqusition_year) {
      // look up grid index cell and do pip for all polygons retreived from that
      // cell
      size_t lincoord = pindex.getLinearCoord(point[0], point[1]);
      if (lincoord >= pindex_vals.size() || lincoord < 0) {
        // std::cout << "Point (" << point[0] << ", " <<point[1] << ", "  <<
        // point[2] << ") is not in the polygon bbox.\n";
        return;
      }
      // For the ground points we only test if the point is within the grid
      // index cell, but we do not do a pip for the footprint itself.
      // The reasons for this:
      //  - If we would do a pip for with the ground points, we would need to
      //    have a separate list of buffered footprints that we use for the
      //    ground pip. Still it would be often the case that there are no
      //    ground points found for a buffered footprint. Thus we need a larger
      //    area in which we can guarantee that we find at least a couple of
      //    ground points.
      //  - Because we work with Netherlands data, the ground relief is small.
      //    Thus a single ground height value per grid cell is good enough for
      //    representing the ground/floor elevation of the buildings in that
      //    grid cell.
      pPipoint pipoint = new Pipoint{point[0], point[1]};
      std::vector<size_t> poly_intersect;
      for (size_t& poly_i : pindex_vals[lincoord]) {
        if (GridTest(buf_poly_grids[poly_i], pipoint)) {
          auto& point_cloud = point_clouds.at(poly_i);
          auto classification =
              point_cloud.attributes.get_if<int>("classification");

          if (point_class == ground_class) {
            min_ground_elevation = std::min(min_ground_elevation, point[2]);
            z_ground[poly_i].push_back(point[2]);
          }

          if (GridTest(poly_grids[poly_i], pipoint)) {
            if (point_class == ground_class) {
              point_cloud.push_back(point);
              (*classification).push_back(2);
            } else if (point_class == building_class) {
              poly_intersect.push_back(poly_i);
            }
            acquisition_years[poly_i] =
                std::max(acqusition_year, acquisition_years[poly_i]);
          } else if (point_class == ground_class) {
            ground_buffer_points[poly_i].push_back(point);
          }
        }
      }

      if (point_class == building_class) {
        if (poly_intersect.size() > 1 && handle_overlap_points) {
          // decide later to which polygon to assign this point to
          points_overlap[std::make_unique<arr3f>(point)] = poly_intersect;
        } else {
          // assign point to all intersecting polygons
          for (auto& poly_i : poly_intersect) {
            auto& point_cloud = point_clouds.at(poly_i);
            auto classification =
                point_cloud.attributes.get_if<int>("classification");
            point_cloud.push_back(point);
            (*classification).push_back(6);
          }
        }
      }
      delete pipoint;
    }

    /**
     * @brief Post processes building objects after adding all points.
     *  - compute building polygon area
     *  - compute building elevation value for building
     *  - compute point counts for ground and building classification
     *  - compute ground elevation value for building
     *  - clear point clouds with very low coverage (ie. underground footprints)
     *    cov_thres = mean_density - coverage_threshold * std_dev_density
     */
    void do_post_process(float& ground_percentile, float& max_density_delta,
                         float& coverage_threshold, bool& clear_if_insufficient,
                         vec1f& poly_areas, vec1i& poly_pt_counts_bld,
                         vec1i& poly_pt_counts_grd,
                         vec1s& poly_ptcoverage_class, vec1f& poly_densities) {
      // compute poly properties
      struct PolyInfo {
        size_t pt_count_bld;
        size_t pt_count_grd;
        size_t pt_count_bld_overlap{0};
        float avg_elevation;
        float area;
      };
      std::unordered_map<size_t, PolyInfo> poly_info;

      auto& logger = logger::Logger::get_logger();

      // compute totals and statistics about pointcloud for each polygon
      // - polygon area
      // - count of ground points
      // - count of building points
      // - average elevation of building points
      for (size_t poly_i = 0; poly_i < polygons.size(); poly_i++) {
        auto& polygon = polygons.at(poly_i);
        auto& point_cloud = point_clouds.at(poly_i);
        auto classification =
            point_cloud.attributes.get_if<int>("classification");
        PolyInfo info;

        info.area = polygon.signed_area();
        size_t pt_cnt_bld = 0;
        size_t pt_cnt_grd = 0;
        float z_sum = 0;
        for (size_t pi = 0; pi < point_cloud.size(); ++pi) {
          if ((*classification)[pi] == 6) {
            ++pt_cnt_bld;
            z_sum += point_cloud[pi][2];
          } else if ((*classification)[pi] == 2) {
            ++pt_cnt_grd;
          }
        }
        info.pt_count_bld = pt_cnt_bld;
        info.pt_count_grd = pt_cnt_grd;
        if (info.pt_count_bld > 0) {
          info.avg_elevation = z_sum / pt_cnt_bld;
        }
        poly_info.insert({poly_i, info});
      }

      // merge buffer ground points into regular point_clouds now that the
      // proper counts have been established
      for (size_t poly_i; poly_i < polygons.size(); poly_i++) {
        auto& point_cloud = point_clouds.at(poly_i);
        auto classification =
            point_cloud.attributes.get_if<int>("classification");
        for (auto& p : ground_buffer_points[poly_i]) {
          point_cloud.push_back(p);
          (*classification).push_back(2);
        }
      }
      ground_buffer_points.clear();

      // assign points_overlap
      if (handle_overlap_points) {
        for (auto& [p, polylist] : points_overlap) {
          for (auto& poly_i : polylist) {
            poly_info[poly_i].pt_count_bld_overlap++;
          }
        }
        for (auto& [p, polylist] : points_overlap) {
          // find best polygon to assign this point to
          std::sort(polylist.begin(), polylist.end(),
                    [&max_density_delta, &poly_info, this](auto& d1, auto& d2) {
                      // we look at the maximim possible point density (proxy
                      // for point coverage) and the average elevation compute
                      // poitncloud density for both polygons
                      float pd1 = (poly_info[d1].pt_count_bld +
                                   poly_info[d1].pt_count_bld_overlap) /
                                  poly_info[d1].area;
                      float pd2 = (poly_info[d2].pt_count_bld +
                                   poly_info[d2].pt_count_bld_overlap) /
                                  poly_info[d2].area;

                      // check if the difference in point densities is less than
                      // 5%
                      if (std::abs(1 - pd1 / pd2) < max_density_delta) {
                        // if true, then look at the polygon with the highest
                        // elevation point cloud
                        return poly_info[d1].avg_elevation <
                               poly_info[d2].avg_elevation;
                      } else {
                        // otherwise decide based on the density values
                        return pd1 < pd2;
                      }
                    });

          // now the most suitable polygon (footprint) is the last in the list.
          // We will assign this point to that footprint.
          auto& point_cloud = point_clouds.at(polylist.back());
          auto classification =
              point_cloud.attributes.get_if<int>("classification");
          point_cloud.push_back(*p);
          (*classification).push_back(6);
          poly_info[polylist.back()].pt_count_bld++;
        }
      }

      // Compute ground elevation per polygon (eg 5th percentile of all ground
      // pts) std::cout <<"Computing the average ground elevation per
      // polygon..." << std::endl;
      for (size_t i = 0; i < z_ground.size(); ++i) {
        float ground_ele = min_ground_elevation;
        if (z_ground[i].size() != 0) {
          std::sort(z_ground[i].begin(), z_ground[i].end(),
                    [](auto& z1, auto& z2) { return z1 < z2; });
          int elevation_id =
              std::floor(ground_percentile * float(z_ground[i].size() - 1));
          ground_ele = z_ground[i][elevation_id];
        } else {
          // spdlog::info("no ground pts found for polygon");
        }
        // Assign the median ground elevation to each polygon
        ground_elevations.push_back(ground_ele);
      }

      // clear footprints with very low coverage (ie. underground footprints)
      // TODO: improve method for computing mean_density
      float total_cnt = 0, total_area = 0;
      for (auto& [poly_i, info] : poly_info) {
        total_cnt += info.pt_count_bld + info.pt_count_grd;
        total_area += info.area;
      }
      float mean_density = total_cnt / total_area;
      float diff_sum = 0;
      for (auto& [poly_i, info] : poly_info) {
        diff_sum += std::pow(mean_density - (info.pt_count_bld / info.area), 2);
      }
      float std_dev_density = std::sqrt(diff_sum / poly_info.size());
      logger.info("Mean point density = {}", mean_density);
      logger.info("Standard deviation = {}", std_dev_density);

      float cov_thres = mean_density - coverage_threshold * std_dev_density;
      for (size_t poly_i = 0; poly_i < polygons.size(); ++poly_i) {
        auto& info = poly_info[poly_i];

        if ((info.pt_count_bld / info.area) < cov_thres) {
          if (clear_if_insufficient) {
            auto& point_cloud = point_clouds.at(poly_i);
            point_cloud.clear();
          }
          poly_ptcoverage_class.push_back(std::string("insufficient"));
        } else {
          poly_ptcoverage_class.push_back(std::string("sufficient"));
        }
        // info.pt_count = point_cloud.size();
        poly_areas.push_back(float(info.area));
        poly_pt_counts_bld.push_back(int(info.pt_count_bld));
        poly_pt_counts_grd.push_back(int(info.pt_count_grd));
        poly_densities.push_back(float(info.pt_count_bld / info.area));
      }
    }
  };

  // Get the year-part from the GPS time of the point.
  // Assumes that the GPS time is Adjusted Standard GPS Time.
  int getAcquisitionYearOfPoint(LASpoint* laspoint) {
    // Adjusted Standard GPS Time
    const auto adjusted_gps_time = (double)laspoint->get_gps_time();
    // GPS epoch - UNIX epoch + 10^9
    // -10^9 is the adjustment in the gps time to lower the value
    auto gps_time_unix_epoch = (std::time_t)(adjusted_gps_time + 1315964800.0);
    auto lt = std::gmtime(&gps_time_unix_epoch);
    // spdlog::info("{}-{}-{}", lt->tm_year + 1900, lt->tm_mon, lt->tm_mday);
    return lt->tm_year + 1900;
  }

  // If GPS Week Time is used on the points, then we use the 'file creation
  // year' as acquisition year.
  bool useFileCreationYear(LASreader* lasreader) {
    typedef std::bitset<sizeof(uint16_t)> GlobalEncodingBits;
    auto& logger = logger::Logger::get_logger();
    // Table 4 in
    // https://www.asprs.org/wp-content/uploads/2010/12/LAS_1_4_r13.pdf
    bool gps_standard_time =
        GlobalEncodingBits(lasreader->header.global_encoding).test(0);
    if (gps_standard_time) {
      return false;
    } else {
      logger.info(
          "No good GPS time available, defaulting to file creation year");
      // If it is GPS Week Time, probably could handle this better here, but I'm
      // simplifying. Also, AHN3 for instance uses week time, but there is no
      // way of knowing which week is date...
      // https://geoforum.nl/t/ahn3-datum-tijd-uit-gps-tijd-converteren/3476
      // https://geoforum.nl/t/ahn3-verschillende-gps-time-types/3536
      return true;
    }
  }

  void getOgcWkt(LASheader* lasheader, std::string& wkt) {
    auto& logger = logger::Logger::get_logger();
    for (int i = 0; i < (int)lasheader->number_of_variable_length_records;
         i++) {
      if (lasheader->vlrs[i].record_id == 2111)  // OGC MATH TRANSFORM WKT
      {
        logger.info("Found and ignored: OGC MATH TRANSFORM WKT");
      } else if (lasheader->vlrs[i].record_id ==
                 2112)  // OGC COORDINATE SYSTEM WKT
      {
        logger.info("Found: OGC COORDINATE SYSTEM WKT");
        wkt = (char*)(lasheader->vlrs[i].data);
      } else if (lasheader->vlrs[i].record_id == 34735)  // GeoKeyDirectoryTag
      {
        logger.info("Found and ignored: GeoKeyDirectoryTag");
      }
    }

    for (int i = 0;
         i < (int)lasheader->number_of_extended_variable_length_records; i++) {
      if (strcmp(lasheader->evlrs[i].user_id, "LASF_Projection") == 0) {
        if (lasheader->evlrs[i].record_id == 2111)  // OGC MATH TRANSFORM WKT
        {
          logger.info("Found and ignored: OGC MATH TRANSFORM WKT");

        } else if (lasheader->evlrs[i].record_id ==
                   2112)  // OGC COORDINATE SYSTEM WKT
        {
          logger.info("Found: OGC COORDINATE SYSTEM WKT");
          wkt = (char*)(lasheader->evlrs[i].data);
        }
      }
    }
    // std::cout << wkt << std::endl;
  };

  struct PointCloudCropper : public PointCloudCropperInterface {
    using PointCloudCropperInterface::PointCloudCropperInterface;

    void process(const std::vector<std::string>& lasfiles,
                 std::vector<LinearRing>& polygons,
                 std::vector<LinearRing>& buf_polygons,
                 std::vector<PointCollection>& point_clouds,
                 vec1f& ground_elevations, vec1i& acquisition_years,
                 const Box& polygon_extent, PointCloudCropperConfig cfg) {
      // vec1f ground_elevations;
      vec1f poly_areas;
      vec1i poly_pt_counts_bld;
      vec1i poly_pt_counts_grd;
      vec1s poly_ptcoverage_class;
      vec1f poly_densities;

      auto& logger = logger::Logger::get_logger();

      PointsInPolygonsCollector pip_collector{polygons,
                                              buf_polygons,
                                              point_clouds,
                                              ground_elevations,
                                              acquisition_years,
                                              polygon_extent,
                                              cfg.cellsize,
                                              cfg.buffer,
                                              cfg.ground_class,
                                              cfg.building_class,
                                              cfg.handle_overlap_points};

      for (auto lasfile : lasfiles) {
        LASreadOpener lasreadopener;
        lasreadopener.set_file_name(lasfile.c_str());
        LASreader* lasreader = lasreadopener.open();

        std::string wkt = cfg.wkt_;
        if (wkt.size() == 0) {
          getOgcWkt(&lasreader->header, wkt);
        }
        if (wkt.size() != 0) {
          pjHelper.set_fwd_crs_transform(wkt.c_str());
        }

        if (!lasreader) {
          logger.warning("cannot read las file: {}", lasfile);
          continue;
        }

        Box file_bbox;
        file_bbox.add(pjHelper.coord_transform_fwd(lasreader->get_min_x(),
                                                   lasreader->get_min_y(),
                                                   lasreader->get_min_z()));
        file_bbox.add(pjHelper.coord_transform_fwd(lasreader->get_max_x(),
                                                   lasreader->get_max_y(),
                                                   lasreader->get_max_z()));

        if (!file_bbox.intersects(polygon_extent)) {
          logger.info("no intersection footprints with las file: {}", lasfile);
          continue;
        }

        // tell lasreader our area of interest. It will then use quadtree
        // indexing if available (.lax file created with lasindex)
        pjHelper.set_rev_crs_transform(wkt.c_str());
        const auto aoi_min = pjHelper.coord_transform_rev(polygon_extent.min());
        const auto aoi_max = pjHelper.coord_transform_rev(polygon_extent.max());
        pjHelper.clear_rev_crs_transform();
        // std::cout << lasreader->npoints << std::endl;
        // std::cout << lasreader->get_min_x() << " " << lasreader->get_min_y()
        // << " " << lasreader->get_min_z() << std::endl; std::cout <<
        // aoi_min[0] << " " << aoi_min[1] << " " << aoi_min[2] << std::endl;
        // std::cout << lasreader->get_max_x() << " " << lasreader->get_max_y()
        // << " " << lasreader->get_max_z() << std::endl; std::cout <<
        // aoi_max[0] << " " << aoi_max[1] << " " << aoi_max[2] << std::endl;
        lasreader->inside_rectangle(aoi_min[0], aoi_min[1], aoi_max[0],
                                    aoi_max[1]);

        // The point cloud acquisition year is the year of the GPS time of the
        // last point in the AOI. Unless, GPS Week Time is used, in which case
        // we default to the 'file creation year'.
        int acqusition_year(0);
        bool use_file_creation_year = useFileCreationYear(lasreader);
        if (use_file_creation_year) {
          acqusition_year = (int)lasreader->header.file_creation_year;
        }
        while (lasreader->read_point()) {
          if (!use_file_creation_year && cfg.use_acquisition_year)
            acqusition_year = getAcquisitionYearOfPoint(&lasreader->point);
          pip_collector.add_point(
              pjHelper.coord_transform_fwd(lasreader->point.get_x(),
                                           lasreader->point.get_y(),
                                           lasreader->point.get_z()),
              lasreader->point.get_classification(), acqusition_year);
        }
        logger.info("Point cloud acquisition year: {}",
                    acqusition_year);  // just for debug

        pjHelper.clear_fwd_crs_transform();
        lasreader->close();
        delete lasreader;
      }

      pip_collector.do_post_process(
          cfg.ground_percentile, cfg.max_density_delta, cfg.coverage_threshold,
          cfg.clear_if_insufficient, poly_areas, poly_pt_counts_bld,
          poly_pt_counts_grd, poly_ptcoverage_class, poly_densities);
    }

    void process(std::string source, std::vector<LinearRing>& polygons,
                 std::vector<LinearRing>& buf_polygons,
                 std::vector<PointCollection>& point_clouds,
                 vec1f& ground_elevations, vec1i& acquisition_years,
                 const Box& polygon_extent, PointCloudCropperConfig cfg) {
      std::vector<std::string> lasfiles =
          roofer::find_filepaths(source, {".las", ".LAS", ".laz", ".LAZ"});
      process(lasfiles, polygons, buf_polygons, point_clouds, ground_elevations,
              acquisition_years, polygon_extent, cfg);
    }
  };

  std::unique_ptr<PointCloudCropperInterface> createPointCloudCropper(
      roofer::misc::projHelperInterface& pjh) {
    return std::make_unique<PointCloudCropper>(pjh);
  };

}  // namespace roofer::io
