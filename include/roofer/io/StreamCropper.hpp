#pragma once
#include <memory>
#include <roofer/common/datastructures.hpp>
#include <roofer/misc/projHelper.hpp>

namespace roofer::io {
  struct PointCloudCropperConfig {
    std::string filepaths_ = "";
    float cellsize = 50.0;
    float buffer = 1.0;
    float ground_percentile = 0.05;
    float max_density_delta = 0.05;
    float coverage_threshold = 2.0;
    int ground_class = 2;
    int building_class = 6;
    bool clear_if_insufficient = true;
    std::string wkt_ = "";
    bool handle_overlap_points = false;
    bool use_acquisition_year = true;
  };
  struct PointCloudCropperInterface {
    roofer::misc::projHelperInterface& pjHelper;

    PointCloudCropperInterface(roofer::misc::projHelperInterface& pjh)
        : pjHelper(pjh){};
    virtual ~PointCloudCropperInterface() = default;

    virtual void process(
        const std::vector<std::string>& lasfiles,
        std::vector<LinearRing>& polygons,
        std::vector<LinearRing>& buf_polygons,
        std::vector<PointCollection>& point_clouds, vec1f& ground_elevations,
        vec1i& acquisition_years, const Box& polygon_extent,
        PointCloudCropperConfig cfg = PointCloudCropperConfig{}) = 0;
    virtual void process(
        std::string filepaths, std::vector<LinearRing>& polygons,
        std::vector<LinearRing>& buf_polygons,
        std::vector<PointCollection>& point_clouds, vec1f& ground_elevations,
        vec1i& acquisition_years, const Box& polygon_extent,
        PointCloudCropperConfig cfg = PointCloudCropperConfig{}) = 0;
  };

  std::unique_ptr<PointCloudCropperInterface> createPointCloudCropper(
      roofer::misc::projHelperInterface& pjh);
}  // namespace roofer::io
