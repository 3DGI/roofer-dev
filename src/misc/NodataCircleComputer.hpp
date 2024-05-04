
#include <memory>

#include "common.hpp"
#include "datastructures.hpp"

namespace roofer {

  void draw_circle(
    LinearRing& polygon, 
    float& radius,
    arr2f& center
  );

  void compute_nodata_circle(
    PointCollection& pointcloud,
    LinearRing& footprint,
    float* nodata_radius,
    arr2f* nodata_centerpoint = nullptr,
    float polygon_densify = 0.5
  );

}