#pragma once
#include <memory>
#include <roofer/common/datastructures.hpp>
#include <roofer/reconstruction/cgal_shared_definitions.hpp>

namespace roofer::reconstruction {

  struct ArrangementSnapperConfig {
    float dist_thres = 0.005;
  };

  struct ArrangementSnapperInterface {
    // add_output("triangles_og", typeid(TriangleCollection));
    // add_output("segment_ids_og", typeid(vec1i));
    // add_output("triangles_snapped", typeid(TriangleCollection));
    // add_output("segment_ids_snapped", typeid(vec1i));

    virtual ~ArrangementSnapperInterface() = default;
    virtual void compute(
        Arrangement_2& arrangement,
        ArrangementSnapperConfig config = ArrangementSnapperConfig()) = 0;
  };

  std::unique_ptr<ArrangementSnapperInterface> createArrangementSnapper();
}  // namespace roofer::reconstruction
