#pragma once
#include <memory>
#include <roofer/common/common.hpp>
#include <roofer/common/datastructures.hpp>
#include <vector>

namespace roofer::misc {
  struct RTreeInterface {
    virtual ~RTreeInterface(){};

    virtual void insert(const roofer::TBox<double>& box, void* item) = 0;
    virtual std::vector<void*> query(const roofer::TBox<double>& query) = 0;
  };

  struct Vector2DOpsInterface {
    virtual ~Vector2DOpsInterface() = default;

    virtual void simplify_polygons(std::vector<LinearRing>& polygons,
                                   float tolerance = 0.01,
                                   // bool output_failures = true,
                                   bool orient_after_simplify = true) = 0;

    virtual void buffer_polygons(std::vector<LinearRing>& polygons,
                                 float offset = 4) = 0;
  };

  std::unique_ptr<Vector2DOpsInterface> createVector2DOpsGEOS();
  std::unique_ptr<RTreeInterface> createRTreeGEOS();
}  // namespace roofer::misc
