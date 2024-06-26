#pragma once
#include <cstddef>
#include <memory>
#include <roofer/common/datastructures.hpp>
#include <roofer/misc/projHelper.hpp>

namespace roofer::io {
  struct VectorWriterInterface {
    std::string srs = "";  // "EPSG:7415";
    std::string conn_string_ = "out";
    std::string gdaldriver_ = "GPKG";
    std::string layername_ = "geom";
    bool overwrite_layer_ = true;
    bool overwrite_file_ = true;
    bool create_directories_ = true;
    bool do_transactions_ = false;
    int transaction_batch_size_ = 1000;

    roofer::misc::projHelperInterface& pjHelper;

    VectorWriterInterface(roofer::misc::projHelperInterface& pjh)
        : pjHelper(pjh){};
    virtual ~VectorWriterInterface() = default;

    virtual void writePolygons(const std::string& source,
                               const std::vector<LinearRing>& polygons,
                               const AttributeVecMap& attributes, size_t begin,
                               size_t end) = 0;

    void writePolygons(const std::string& source,
                       const std::vector<LinearRing>& polygons,
                       const AttributeVecMap& attributes) {
      writePolygons(source, polygons, attributes, 0, polygons.size());
    };
  };

  std::unique_ptr<VectorWriterInterface> createVectorWriterOGR(
      roofer::misc::projHelperInterface& pjh);
}  // namespace roofer::io
