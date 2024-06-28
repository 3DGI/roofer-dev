#pragma once
#include <memory>
#include <roofer/common/datastructures.hpp>
#include <roofer/misc/projHelper.hpp>

namespace roofer::io {
  struct PointCloudReaderInterface {
    roofer::misc::projHelperInterface& pjHelper;

    PointCloudReaderInterface(roofer::misc::projHelperInterface& pjh)
        : pjHelper(pjh){};
    virtual ~PointCloudReaderInterface() = default;

    virtual void open(const std::string& source) = 0;

    virtual void close() = 0;

    virtual TBox<double> getExtent() = 0;

    virtual void readPointCloud(PointCollection& points,
                                vec1i* classification = nullptr,
                                vec1i* order = nullptr,
                                vec1f* intensities = nullptr,
                                vec3f* colors = nullptr) = 0;
  };

  std::unique_ptr<PointCloudReaderInterface> createPointCloudReaderLASlib(
      roofer::misc::projHelperInterface& pjh);
}  // namespace roofer::io
