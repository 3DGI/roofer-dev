#pragma once
#include <cstddef>
#include <memory>
#include <roofer/common/datastructures.hpp>
#include <roofer/misc/projHelper.hpp>

namespace roofer::io {
  struct CityJsonWriterInterface {
    // parameter variables
    std::string CRS_ = "EPSG:7415";
    std::string filepath_;
    std::string identifier_attribute = "";

    bool prettyPrint_ = false;
    bool only_output_renamed_ = false;

    vec1s key_options;
    StrMap output_attribute_names;

    float translate_x_ = 0.;
    float translate_y_ = 0.;
    float translate_z_ = 0.;
    float scale_x_ = 1.;
    float scale_y_ = 1.;
    float scale_z_ = 1.;

    roofer::misc::projHelperInterface& pjHelper;

    CityJsonWriterInterface(roofer::misc::projHelperInterface& pjh)
        : pjHelper(pjh){};
    virtual ~CityJsonWriterInterface() = default;

    // add_poly_input("part_attributes", {typeid(bool), typeid(int),
    // typeid(float), typeid(std::string), typeid(Date), typeid(Time),
    // typeid(DateTime)}); add_poly_input("attributes", {typeid(bool),
    // typeid(int), typeid(float), typeid(std::string), typeid(std::string),
    // typeid(Date), typeid(Time), typeid(DateTime)});

    virtual void write(
        const std::string& source, const LinearRing& footprints,
        const std::unordered_map<int, Mesh>* geometry_lod12,
        const std::unordered_map<int, Mesh>* geometry_lod13,
        const std::unordered_map<int, Mesh>* geometry_lod22,
        const AttributeVecMap& attributes,
        const size_t attribute_index) = 0;
  };

  std::unique_ptr<CityJsonWriterInterface> createCityJsonWriter(
      roofer::misc::projHelperInterface& pjh);
}  // namespace roofer::io
