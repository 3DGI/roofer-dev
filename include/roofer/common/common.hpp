// This file is part of Geoflow
// Copyright (C) 2018-2022 Ravi Peters

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#pragma once

#include <array>
#include <optional>
#include <unordered_map>
#include <vector>
// #include <any>
// #include <typeinfo>
// #include <typeindex>
#include <ctime>
#include <string>
#include <variant>

#include "box.hpp"

namespace roofer {

  typedef std::array<float, 2> arr2f;
  typedef std::array<double, 2> arr2d;
  typedef std::array<float, 3> arr3f;
  typedef std::array<double, 3> arr3d;
  typedef std::vector<std::array<float, 2>> vec2f;

  typedef std::vector<size_t> vec1ui;
  typedef std::vector<int> vec1i;
  typedef std::vector<bool> vec1b;
  typedef std::vector<float> vec1f;
  typedef std::vector<arr3f> vec3f;
  typedef std::vector<std::string> vec1s;

  typedef std::vector<std::optional<size_t>> veco1ui;
  typedef std::vector<std::optional<int>> veco1i;
  typedef std::vector<std::optional<bool>> veco1b;
  typedef std::vector<std::optional<float>> veco1f;
  typedef std::vector<std::optional<arr3f>> veco3f;
  typedef std::vector<std::optional<std::string>> veco1s;

  typedef std::unordered_map<std::string, std::string> StrMap;

  // modelled after
  // https://gdal.org/api/ogrfeature_cpp.html#_CPPv4NK10OGRFeature18GetFieldAsDateTimeEiPiPiPiPiPiPiPi
  struct Date {
    int year;
    int month;
    int day;
    std::time_t to_time_t();
    std::string format_to_ietf();
  };
  struct Time {
    int hour;
    int minute;
    float second;
    int timeZone;
  };
  struct DateTime {
    Date date;
    Time time;
    std::time_t to_time_t();
    std::string format_to_ietf();
  };

  typedef std::vector<std::optional<Date>> veco1D;
  typedef std::vector<std::optional<Time>> veco1T;
  typedef std::vector<std::optional<DateTime>> veco1DT;

  // Attribute types
  typedef std::variant<bool, int, std::string, float, Date, DateTime, Time>
      attribute_value;
  typedef std::unordered_map<std::string, std::vector<attribute_value>>
      AttributeMap;

  typedef std::variant<veco1b, veco1i, veco1s, veco1f, veco3f, veco1D, veco1T,
                       veco1DT>
      attribute_vec;
  typedef std::unordered_map<std::string, attribute_vec> attribute_vec_map;
  // missing:
  // - maintain equal size for all vectors in map
  // - size() member
  class AttributeVecMap {
    attribute_vec_map attribs_;

   public:
    typedef attribute_vec_map::const_iterator const_iterator;
    template <typename T>
    bool holds_alternative(const std::string& name) const;
    template <typename T>
    const std::vector<std::optional<T>>* get_if(const std::string& name) const;
    template <typename T>
    std::vector<std::optional<T>>* get_if(const std::string& name);
    template <typename T>
    std::vector<std::optional<T>>& insert_vec(const std::string& name);

    attribute_vec_map& get_attributes();
    const attribute_vec_map& get_attributes() const;
    bool has_attributes() const;

    const_iterator begin() const;
    const_iterator end() const;
  };

  class Geometry {
   protected:
    std::optional<Box> bbox;
    virtual void compute_box() = 0;

   public:
    virtual size_t vertex_count() const = 0;
    virtual const Box& box();
    size_t dimension();
    virtual float* get_data_ptr() = 0;
  };

  // geometry types:
  // typedef arr3f Point;
  typedef std::array<arr3f, 3> Triangle;
  // typedef std::array<arr3f, 2> Segment;

  class LinearRing : public vec3f, public Geometry {
    std::vector<vec3f> interior_rings_;

   protected:
    void compute_box();

   public:
    size_t vertex_count() const;
    float* get_data_ptr();
    std::vector<vec3f>& interior_rings();
    const std::vector<vec3f>& interior_rings() const;
    float signed_area() const;
  };
  class Segment : public std::array<arr3f, 2>, public Geometry {
   protected:
    void compute_box();

   public:
    // using std::array<arr3f, 2>::array;
    Segment();
    Segment(arr3f source, arr3f target);
    size_t vertex_count() const;
    float* get_data_ptr();
  };

  // class Polygon : public LinearRing
  // {
  // public:
  // std::vector<vec3f> interior_rings_;
  // };

  class LineString : public vec3f, public Geometry {
   protected:
    void compute_box();

   public:
    size_t vertex_count() const;
    float* get_data_ptr();
  };
  template <typename geom_def>
  class GeometryCollection : public Geometry, public std::vector<geom_def> {};

  class TriangleCollection : public GeometryCollection<Triangle> {
   public:
    size_t vertex_count() const;
    virtual void compute_box();
    float* get_data_ptr();
  };

  // MultiTriangleCollection stores a collection of TriangleCollections along
  // with attributes for each TriangleCollection. The vector of
  // TriangleCollections `trianglecollections_` and the vector of AttributeMaps
  // `attributes_` supposed to have the same length when attributes are present,
  // however this is not enforced. The `attributes_` can be empty.
  class MultiTriangleCollection {
    std::vector<TriangleCollection> trianglecollections_;
    std::vector<AttributeMap> attributes_;

   public:
    std::vector<int> building_part_ids_;

    void push_back(TriangleCollection& trianglecollection);
    void push_back(AttributeMap& attributemap);
    std::vector<TriangleCollection>& get_tricollections();
    const std::vector<TriangleCollection>& get_tricollections() const;
    std::vector<AttributeMap>& get_attributes();
    const std::vector<AttributeMap>& get_attributes() const;
    TriangleCollection& tri_at(size_t i);
    const TriangleCollection& tri_at(size_t i) const;
    AttributeMap& attr_at(size_t i);
    const AttributeMap& attr_at(size_t i) const;
    size_t tri_size() const;
    size_t attr_size() const;
    bool has_attributes();
    bool has_attributes() const;
  };

  class SegmentCollection : public GeometryCollection<std::array<arr3f, 2>> {
   public:
    AttributeVecMap attributes;
    size_t vertex_count() const;
    virtual void compute_box();
    float* get_data_ptr();
  };

  class PointCollection : public GeometryCollection<arr3f> {
   public:
    AttributeVecMap attributes;
    size_t vertex_count() const;
    virtual void compute_box();
    float* get_data_ptr();
  };

  class LineStringCollection : public GeometryCollection<vec3f> {
   public:
    size_t vertex_count() const;
    void compute_box();
    float* get_data_ptr();
  };

  class LinearRingCollection : public GeometryCollection<vec3f> {
   public:
    size_t vertex_count() const;
    void compute_box();
    float* get_data_ptr();
  };

  // struct AttributeVec {
  //   AttributeVec(std::type_index ttype) : value_type(ttype) {};
  //   std::vector<std::any> values;
  //   std::type_index value_type;
  // };

  // class Mesh : public Geometry {
  // use indexed vertices?
  class Mesh {
    std::vector<LinearRing> polygons_;
    std::vector<int> labels_;
    // std::unordered_map<std::string, AttributeVec>  attributes_;

   public:
    // Mesh() {};

    void push_polygon(LinearRing& polygon, int label);
    // template <typename T> void create_attribute_field(std::string name) {
    //   attributes_.emplace(name, typeid(T));
    // }
    // void push_attribute(std::string name, std::any value);

    std::vector<LinearRing>& get_polygons();
    const std::vector<LinearRing>& get_polygons() const;
    std::vector<int>& get_labels();
    const std::vector<int>& get_labels() const;
    // std::unordered_map<std::string, AttributeVec>&  get_attributes();
    // const std::unordered_map<std::string, AttributeVec>&  get_attributes()
    // const;
  };

  struct Image {
    std::vector<float> array;
    size_t dim_x;
    size_t dim_y;
    float min_x;
    float min_y;
    float cellsize;
    float nodataval;
  };
  typedef std::unordered_map<std::string, Image> ImageMap;

  std::vector<std::string> split_string(const std::string& s,
                                        std::string delimiter);

  std::vector<std::string> find_filepaths(
      const std::string& path, std::initializer_list<std::string> extensions);

  bool has_duplicates_ring(const vec3f& poly, const float& dupe_threshold);
  bool is_degenerate(const LinearRing& poly, const float& dupe_threshold);
  LinearRing fix_duplicates(LinearRing& poly, float& dupe_threshold);
  void pop_back_if_equal_to_front(LinearRing& poly);

}  // namespace roofer
