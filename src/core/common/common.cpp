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

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <initializer_list>
#include <roofer/common/common.hpp>

namespace roofer {

  namespace fs = std::filesystem;

  const Box& Geometry::box() {
    if (!bbox.has_value()) {
      compute_box();
    }
    return *bbox;
  };
  size_t Geometry::dimension() { return 3; }

  // geometry helpers:
  template <typename T>
  float ring_signed_area(T& ring) {
    float result = 0;
    const auto n = ring.size();
    for (size_t i = 0; i < n; ++i) {
      size_t i_n;
      if (i == (n - 1)) {
        i_n = 0;
      } else {
        i_n = i + 1;
      }

      result += ring[i][0] * ring[i_n][1] - ring[i_n][0] * ring[i][1];
    }
    return result / 2;
  }

  // geometry types:

  void LinearRing::compute_box() {
    if (!bbox.has_value()) {
      bbox = Box();
      for (auto& t : *this) {
        bbox->add(t);
      }
    }
  }
  float LinearRing::signed_area() const {
    float result = ring_signed_area(*this);
    for (auto& iring : interior_rings_) {
      // ring_signed_area should be negative if iring is stored clockwise as it
      // should
      result += ring_signed_area(iring);
    }
    return result;
  }
  size_t LinearRing::vertex_count() const { return size(); }
  float* LinearRing::get_data_ptr() { return (*this)[0].data(); }
  std::vector<vec3f>& LinearRing::interior_rings() { return interior_rings_; }
  const std::vector<vec3f>& LinearRing::interior_rings() const {
    return interior_rings_;
  }

  Segment::Segment() {}
  Segment::Segment(arr3f source, arr3f target) {
    (*this)[0] = source;
    (*this)[1] = target;
  }
  void Segment::compute_box() {
    if (!bbox.has_value()) {
      bbox = Box();
      bbox->add((*this)[0]);
      bbox->add((*this)[1]);
    }
  }
  size_t Segment::vertex_count() const { return 2; }
  float* Segment::get_data_ptr() { return (*this)[0].data(); }

  void LineString::compute_box() {
    if (!bbox.has_value()) {
      bbox = Box();
      for (auto& t : *this) {
        bbox->add(t);
      }
    }
  }
  size_t LineString::vertex_count() const { return size(); }
  float* LineString::get_data_ptr() { return (*this)[0].data(); }

  size_t PointCollection::vertex_count() const { return size(); }
  void PointCollection::compute_box() {
    if (!bbox.has_value()) {
      bbox = Box();
      bbox->add(*this);
    }
  }
  float* PointCollection::get_data_ptr() { return (*this)[0].data(); }

  attribute_vec_map& AttributeVecMap::get_attributes() { return attribs_; }
  const attribute_vec_map& AttributeVecMap::get_attributes() const {
    return attribs_;
  }
  bool AttributeVecMap::has_attributes() const { return attribs_.size() != 0; }
  template <typename T>
  bool AttributeVecMap::holds_alternative(const std::string& name) const {
    if (attribs_.find(name) != attribs_.end()) {
      return std::holds_alternative<std::vector<std::optional<T>>>(
          attribs_.at(name));
    }
    return false;
  }
  template bool AttributeVecMap::holds_alternative<bool>(
      const std::string& name) const;
  template bool AttributeVecMap::holds_alternative<int>(
      const std::string& name) const;
  template bool AttributeVecMap::holds_alternative<float>(
      const std::string& name) const;
  template bool AttributeVecMap::holds_alternative<std::string>(
      const std::string& name) const;
  template bool AttributeVecMap::holds_alternative<arr3f>(
      const std::string& name) const;
  template bool AttributeVecMap::holds_alternative<DateTime>(
      const std::string& name) const;
  template bool AttributeVecMap::holds_alternative<Date>(
      const std::string& name) const;
  template bool AttributeVecMap::holds_alternative<Time>(
      const std::string& name) const;

  template <typename T>
  const std::vector<std::optional<T>>* AttributeVecMap::get_if(
      const std::string& name) const {
    if (attribs_.find(name) != attribs_.end()) {
      return std::get_if<std::vector<std::optional<T>>>(&attribs_.at(name));
    }
    return nullptr;
  }
  template const veco1b* AttributeVecMap::get_if<bool>(
      const std::string& name) const;
  template const veco1i* AttributeVecMap::get_if<int>(
      const std::string& name) const;
  template const veco1f* AttributeVecMap::get_if<float>(
      const std::string& name) const;
  template const veco1s* AttributeVecMap::get_if<std::string>(
      const std::string& name) const;
  template const veco3f* AttributeVecMap::get_if<arr3f>(
      const std::string& name) const;
  template const veco1DT* AttributeVecMap::get_if<DateTime>(
      const std::string& name) const;
  template const veco1D* AttributeVecMap::get_if<Date>(
      const std::string& name) const;
  template const veco1T* AttributeVecMap::get_if<Time>(
      const std::string& name) const;

  template <typename T>
  std::vector<std::optional<T>>* AttributeVecMap::get_if(
      const std::string& name) {
    if (attribs_.find(name) != attribs_.end()) {
      return std::get_if<std::vector<std::optional<T>>>(&attribs_.at(name));
    }
    return nullptr;
  }
  template veco1b* AttributeVecMap::get_if<bool>(const std::string& name);
  template veco1i* AttributeVecMap::get_if<int>(const std::string& name);
  template veco1f* AttributeVecMap::get_if<float>(const std::string& name);
  template veco1s* AttributeVecMap::get_if<std::string>(
      const std::string& name);
  template veco3f* AttributeVecMap::get_if<arr3f>(const std::string& name);
  template veco1DT* AttributeVecMap::get_if<DateTime>(const std::string& name);
  template veco1D* AttributeVecMap::get_if<Date>(const std::string& name);
  template veco1T* AttributeVecMap::get_if<Time>(const std::string& name);

  template <typename T>
  std::vector<std::optional<T>>& AttributeVecMap::insert_vec(
      const std::string& name) {
    attribs_[name] = std::vector<std::optional<T>>{};
    return std::get<std::vector<std::optional<T>>>(attribs_.at(name));
  }
  template veco1b& AttributeVecMap::insert_vec<bool>(const std::string& name);
  template veco1i& AttributeVecMap::insert_vec<int>(const std::string& name);
  template veco1f& AttributeVecMap::insert_vec<float>(const std::string& name);
  template veco1s& AttributeVecMap::insert_vec<std::string>(
      const std::string& name);
  template veco3f& AttributeVecMap::insert_vec<arr3f>(const std::string& name);
  template veco1DT& AttributeVecMap::insert_vec<DateTime>(
      const std::string& name);
  template veco1D& AttributeVecMap::insert_vec<Date>(const std::string& name);
  template veco1T& AttributeVecMap::insert_vec<Time>(const std::string& name);

  attribute_vec_map::const_iterator AttributeVecMap::begin() const {
    return attribs_.begin();
  };
  attribute_vec_map::const_iterator AttributeVecMap::end() const {
    return attribs_.end();
  };

  size_t TriangleCollection::vertex_count() const { return size() * 3; }
  void TriangleCollection::compute_box() {
    if (!bbox.has_value()) {
      bbox = Box();
      for (auto& t : *this) {
        bbox->add(t[0]);
        bbox->add(t[1]);
        bbox->add(t[2]);
      }
    }
  }
  float* TriangleCollection::get_data_ptr() { return (*this)[0][0].data(); }

  size_t SegmentCollection::vertex_count() const { return size() * 2; }
  void SegmentCollection::compute_box() {
    if (!bbox.has_value()) {
      bbox = Box();
      for (auto& t : *this) {
        bbox->add(t[0]);
        bbox->add(t[1]);
      }
    }
  }
  float* SegmentCollection::get_data_ptr() { return (*this)[0][0].data(); }

  size_t LineStringCollection::vertex_count() const {
    size_t result = 0;
    for (auto& vec : *this) {
      result += vec.size();
    }
    return result;
  }
  void LineStringCollection::compute_box() {
    if (!bbox.has_value()) {
      bbox = Box();
      for (auto& vec : *this) {
        bbox->add(vec);
      }
    }
  }
  float* LineStringCollection::get_data_ptr() { return (*this)[0][0].data(); }

  size_t LinearRingCollection::vertex_count() const {
    size_t result = 0;
    for (auto& vec : *this) {
      result += vec.size();
    }
    return result;
  }
  void LinearRingCollection::compute_box() {
    if (!bbox.has_value()) {
      bbox = Box();
      for (auto& vec : *this) {
        bbox->add(vec);
      }
    }
  }
  float* LinearRingCollection::get_data_ptr() { return (*this)[0][0].data(); }

  void Mesh::push_polygon(LinearRing& polygon, int label) {
    polygons_.push_back(polygon);
    labels_.push_back(label);
  }
  // void Mesh::push_attribute(std::string name, std::any value) {
  //   attributes_.at(name).values.push_back(value);
  // }
  std::vector<LinearRing>& Mesh::get_polygons() { return polygons_; };
  const std::vector<LinearRing>& Mesh::get_polygons() const {
    return polygons_;
  };

  std::vector<int>& Mesh::get_labels() { return labels_; };
  const std::vector<int>& Mesh::get_labels() const { return labels_; };
  // std::unordered_map<std::string, AttributeVec>&  Mesh::get_attributes(){
  //   return attributes_;
  // };
  // const std::unordered_map<std::string, AttributeVec>&
  // Mesh::get_attributes()const {
  //   return attributes_;
  // };

  void MultiTriangleCollection::push_back(
      TriangleCollection& trianglecollection) {
    trianglecollections_.push_back(trianglecollection);
  }
  void MultiTriangleCollection::push_back(AttributeMap& attributemap) {
    attributes_.push_back(attributemap);
  }

  size_t MultiTriangleCollection::tri_size() const {
    return trianglecollections_.size();
  }
  size_t MultiTriangleCollection::attr_size() const {
    return attributes_.size();
  }

  bool MultiTriangleCollection::has_attributes() {
    return !attributes_.empty();
  }
  bool MultiTriangleCollection::has_attributes() const {
    return !attributes_.empty();
  }

  std::vector<TriangleCollection>&
  MultiTriangleCollection::get_tricollections() {
    return trianglecollections_;
  }
  const std::vector<TriangleCollection>&
  MultiTriangleCollection::get_tricollections() const {
    return trianglecollections_;
  }

  std::vector<AttributeMap>& MultiTriangleCollection::get_attributes() {
    return attributes_;
  }
  const std::vector<AttributeMap>& MultiTriangleCollection::get_attributes()
      const {
    return attributes_;
  }

  TriangleCollection& MultiTriangleCollection::tri_at(size_t i) {
    return trianglecollections_.at(i);
  }
  const TriangleCollection& MultiTriangleCollection::tri_at(size_t i) const {
    return trianglecollections_.at(i);
  }

  AttributeMap& MultiTriangleCollection::attr_at(size_t i) {
    return attributes_.at(i);
  }
  const AttributeMap& MultiTriangleCollection::attr_at(size_t i) const {
    return attributes_.at(i);
  }

  std::vector<std::string> split_string(const std::string& s,
                                        std::string delimiter) {
    std::vector<std::string> parts;
    size_t last = 0;
    size_t next = 0;

    while ((next = s.find(delimiter, last)) != std::string::npos) {
      parts.push_back(s.substr(last, next - last));
      last = next + 1;
    }
    parts.push_back(s.substr(last));
    return parts;
  }

  std::vector<std::string> find_filepaths(
      const std::string& filepaths,
      std::initializer_list<std::string> extensions) {
    std::vector<std::string> files;
    std::vector<std::string> filepath_parts = split_string(filepaths, " ");
    for (const auto& filepath_part : filepath_parts) {
      if (fs::is_directory(filepath_part)) {
        for (auto& p : fs::directory_iterator(filepath_part)) {
          auto ext = p.path().extension();
          for (auto& filter_ext : extensions) {
            if (filter_ext == ext) {
              files.push_back(p.path().string());
            }
          }
        }
      } else {
        if (fs::exists(filepath_part)) files.push_back(filepath_part);
      }
    }
    return files;
  }

  bool has_duplicates_ring(const vec3f& poly, const float& dupe_threshold) {
    auto pl = *poly.rbegin();
    for (auto& p : poly) {
      if (std::fabs(pl[0] - p[0]) < dupe_threshold &&
          std::fabs(pl[1] - p[1]) < dupe_threshold &&
          std::fabs(pl[2] - p[2]) < dupe_threshold) {
        return true;
      }
      pl = p;
    }
    return false;
  }

  bool is_degenerate(const LinearRing& poly, const float& dupe_threshold) {
    if (poly.size() < 3) return true;
    if (has_duplicates_ring(poly, dupe_threshold)) return true;

    for (auto& ring : poly.interior_rings()) {
      if (ring.size() < 3) return true;
      if (has_duplicates_ring(ring, dupe_threshold)) return true;
    }
    return false;
  }

  void fix_duplicates_ring(vec3f& poly, vec3f& new_ring,
                           float& dupe_threshold) {
    auto pl = *poly.rbegin();
    for (auto& p : poly) {
      if (!(std::fabs(pl[0] - p[0]) < dupe_threshold &&
            std::fabs(pl[1] - p[1]) < dupe_threshold &&
            std::fabs(pl[2] - p[2]) < dupe_threshold)) {
        new_ring.push_back(p);
      }
      pl = p;
    }
  }

  LinearRing fix_duplicates(LinearRing& poly, float& dupe_threshold) {
    LinearRing new_lr;
    fix_duplicates_ring(poly, new_lr, dupe_threshold);

    for (auto& ring : poly.interior_rings()) {
      vec3f new_ring;
      fix_duplicates_ring(ring, new_ring, dupe_threshold);
      new_lr.interior_rings().push_back(new_ring);
    }
    return new_lr;
  }

  void pop_back_if_equal_to_front(LinearRing& poly) {
    auto it = poly.end();
    --it;
    if ((*poly.begin()) == *it) poly.erase(it);
  }

  std::time_t Date::to_time_t() {
    std::tm tm{};
    tm.tm_year = this->year - 1900;
    tm.tm_mon = this->month - 1;
    tm.tm_mday = this->day;
    return std::mktime(&tm);
  }

  std::string Date::format_to_ietf() {
    std::time_t t = this->to_time_t();
    char timeString[std::size("yyyy-mm-dd")];
    std::strftime(std::data(timeString), std::size(timeString), "%FT",
                  std::gmtime(&t));
    std::string ret(timeString);
    return ret;
  }

  std::time_t DateTime::to_time_t() {
    std::tm tm{};
    tm.tm_year = this->date.year - 1900;
    tm.tm_mon = this->date.month - 1;
    tm.tm_mday = this->date.day;
    tm.tm_hour = this->time.hour;
    tm.tm_min = this->time.minute;
    tm.tm_sec = this->time.second;
    return std::mktime(&tm);
  }

  // Format to date-time, ignoring the time zone and assuming UTC.
  // According to https://datatracker.ietf.org/doc/html/rfc3339#section-5.6
  std::string DateTime::format_to_ietf() {
    std::time_t t = this->to_time_t();
    char timeString[std::size("yyyy-mm-ddThh:mm:ssZ")];
    std::strftime(std::data(timeString), std::size(timeString), "%FT%TZ",
                  std::gmtime(&t));
    std::string ret(timeString);
    return ret;
  }
}  // namespace roofer
