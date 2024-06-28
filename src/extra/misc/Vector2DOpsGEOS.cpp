// This file is part of gfp-gdal
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
#include <geos_c.h>
#include <roofer/logger/logger.h>

#include <roofer/misc/Vector2DOps.hpp>
#include <vector>

namespace roofer::misc {

  GEOSContextHandle_t gc;

  enum ORIENTATION { CW, CCW };

  void print_geos_message(const char* message, ...) {
    logger::Logger::get_logger().error("{}", message);
  }

  GEOSGeometry* orient_ring(const GEOSGeometry*& g_ring,
                            ORIENTATION orientation) {
    const GEOSCoordSequence* g_coord_seq = GEOSGeom_getCoordSeq_r(gc, g_ring);
    char is_ccw;
    GEOSCoordSeq_isCCW_r(gc, g_coord_seq, &is_ccw);
    if ((is_ccw == 1 && orientation == CW) ||
        (is_ccw == 0 && orientation == CCW)) {
      return GEOSReverse_r(gc, g_ring);
    }
    return nullptr;
  }
  bool orient_polygon(GEOSGeometry*& g_polygon, ORIENTATION orientation) {
    const GEOSGeometry* g_ring = GEOSGetExteriorRing_r(gc, g_polygon);
    GEOSGeometry* g_ring_ = orient_ring(g_ring, orientation);
    bool reversed = false;
    if (!g_ring_) {
      g_ring_ = GEOSGeom_clone_r(gc, g_ring);
    } else {
      reversed = true;
    }

    ORIENTATION orientation_int = (orientation == CCW) ? CW : CCW;
    std::vector<GEOSGeometry*> g_holes;
    for (size_t i = 0; i < GEOSGetNumInteriorRings_r(gc, g_polygon); ++i) {
      const GEOSGeometry* g_iring = GEOSGetInteriorRingN_r(gc, g_polygon, i);
      GEOSGeometry* g_iring_ = orient_ring(g_iring, orientation_int);
      if (g_iring_) {
        g_holes.push_back(g_iring_);
        reversed |= reversed;
      } else {
        g_holes.push_back(GEOSGeom_clone_r(gc, g_iring));
      }
    }

    GEOSGeom_destroy_r(gc, g_polygon);
    g_polygon =
        GEOSGeom_createPolygon_r(gc, g_ring_, g_holes.data(), g_holes.size());

    return reversed;
  }

  template <typename T>
  void to_geos_linear_ring(const T& lr, GEOSGeometry*& g_lr) {
    const auto size = lr.size();
    GEOSCoordSequence* g_coord_seq = GEOSCoordSeq_create_r(gc, size + 1, 3);
    for (size_t i = 0; i < size; ++i) {
      GEOSCoordSeq_setX_r(gc, g_coord_seq, i, lr[i][0]);
      GEOSCoordSeq_setY_r(gc, g_coord_seq, i, lr[i][1]);
      GEOSCoordSeq_setZ_r(gc, g_coord_seq, i, lr[i][2]);
    }
    // close the ring
    GEOSCoordSeq_setX_r(gc, g_coord_seq, size, lr[0][0]);
    GEOSCoordSeq_setY_r(gc, g_coord_seq, size, lr[0][1]);
    GEOSCoordSeq_setZ_r(gc, g_coord_seq, size, lr[0][2]);

    g_lr = GEOSGeom_createLinearRing_r(gc, g_coord_seq);
    GEOSGeometry* g_lr_ = nullptr;
  }

  void to_geos_polygon(const LinearRing& lr, GEOSGeometry*& g_polygon) {
    GEOSGeometry* g_exterior = nullptr;
    to_geos_linear_ring(lr, g_exterior);

    std::vector<GEOSGeometry*> g_holes;
    for (auto& hole : lr.interior_rings()) {
      GEOSGeometry* g_hole = nullptr;
      to_geos_linear_ring(hole, g_hole);
      g_holes.push_back(g_hole);
    }
    if (g_holes.size() == 0)
      g_polygon = GEOSGeom_createPolygon_r(gc, g_exterior, nullptr, 0);
    else
      g_polygon = GEOSGeom_createPolygon_r(gc, g_exterior, g_holes.data(),
                                           g_holes.size());
  }

  template <typename T>
  void from_geos_linear_ring(const GEOSGeometry* g_lin_ring, T& gf_ring) {
    const GEOSCoordSequence* g_coord_seq =
        GEOSGeom_getCoordSeq_r(gc, g_lin_ring);
    unsigned int size, dims;
    GEOSCoordSeq_getSize_r(gc, g_coord_seq, &size);
    GEOSCoordSeq_getDimensions_r(gc, g_coord_seq, &dims);

    // note we do not repeat the first coordinate
    for (size_t i = 0; i < (size - 1); ++i) {
      double x, y, z = 0;
      GEOSCoordSeq_getX_r(gc, g_coord_seq, i, &x);
      GEOSCoordSeq_getY_r(gc, g_coord_seq, i, &y);
      if (dims == 3) GEOSCoordSeq_getZ_r(gc, g_coord_seq, i, &z);
      gf_ring.push_back({float(x), float(y), float(z)});
    }
  }

  LinearRing from_geos_polygon(const GEOSGeometry* g_polygon) {
    LinearRing lr;
    const GEOSGeometry* g_ring = GEOSGetExteriorRing_r(gc, g_polygon);
    from_geos_linear_ring(g_ring, lr);
    for (size_t i = 0; i < GEOSGetNumInteriorRings_r(gc, g_polygon); ++i) {
      const GEOSGeometry* g_iring = GEOSGetInteriorRingN_r(gc, g_polygon, i);
      vec3f hole;
      from_geos_linear_ring(g_iring, hole);
      lr.interior_rings().push_back(hole);
    }
    return lr;
  }

  void itemQueryCallback(void* item, void* userdata) {
    std::vector<void*>* vecdata = static_cast<std::vector<void*>*>(userdata);
    vecdata->push_back(item);
  }
  struct RTreeGEOS : public RTreeInterface {
    GEOSSTRtree* tree;
    std::vector<GEOSGeometry*> geoms;
    GEOSContextHandle_t gc_;

    RTreeGEOS() : RTreeInterface() {
      gc_ = GEOS_init_r();
      tree = GEOSSTRtree_create_r(gc_, 10);
    };

    ~RTreeGEOS() override {
      GEOSSTRtree_destroy_r(gc_, tree);
      for (auto box_g : geoms) GEOSGeom_destroy_r(gc_, box_g);
    };

    void insert(const roofer::TBox<double>& box, void* item) override {
      GEOSGeometry* box_g = GEOSGeom_createRectangle_r(
          gc_, box.pmin[0], box.pmin[1], box.pmax[0], box.pmax[1]);
      auto& logger = logger::Logger::get_logger();
      logger.info("WKT: {}", GEOSGeomToWKT_r(gc_, box_g));
      GEOSSTRtree_insert_r(gc_, tree, box_g, item);
      geoms.push_back(box_g);
    };

    virtual std::vector<void*> query(
        const roofer::TBox<double>& query) override {
      auto* query_g = GEOSGeom_createRectangle_r(
          gc_, query.pmin[0], query.pmin[1], query.pmax[0], query.pmax[1]);
      std::vector<void*> result;
      GEOSSTRtree_query_r(gc_,
                          tree,               // STRTree to query
                          query_g,            // GEOSGeometry query bounds
                          itemQueryCallback,  // Callback to process index
                                              // entries that pass query
                          &result);  // Userdata to hand to the callback
      /* Free the query bounds geometry */
      GEOSGeom_destroy_r(gc_, query_g);
      return result;
    };
  };

  std::unique_ptr<RTreeInterface> createRTreeGEOS() {
    return std::make_unique<RTreeGEOS>();
  };

  struct Vector2DOpsGEOS : public Vector2DOpsInterface {
    void simplify_polygons(std::vector<LinearRing>& polygons, float tolerance,
                           // bool output_failures,
                           bool orient_after_simplify) override {
      gc = GEOS_init_r();
      // GEOSContext_setNoticeHandler_r(gc, print_geos_message);
      // GEOSContext_setErrorHandler_r(gc, print_geos_message);

      auto& logger = logger::Logger::get_logger();

      for (auto& lr : polygons) {
        if (lr.size() < 3) {
          logger.info("feature skipped with less than 3 points");
          // if (output_failures) polygons_out.push_back(lr);
          continue;
        }

        GEOSGeometry* g_polygon = nullptr;
        to_geos_polygon(lr, g_polygon);

        if (GEOSisValid_r(gc, g_polygon) != 1) {
          logger.info("feature not valid");
          GEOSGeom_destroy_r(gc, g_polygon);
          // if (output_failures) polygons_out.push_back(lr);
          continue;
        }

        GEOSGeometry* simplified_geom =
            GEOSSimplify_r(gc, g_polygon, double(tolerance));
        if (GEOSisValid_r(gc, simplified_geom) != 1) {
          logger.info("feature not valid after simplify");
          GEOSGeom_destroy_r(gc, g_polygon);
          // if (output_failures) polygons_out.push_back(lr);
          continue;
        }
        if (orient_after_simplify) orient_polygon(simplified_geom, CCW);

        // check if the simplified geometry is valid and has vertices
        unsigned int size;
        const GEOSGeometry* g_ring = GEOSGetExteriorRing_r(gc, simplified_geom);
        const GEOSCoordSequence* g_coord_seq =
            GEOSGeom_getCoordSeq_r(gc, g_ring);
        GEOSCoordSeq_getSize_r(gc, g_coord_seq, &size);
        if (size == 0) {
          logger.info("feature size 0 after simplify");
          // if (output_failures) polygons_out.push_back(lr);
        } else {
          lr = from_geos_polygon(simplified_geom);
        }

        GEOSGeom_destroy_r(gc, g_polygon);
        GEOSGeom_destroy_r(gc, simplified_geom);
      }
      GEOS_finish_r(gc);
    }

    void buffer_polygons(std::vector<LinearRing>& polygons,
                         float offset = 4) override {
      gc = GEOS_init_r();

      auto& logger = logger::Logger::get_logger();

      for (auto& lr : polygons) {
        GEOSGeometry* g_polygon = nullptr;
        to_geos_polygon(lr, g_polygon);

        GEOSGeometry* buffered_geom =
            GEOSBuffer_r(gc, g_polygon, double(offset), 8);
        orient_polygon(buffered_geom, CCW);

        if (GEOSisValid_r(gc, buffered_geom) != 1) {
          logger.info("feature not simplified");
        } else {
          lr = from_geos_polygon(buffered_geom);
        }

        GEOSGeom_destroy_r(gc, g_polygon);
        GEOSGeom_destroy_r(gc, buffered_geom);
      }
      GEOS_finish_r(gc);
    }
  };

  std::unique_ptr<Vector2DOpsInterface> createVector2DOpsGEOS() {
    return std::make_unique<Vector2DOpsGEOS>();
  };

  // void GEOSMergeLinesNode::process()
  // {
  //     std::cout << "Merging lines\n";
  //     auto lines = input("lines").get<LineStringCollection>();
  //     gc = GEOS_init_r();
  //     std::vector<GEOSGeometry *> linearray;
  //     for (int i = 0; i < lines.size(); i++)
  //     {
  //         GEOSCoordSequence *points = GEOSCoordSeq_create_r(gc, 2, 3);
  //         for (int j = 0; j < 2; j++)
  //         {
  //         GEOSCoordSeq_setX_r(gc, points, j, lines[i][j][0]);
  //         GEOSCoordSeq_setY_r(gc, points, j, lines[i][j][1]);
  //         GEOSCoordSeq_setZ_r(gc, points, j, lines[i][j][2]);
  //         }
  //         GEOSGeometry *line = GEOSGeom_createLineString_r(gc, points);
  //         linearray.push_back(line);
  //     }
  //     GEOSGeometry *lineCollection = GEOSGeom_createCollection_r(gc,
  //     GEOS_GEOMETRYCOLLECTION, linearray.data(), lines.size()); GEOSGeometry
  //     *mergedlines = GEOSLineMerge_r(gc, lineCollection);

  //     LineStringCollection outputLines;
  //     for (int i = 0; i < GEOSGetNumGeometries_r(gc, mergedlines); i++)
  //     {
  //         const GEOSCoordSequence *l = GEOSGeom_getCoordSeq_r(gc,
  //         GEOSGetGeometryN_r(gc, mergedlines, i)); vec3f line_vec3f; unsigned
  //         int size; GEOSCoordSeq_getSize_r(gc, l, &size); for (int j = 0; j <
  //         size; j++)
  //         {
  //         double x, y, z = 0;
  //         GEOSCoordSeq_getX_r(gc, l, j, &x);
  //         GEOSCoordSeq_getY_r(gc, l, j, &y);
  //         GEOSCoordSeq_getZ_r(gc, l, j, &z);
  //         line_vec3f.push_back({float(x), float(y), float(z)});
  //         }
  //         outputLines.push_back(line_vec3f);
  //     }

  //     // clean GEOS geometries
  //     for (auto l : linearray)
  //     {
  //         GEOSGeom_destroy_r(gc, l);
  //     }
  //     //GEOSGeom_destroy_r(gc, lineCollection);
  //     GEOSGeom_destroy_r(gc, mergedlines);
  //     GEOS_finish_r(gc);

  //     output("lines").set(outputLines);
  // }

}  // namespace roofer::misc
