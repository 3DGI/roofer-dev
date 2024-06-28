// This file is part of gfp-building-reconstruction
// Copyright (C) 2018-2022 Ravi Peters

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Affero General Public License for more details.

// You should have received a copy of the GNU Affero General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
#pragma once

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>

#include <boost/heap/fibonacci_heap.hpp>
#include <roofer/common/datastructures.hpp>

namespace roofer::linereg {
  using K = CGAL::Exact_predicates_exact_constructions_kernel;
  // typedef CGAL::Exact_predicates_inexact_constructions_kernel K;

  using Point_2 = K::Point_2;
  using Point_3 = K::Point_3;
  using Vector_2 = K::Vector_2;
  using Line_2 = K::Line_2;
  using Segment_2 = K::Segment_2;
  using Polygon_2 = CGAL::Polygon_2<K>;
  using Polygon_with_holes_2 = CGAL::Polygon_with_holes_2<K>;

  struct linetype;

  template <typename T>
  struct Cluster {
    T value;
    bool has_intersection_line;
    std::vector<linetype*> lines;
    virtual double distance(Cluster<T>* other_cluster) = 0;
    virtual void calc_mean_value() = 0;
  };
  struct AngleCluster : public Cluster<double> {
    double distance(Cluster<double>* other_cluster) override;
    void calc_mean_value() override;
  };
  struct DistCluster : public Cluster<Segment_2> {
    double distance(Cluster<Segment_2>* other_cluster) override;
    void calc_mean_value() override;
  };

  template <typename ClusterH>
  struct DistanceTable {
    using ClusterPair = std::pair<ClusterH, ClusterH>;

    // fibonacci heap from boost
    struct ClusterPairDist {
      ClusterPairDist(ClusterPair p, double d) : clusters(p), dist(d) {}
      ClusterPair clusters;
      double dist;

      bool operator<(ClusterPairDist const& rhs) const {
        return dist > rhs.dist;
      }
    };
    using DistanceHeap = boost::heap::fibonacci_heap<ClusterPairDist>;
    using heap_handle = typename DistanceHeap::handle_type;

    // define hash function such that the same hash results regardless of the
    // order of cluster handles in the pair struct KeyHash {
    //   size_t operator()(const ClusterPair& key) const {
    //     if (key.first.get() < key.second.get())
    //       return std::hash<ClusterH>()(key.first) ^
    //         (std::hash<ClusterH>()(key.second) << 1);
    //     else
    //       return std::hash<ClusterH>()(key.second) ^
    //         (std::hash<ClusterH>()(key.first) << 1);

    //   }
    // };
    // // True equality function is needed to deal with collisions
    // struct KeyEqual {
    //   bool operator()(const ClusterPair& lhs, const ClusterPair& rhs) const {
    //     return
    //       ((lhs.first==rhs.first) && (lhs.second==rhs.second))
    //       ||
    //       ((lhs.first==rhs.second) && (lhs.second==rhs.first));
    //   }
    // };
    using Cluster2DistPairMap = std::unordered_map<ClusterH, std::list<std::shared_ptr<heap_handle>>>;

    Cluster2DistPairMap cluster_to_dist_pairs;
    DistanceHeap distances;
    std::set<ClusterH>& clusters;

    DistanceTable(std::set<ClusterH>& clusters);  // computes initial distances
    void merge(
        ClusterH lhs,
        ClusterH rhs);  // merges two clusters, then removes one from the
                        // distances map and update the affected distances
    ClusterPairDist
    get_closest_pair();  // returns the cluster pair with the smallest distance
  };

  // extern template class Cluster<double>;
  // extern template class Cluster<Segment_2>;

  using AngleClusterH = std::shared_ptr<AngleCluster>;
  using DistClusterH = std::shared_ptr<DistCluster>;

  extern template class DistanceTable<AngleClusterH>;
  extern template class DistanceTable<DistClusterH>;

  double calc_mean_angle(const std::vector<linetype*>& lines);
  Point_2 calc_centroid(const std::vector<linetype*>& lines);
  Segment_2 calc_segment(Point_2 centroid, double mean_angle,
                         const std::vector<linetype*>& lines,
                         double extension = 0);

  struct linetype {
    linetype(Segment_2 segment_, double angle_, Point_2 midpoint_,
             double dist_in_ang_cluster_, size_t priority_, size_t segment_id_,
             double sqlength_)
        : segment(segment_),
          angle(angle_),
          midpoint(midpoint_),
          priority(priority_),
          segment_id(segment_id_),
          sqlength(sqlength_){};

    Segment_2 segment;
    double angle;
    Point_2 midpoint;
    Vector_2 direction;
    double sqlength;
    // double dist_in_ang_cluster;
    size_t priority;
    size_t segment_id;

    Segment_2 reg_segment;
    AngleClusterH angle_cluster;
    DistClusterH dist_cluster;
    size_t angle_cluster_id, dist_cluster_id;
  };

  static constexpr double pi = 3.14159265358979323846;
  class LineRegulariser {
    using SegmentVec = std::vector<Segment_2>;

    // roofer::SegmentCollection& input_segments;
   public:
    std::vector<linetype> lines;
    // SegmentVec input_reg_exact;
    double angle_threshold, dist_threshold;

    std::unordered_map<size_t, SegmentVec> segments;
    std::set<AngleClusterH> angle_clusters;
    std::set<DistClusterH> dist_clusters;

    LineRegulariser()= default;

    void add_segments(size_t priority, const Polygon_2& polygon,
                      double offset) {
      size_t i;
      if (segments.find(priority) == segments.end()) {
        i = 0;
      } else {
        i = segments.size();
      }
      auto orientation = polygon.orientation();
      for (auto edge = polygon.edges_begin(); edge != polygon.edges_end();
           ++edge) {
        auto source = edge->source();
        auto target = edge->target();
        auto perp = (target - source).perpendicular(orientation);
        auto len = CGAL::sqrt(CGAL::to_double(perp.squared_length()));
        // std::cout << "len: " << len << "\n";
        perp = offset * (perp / len);
        target -= perp;
        source -= perp;
        auto v = target - source;
        auto p_ = source + v / 2;
        auto p = Point_2(CGAL::to_double(p_.x()), CGAL::to_double(p_.y()));
        auto l = CGAL::to_double(v.squared_length());
        if (l < 0.001) continue;
        auto angle = std::atan2(CGAL::to_double(v.x()), CGAL::to_double(v.y()));
        if (angle < 0) angle += pi;
        lines.push_back(linetype(*edge, angle, p, 0, priority, i++, l));
        segments[priority].push_back(Segment_2(target, source));
      }
    }

    void add_segments(size_t priority, const roofer::SegmentCollection& segs) {
      if (segs.size() == 0) return;
      size_t i;
      if (segments.find(priority) == segments.end()) {
        i = 0;
      } else {
        i = segments.size();
      }

      for (auto& edge : segs) {
        auto source = Point_2(edge[0][0], edge[0][1]);
        auto target = Point_2(edge[1][0], edge[1][1]);
        auto v = target - source;
        auto p_ = source + v / 2;
        auto p = Point_2(CGAL::to_double(p_.x()), CGAL::to_double(p_.y()));
        auto l = CGAL::to_double(v.squared_length());
        if (l < 0.001) continue;
        auto angle = std::atan2(CGAL::to_double(v.x()), CGAL::to_double(v.y()));
        if (angle < 0) angle += pi;
        lines.push_back(
            linetype(Segment_2(source, target), angle, p, 0, priority, i++, l));
        segments[priority].push_back(Segment_2(source, target));
      }
    }

    SegmentVec& get_segments(const size_t& priority) {
      return segments[priority];
    }

    void perform_angle_clustering();
    void perform_distance_clustering();
  };
  template <class Kernel>
  void chain(const typename Kernel::Plane_3& plane,
             const typename Kernel::Segment_3& a,
             const typename Kernel::Segment_3& b,
             std::vector<typename Kernel::Point_3>& ring_pts,
             const float& snap_threshold, const float& line_extend) {
    typedef typename Kernel::Segment_2 Segment_2;
    typedef typename Kernel::Point_2 Point_2;
    typedef typename Kernel::Point_3 Point_3;
    typedef typename Kernel::Plane_3 Plane_3;

    auto a_2d = Segment_2(Point_2(a.source().x(), a.source().y()),
                          Point_2(a.target().x(), a.target().y()));
    auto b_2d = Segment_2(Point_2(b.source().x(), b.source().y()),
                          Point_2(b.target().x(), b.target().y()));

    auto l_a = a_2d.supporting_line();
    auto l_b = b_2d.supporting_line();
    Segment_2 s(a_2d.target(), b_2d.source());
    auto result = CGAL::intersection(l_a, l_b);
    if (result) {
      if (auto p = boost::get<Point_2>(&*result)) {
        if (CGAL::squared_distance(*p, s) < snap_threshold) {
          double z = -plane.a() / plane.c() * p->x() -
                     plane.b() / plane.c() * p->y() - plane.d() / plane.c();
          ring_pts.push_back(Point_3(p->x(), p->y(), z));
          // ring_pts.push_back( plane.to_3d(*p) );
        } else {
          // undo any previously applied line extension prior to connected the
          // endpoints
          auto va = (a.target() - a.source());
          va = va / CGAL::sqrt(va.squared_length());
          auto vb = (b.source() - b.target());
          vb = vb / CGAL::sqrt(vb.squared_length());

          ring_pts.push_back(a.target() - va * line_extend);
          ring_pts.push_back(b.source() - vb * line_extend);
        }
      }
      // } else if (auto l = boost::get<K::Line_2>(&*result)) {
    } else {  // there is no intersection
      // undo any previously applied line extension prior to connected the
      // endpoints
      auto va = (a.target() - a.source());
      va = va / CGAL::sqrt(va.squared_length());
      auto vb = (b.source() - b.target());
      vb = vb / CGAL::sqrt(vb.squared_length());

      ring_pts.push_back(a.target() - va * line_extend);
      ring_pts.push_back(b.source() - vb * line_extend);
    }
  }

  // void chain(Segment& a, Segment& b, LinearRing& ring, const float&
  // snap_threshold) {
  template <class Kernel>
  inline void check_dist(std::vector<typename Kernel::Point_3>& iring,
                         std::vector<typename Kernel::Point_3>& aring,
                         const size_t a, const size_t b) {
    auto d = CGAL::squared_distance(iring[a], iring[b]);
    if (d > 1E-6) aring.push_back(iring[a]);
  }

  // template<class Kernel> CGAL::Polygon_2<Kernel>
  template <class Kernel>
  std::vector<typename Kernel::Point_3> chain_ring(
      const std::vector<size_t>& idx, const typename Kernel::Plane_3& plane,
      const std::vector<typename Kernel::Segment_3>& segments,
      const float& snap_threshold, const float& line_extend) {
    std::vector<typename Kernel::Point_3> ring_points, new_ring_points;

    if (idx.size() > 1) {  // we need at least 2 segments
      for (size_t i = 1; i < idx.size(); ++i) {
        chain<Kernel>(plane, segments[idx[i - 1]], segments[idx[i]],
                      ring_points, snap_threshold, line_extend);
      }
      chain<Kernel>(plane, segments[idx[idx.size() - 1]], segments[idx[0]],
                    ring_points, snap_threshold, line_extend);

      // get rid of segments with zero length (ie duplicate points)
      // check again the size, to ignore degenerate case of input ring that
      // consists of 3 co-linear segments (would get chained to eg 0 vertices)
      if (ring_points.size() > 2) {
        for (size_t i = 1; i < ring_points.size(); ++i) {
          check_dist<Kernel>(ring_points, new_ring_points, i - 1, i);
        }
        check_dist<Kernel>(ring_points, new_ring_points, ring_points.size() - 1,
                           0);
      }

      // NOTE: at this point there can still be vertices between co-linear
      // segments (ie 3 consecutive points on the same line)
    }

    return new_ring_points;
  }
}  // namespace roofer::linereg
