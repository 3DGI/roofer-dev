#include <array>
#include <initializer_list>

namespace roofer {

  template <typename T>
  struct TBox {
    std::array<T, 3> pmin, pmax;
    bool just_cleared;

    TBox() { clear(); };

    TBox(std::initializer_list<T> initList) {
      clear();
      auto it = initList.begin();
      pmin[0] = *it++;
      pmin[1] = *it++;
      pmin[2] = *it++;
      pmax[0] = *it++;
      pmax[1] = *it++;
      pmax[2] = *it;
      just_cleared = false;
    }

    std::array<T, 3> min() const { return pmin; };
    std::array<T, 3> max() const { return pmax; };
    T size_x() const { return pmax[0] - pmin[0]; };
    T size_y() const { return pmax[1] - pmin[1]; };
    void set(std::array<T, 3> nmin, std::array<T, 3> nmax) {
      pmin = nmin;
      pmax = nmax;
      just_cleared = false;
    };
    void add(T p[]) {
      if (just_cleared) {
        pmin[0] = p[0];
        pmin[1] = p[1];
        pmin[2] = p[2];
        pmax[0] = p[0];
        pmax[1] = p[1];
        pmax[2] = p[2];
        just_cleared = false;
      }
      pmin[0] = std::min(p[0], pmin[0]);
      pmin[1] = std::min(p[1], pmin[1]);
      pmin[2] = std::min(p[2], pmin[2]);
      pmax[0] = std::max(p[0], pmax[0]);
      pmax[1] = std::max(p[1], pmax[1]);
      pmax[2] = std::max(p[2], pmax[2]);
    };
    void add(std::array<T, 3> a) { add(a.data()); };
    void add(const TBox& otherBox) {
      add(otherBox.min());
      add(otherBox.max());
    };
    void add(TBox& otherBox) {
      add(otherBox.min());
      add(otherBox.max());
    };
    ;
    void add(const std::vector<std::array<T, 3>>& vec) {
      for (auto& p : vec) add(p);
    };
    void add(std::vector<std::array<T, 3>>& vec) {
      for (auto& p : vec) add(p);
    };
    bool intersects(const TBox& otherBox) const {
      bool intersect_x =
          (pmin[0] < otherBox.pmax[0]) && (pmax[0] > otherBox.pmin[0]);
      bool intersect_y =
          (pmin[1] < otherBox.pmax[1]) && (pmax[1] > otherBox.pmin[1]);
      return intersect_x && intersect_y;
    };
    void clear() {
      pmin.fill(0);
      pmax.fill(0);
      just_cleared = true;
    };
    bool isEmpty() const { return just_cleared; };
    std::array<T, 3> center() const {
      return {(pmax[0] + pmin[0]) / 2, (pmax[1] + pmin[1]) / 2,
              (pmax[2] + pmin[2]) / 2};
    };
  };

  typedef TBox<float> Box;
}  // namespace roofer
