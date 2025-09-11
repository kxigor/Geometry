#pragma once

#include <algorithm>
#include <cmath>
#include <iostream>
#include <ranges>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "Double/Double.hpp"

namespace geometry3d {
using ScalarT = Double<long double, 1e-10L>;

class Vector {
 public:
  /*============================= Usings ==============================*/
  using CoordTuple = std::tuple<ScalarT, ScalarT, ScalarT>;

  /*============================= Helpers =============================*/
  struct Hash {
    std::size_t operator()(const Vector& vector) const noexcept {
      std::size_t seed = 0;
      hash_combine(seed, vector.x_);
      hash_combine(seed, vector.y_);
      hash_combine(seed, vector.z_);
      return seed;
    }

   private:
    static void hash_combine(std::size_t& seed, const ScalarT& v) {
      seed ^= std::hash<ScalarT>{}(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
  };

  /*==================== Constructors/Destructors =====================*/
  constexpr Vector() = default;

  constexpr Vector(const Vector&) = default;

  constexpr Vector(Vector&&) = default;

  constexpr Vector(ScalarT x, ScalarT y, ScalarT z) : x_(x), y_(y), z_(z) {}

  constexpr Vector(const CoordTuple& coord)
      : x_(std::get<0>(coord)),
        y_(std::get<1>(coord)),
        z_(std::get<2>(coord)) {}

  constexpr ~Vector() = default;
  /*=========================== Assignments ===========================*/
  constexpr Vector& operator=(const Vector& /*unused*/) = default;
  constexpr Vector& operator=(Vector&& /*unused*/) = default;

  /*============================ Operators ============================*/
  friend std::istream& operator>>(std::istream& in, Vector& vector) {
    in >> vector.x_ >> vector.y_ >> vector.z_;
    return in;
  }

  friend std::ostream& operator<<(std::ostream& out, const Vector& vector) {
    out << vector.x_ << ' ' << vector.y_ << ' ' << vector.z_;
    return out;
  }

  constexpr auto operator<=>(const Vector& other) const noexcept {
    return std::tie(x_, y_, z_) <=> std::tie(other.x_, other.y_, other.z_);
  }

  constexpr bool operator==(const Vector& other) const noexcept {
    return (*this <=> other) == std::strong_ordering::equal;
  }

  [[nodiscard]] constexpr Vector operator-() const noexcept {
    return {-x_, -y_, -z_};
  }

  constexpr Vector& operator+=(const Vector& other) noexcept {
    x_ += other.x_;
    y_ += other.y_;
    z_ += other.z_;
    return *this;
  }

  [[nodiscard]] constexpr Vector operator+(const Vector& other) const noexcept {
    Vector temp = *this;
    return temp += other;
  }

  constexpr Vector& operator-=(const Vector& other) noexcept {
    x_ -= other.x_;
    y_ -= other.y_;
    z_ -= other.z_;
    return *this;
  }

  [[nodiscard]] constexpr Vector operator-(const Vector& other) const noexcept {
    Vector temp = *this;
    return temp -= other;
  }

  constexpr Vector& operator*=(const ScalarT& scalar) noexcept {
    x_ *= scalar;
    y_ *= scalar;
    z_ *= scalar;
    return *this;
  }

  [[nodiscard]] constexpr Vector operator*(
      const ScalarT& scalar) const noexcept {
    Vector temp = *this;
    return temp *= scalar;
  }

  constexpr Vector& operator/=(const ScalarT& scalar) noexcept {
    x_ /= scalar;
    y_ /= scalar;
    z_ /= scalar;
    return *this;
  }

  [[nodiscard]] constexpr Vector operator/(
      const ScalarT& scalar) const noexcept {
    Vector temp = *this;
    return temp /= scalar;
  }

  /*============================ Geometry =============================*/
  [[nodiscard]] constexpr ScalarT Length() const noexcept {
    using std::sqrt;
    return sqrt(ScalarProduct(*this));
  }

  [[nodiscard]] constexpr ScalarT SquaredLength() const noexcept {
    return ScalarProduct(*this);
  }

  [[nodiscard]] constexpr ScalarT ScalarProduct(
      const Vector& other) const noexcept {
    return (x_ * other.x_) + (y_ * other.y_) + (z_ * other.z_);
  }

  [[nodiscard]] constexpr Vector VectorProduct(
      const Vector& other) const noexcept {
    return Vector(y_ * other.z_ - z_ * other.y_, z_ * other.x_ - x_ * other.z_,
                  x_ * other.y_ - y_ * other.x_);
  }

  constexpr Vector& Normalize() noexcept {
    const auto kLength = Length();
    x_ /= kLength;
    y_ /= kLength;
    z_ /= kLength;
    return *this;
  }

  [[nodiscard]] constexpr Vector GetNormalized() const noexcept {
    auto tmp = *this;
    return tmp.Normalize();
  }

  /*============================= Getters =============================*/
  [[nodiscard]] constexpr const auto& GetX() const noexcept { return x_; }

  [[nodiscard]] constexpr const auto& GetY() const noexcept { return y_; }

  [[nodiscard]] constexpr const auto& GetZ() const noexcept { return z_; }

  [[nodiscard]] constexpr auto GetAsTuple() const noexcept {
    return std::tie(x_, y_, z_);
  }

 private:
  ScalarT x_{};
  ScalarT y_{};
  ScalarT z_{};
};

using Point = Vector;
using PointsT = std::vector<Point>;

[[nodiscard]] constexpr Vector CalcPlaneNormal(const Point& p1, const Point& p2,
                                               const Point& p3) noexcept {
  return (p2 - p1).VectorProduct(p3 - p1).Normalize();
}

[[nodiscard]] constexpr Vector CalcPlaneNormal(const Point& a, const Point& b,
                                               const Point& c,
                                               const Point& centroid) noexcept {
  Vector test_normal = (b - a).VectorProduct(c - a);
  if (test_normal.ScalarProduct(centroid - a) > ScalarT(0)) {
    test_normal = -test_normal;
  }
  return test_normal.Normalize();
}

class Line {
 public:
  /*==================== Constructors/Destructors =====================*/
  constexpr Line() = delete;

  constexpr Line(const Line& /*unused*/) = default;

  constexpr Line(Line&& /*unused*/) = default;

  constexpr Line(const Point& origin, const Point& direction)
      : origin_(origin), direction_(direction) {}

  constexpr ~Line() = default;

  /*=========================== Assignments ===========================*/
  constexpr Line& operator=(const Line& /*unused*/) = default;

  constexpr Line& operator=(Line&& /*unused*/) = default;

  /*============================ Getters ==============================*/
  [[nodiscard]] constexpr const auto& GetOrigin() const noexcept {
    return origin_;
  }

  [[nodiscard]] constexpr const auto& GetDirection() const noexcept {
    return direction_;
  }

 private:
  /*============================= Fields ==============================*/
  Point origin_;
  Point direction_;
};

class Plane {
 public:
  /*==================== Constructors/Destructors =====================*/
  constexpr Plane(const Point& origin, const Vector& normal)
      : origin_(origin), normal_(normal) {}

  constexpr Plane(const Point& p1, const Point& p2, const Point& p3)
      : origin_(p1), normal_(CalcPlaneNormal(p1, p2, p3)) {}

  constexpr ~Plane() = default;

  /*============================ Getters ==============================*/
  [[nodiscard]] constexpr const auto& GetOrigin() const noexcept {
    return origin_;
  }

  [[nodiscard]] constexpr const auto& GetNormal() const noexcept {
    return normal_;
  }

 private:
  /*============================= Fields ==============================*/
  Point origin_;
  Vector normal_;
};

static constexpr const Plane Oxy = {{0, 0, 0}, {0, 0, -1}};
static constexpr const Plane Oxz = {{0, 0, 0}, {0, -1, 0}};
static constexpr const Plane Oyz = {{0, 0, 0}, {-1, 0, 0}};

class Face {
 public:
  /*==================== Constructors/Destructors =====================*/
  Face() = delete;

  Face(const Face& /*unused*/) = default;

  Face(Face&& /*unused*/) = default;

  ~Face() = default;

  Face(const Point& p, const Point& q, const Point& r) : p_(p), q_(q), r_(r) {
    normal_ = CalcPlaneNormal(p, q, r);
  }

  /*=========================== Assignments ===========================*/
  Face& operator=(const Face& /*unused*/) = default;

  Face& operator=(Face&& /*unused*/) = default;

  /*============================= Getters =============================*/
  [[nodiscard]] const auto& GetP() const noexcept { return p_; }

  [[nodiscard]] const auto& GetQ() const noexcept { return q_; }

  [[nodiscard]] const auto& GetR() const noexcept { return r_; }

  [[nodiscard]] const auto& GetNormal() const noexcept { return normal_; }

 private:
  /*============================= Fields ==============================*/
  Point p_;
  Point q_;
  Point r_;
  Vector normal_;
};

using FacesT = std::vector<Face>;

class Segment {
 public:
  /*============================= Helpers =============================*/
  struct Hash {
    std::size_t operator()(const Segment& e) const noexcept {
      auto hash1 = Point::Hash{}(e.a_);
      auto hash2 = Point::Hash{}(e.b_);
      return hash1 ^ hash2;
    }
  };

  /*==================== Constructors/Destructors =====================*/
  Segment() = delete;

  Segment(const Segment& /*unused*/) = default;

  Segment(Segment& /*unused*/) = default;

  ~Segment() = default;

  Segment(const Point& A, const Point& B) : a_(A), b_(B) {}

  /*=========================== Assignments ===========================*/
  Segment& operator=(const Segment& /*unused*/) = default;

  Segment& operator=(Segment&& /*unused*/) = default;

  /*============================ Operators ============================*/
  [[nodiscard]] bool operator==(const Segment& other) const noexcept {
    return (a_ == other.a_ && b_ == other.b_) ||
           (a_ == other.b_ && b_ == other.a_);
  }

  /*============================ Geometry =============================*/
  [[nodiscard]] bool IsDegenerate() const noexcept { return a_ == b_; }

  /*============================= Getters =============================*/
  [[nodiscard]] const auto& GetA() const noexcept { return a_; }

  [[nodiscard]] const auto& GetB() const noexcept { return b_; }

  [[nodiscard]] auto GetAsTuple() const noexcept { return std::tie(a_, b_); }

 private:
  /*============================= Fields ==============================*/
  Point a_;
  Point b_;
};

using EdgesT = std::vector<Segment>;

class ConvexHull {
  /*============================= Usings ==============================*/
  using EdgeT = Segment;
  using OpenEdgesT = std::vector<EdgeT>;

  using EdgesFaceT = std::unordered_map<EdgeT, const Face, EdgeT::Hash>;
  using ClosedEdgesT = std::unordered_set<EdgeT, EdgeT::Hash>;

  using PointsSetT = std::unordered_set<Point, Point::Hash>;

  /*============================= Helpers =============================*/
  struct ZPred {
    bool operator()(const Point& lhs, const Point& rhs) const noexcept;
  };

  struct NormalAnglePred {
    NormalAnglePred(const Point& p1, const Point& p2, const Vector& normal)
        : p1_(p1), p2_(p2), normal_(normal) {}

    bool operator()(const Point& lhs, const Point& rhs) const;

   private:
    const Point& p1_;
    const Point& p2_;
    const Vector& normal_;
  };

 public:
  /*==================== Constructors/Destructors =====================*/
  ConvexHull() = delete;

  ~ConvexHull() = default;

  ConvexHull(PointsT points);

  /*============================ Geometry =============================*/
  [[nodiscard]] ScalarT CalcDistance(const Point& point) const noexcept;

  /*============================= Getters =============================*/
  [[nodiscard]] const FacesT& GetFaces() const noexcept;

 private:
  void DeleteNonUnique();
  void Initializate();

  [[nodiscard]] Point GetFirstPoint() const noexcept;
  [[nodiscard]] Point GetSecondPoint(const Point& P) const noexcept;
  [[nodiscard]] Point GetThirdPoint(const Point& P, const Point& Q) const;

  void Scan();

  void AddEdge(const EdgeT& edge, const Face& face);
  Face& AddFace(const EdgeT& edge, const Point& point);

  /*============================== Impls ==============================*/
  [[nodiscard]] Point GetMinNoTrashPoints(auto pred,
                                          const auto&... points) const {
    auto filtered_points =
        points_ | std::views::filter([&](const Point& point) {
          return ((point != points) && ...);
        });
    return *std::ranges::min_element(filtered_points, pred);
  }

  /*============================= Fields ==============================*/
  PointsT points_;
  OpenEdgesT open_edges_;
  EdgesFaceT edges_face_;
  ClosedEdgesT closed_edges_;
  FacesT faces_;
};

[[nodiscard]] bool IsIntersect(const Face& face, const Point& p) noexcept;
[[nodiscard]] bool IsIntersect(const Point& p, const Face& face) noexcept;

[[nodiscard]] ScalarT CalcDistance(const Point& p, const Segment& seg) noexcept;
[[nodiscard]] ScalarT CalcDistance(const Segment& seg, const Point& p) noexcept;
[[nodiscard]] ScalarT CalcDistance(const Face& face, const Point& p) noexcept;
[[nodiscard]] ScalarT CalcDistance(const Point& p, const Face& face) noexcept;

[[nodiscard]] ScalarT AngleBetweenVectors(const Vector& a, const Vector& b);

};  // namespace geometry3d