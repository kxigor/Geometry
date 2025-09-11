#pragma once

#include <algorithm>
#include <bitset>
#include <cassert>
#include <cmath>
#include <concepts>
#include <iostream>
#include <limits>
#include <optional>
#include <ranges>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

#include "Double/Double.hpp"

namespace geometry {

using ScalarT = Double<long double, 1e-10L>;

class Vector {
 public:
  /*============================= Usings ==============================*/
  using CoordPairT = std::pair<ScalarT, ScalarT>;
  using ScalarTupleT = std::tuple<ScalarT, ScalarT>;
  using HashT = std::size_t;

  /*==================== Constructors/Destructors =====================*/
  Vector() noexcept = default;
  ~Vector() = default;

  Vector(const Vector& /*unused*/) noexcept = default;
  Vector(Vector&& /*unused*/) noexcept = default;

  Vector(const ScalarT& x, const ScalarT& y) noexcept : x_(x), y_(y) {}
  Vector(const CoordPairT& coord) noexcept
      : x_(coord.first), y_(coord.second) {}
  Vector(const ScalarTupleT& coord) noexcept
      : x_(std::get<0>(coord)), y_(std::get<1>(coord)) {}
  Vector(std::initializer_list<ScalarT> init) {
    if (init.size() != 2) {
      throw std::invalid_argument(
          "Initializer list must contain exactly 2 elements");
    }
    x_ = *init.begin(), y_ = *(init.begin() + 1);
  }

  /*=========================== Assignments ===========================*/
  Vector& operator=(const Vector& /*unused*/) noexcept = default;
  Vector& operator=(Vector&& /*unused*/) noexcept = default;

  /*============================ Operators ============================*/
  friend std::istream& operator>>(std::istream& input, Vector& vector) {
    input >> vector.x_ >> vector.y_;
    return input;
  }

  friend std::ostream& operator<<(std::ostream& output, const Vector& vector) {
    output << vector.x_ << ' ' << vector.y_;
    return output;
  }

  auto operator<=>(const Vector& other) const noexcept = default;

  bool operator==(const Vector& other) const noexcept {
    return (*this <=> other) == std::strong_ordering::equal;
  }

  [[nodiscard]] Vector operator-() const noexcept { return {-x_, -y_}; }

  Vector& operator+=(const Vector& other) noexcept {
    x_ += other.x_;
    y_ += other.y_;
    return *this;
  }

  [[nodiscard]] Vector operator+(const Vector& other) const noexcept {
    Vector temp = *this;
    return temp += other;
  }

  Vector& operator-=(const Vector& other) noexcept {
    x_ -= other.x_;
    y_ -= other.y_;
    return *this;
  }

  [[nodiscard]] Vector operator-(const Vector& other) const noexcept {
    Vector temp = *this;
    return temp -= other;
  }

  Vector& operator*=(const ScalarT& scalar) noexcept {
    x_ *= scalar;
    y_ *= scalar;
    return *this;
  }

  [[nodiscard]] Vector operator*(const ScalarT& scalar) const noexcept {
    Vector temp = *this;
    return temp *= scalar;
  }

  Vector& operator/=(const ScalarT& scalar) noexcept {
    x_ /= scalar;
    y_ /= scalar;
    return *this;
  }

  [[nodiscard]] Vector operator/(const ScalarT& scalar) const noexcept {
    Vector temp = *this;
    return temp /= scalar;
  }

  /*============================ Geometry =============================*/
  [[nodiscard]] ScalarT Length() const noexcept {
    using std::sqrt;
    return sqrt(ScalarProduct(*this));
  }

  [[nodiscard]] ScalarT SquaredLength() const noexcept {
    return ScalarProduct(*this);
  }

  [[nodiscard]] ScalarT ScalarProduct(const Vector& other) const noexcept {
    return (x_ * other.x_) + (y_ * other.y_);
  }

  [[nodiscard]] ScalarT VectorProduct(const Vector& other) const noexcept {
    return (x_ * other.y_) - (y_ * other.x_);
  }

  Vector& Normalize() noexcept {
    const auto kLength = Length();
    x_ /= kLength;
    y_ /= kLength;
    return *this;
  }

  [[nodiscard]] Vector GetNormalized() const noexcept {
    auto tmp = *this;
    return tmp.Normalize();
  }

  /*============================= Getters =============================*/
  [[nodiscard]] const ScalarT& GetX() const noexcept { return x_; }

  [[nodiscard]] const ScalarT& GetY() const noexcept { return y_; }

  [[nodiscard]] CoordPairT GetAsPair() const noexcept { return {x_, y_}; }

  [[nodiscard]] ScalarTupleT GetAsTuple() const noexcept { return {x_, y_}; }

  /*============================== Hash ===============================*/
  struct Hash {
    std::size_t operator()(const Vector& vector) const {
      HashT seed = 0;
      seed ^= std::hash<ScalarT>{}(vector.x_) + 0x9e3779b9 + (seed << 6) +
              (seed >> 2);
      seed ^= std::hash<ScalarT>{}(vector.y_) + 0x9e3779b9 + (seed << 6) +
              (seed >> 2);
      return seed;
    }
  };

 private:
  /*============================= Fields ==============================*/
  ScalarT x_{};
  ScalarT y_{};
};

using Point = Vector;
using PointsT = std::vector<Point>;

enum class Orientation : char { Clockwise, CounterClockwise, Collinear };

class Segment {
 public:
  /*============================= Usings ==============================*/
  using PointPairT = std::pair<Point, Point>;
  using PointTupleT = std::tuple<Point, Point>;

  /*==================== Constructors/Destructors =====================*/
  Segment() noexcept = default;
  ~Segment() = default;

  Segment(const Segment& /*unused*/) noexcept = default;
  Segment(Segment&& /*unused*/) noexcept = default;

  Segment(const Point& a, const Point& b) noexcept : a_(a), b_(b) {}
  explicit Segment(const PointPairT& points) noexcept
      : a_(points.first), b_(points.second) {}
  explicit Segment(const PointTupleT& points) noexcept
      : a_(std::get<0>(points)), b_(std::get<1>(points)) {}

  /*=========================== Assignments ===========================*/
  Segment& operator=(const Segment& /*unused*/) noexcept = default;

  Segment& operator=(Segment&& /*unused*/) noexcept = default;

  /*============================ Operators ============================*/
  friend std::istream& operator>>(std::istream& input, Segment& seg) {
    input >> seg.a_ >> seg.b_;
    return input;
  }

  friend std::ostream& operator<<(std::ostream& output, const Segment& seg) {
    output << seg.a_ << ' ' << seg.b_;
    return output;
  }

  /*============================ Geometry =============================*/
  [[nodiscard]] bool IsDegenerate() const noexcept { return a_ == b_; }

  /*============================= Getters =============================*/
  [[nodiscard]] const Point& GetPointA() const noexcept { return a_; }

  [[nodiscard]] const Point& GetPointB() const noexcept { return b_; }

  [[nodiscard]] PointPairT GetAsPair() const noexcept { return {a_, b_}; }

  [[nodiscard]] PointTupleT GetAsTuple() const noexcept { return {a_, b_}; }

  [[nodiscard]] Point GetVector() const noexcept { return b_ - a_; }

 private:
  /*============================= Feilds ==============================*/
  Point a_;
  Point b_;
};

class Line {
 public:
  /*============================= Usings ==============================*/
  using LineTupleT = std::tuple<ScalarT, ScalarT, ScalarT>;

  /*==================== Constructors/Destructors =====================*/
  Line() = delete;

  Line(const Line& /*unused*/) noexcept = default;

  Line(Line&& /*unused*/) noexcept = default;

  Line(ScalarT a, ScalarT b, ScalarT c) : a_(a), b_(b), c_(c) {
    if (a_ == ScalarT(0) && b_ == ScalarT(0)) {
      throw std::runtime_error("Invalid line params (a or b must be nonzero)");
    }
  }

  /*=========================== Assignments ===========================*/
  Line& operator=(const Line& /*unused*/) noexcept = default;

  Line& operator=(Line&& /*unused*/) noexcept = default;

  /*============================ Geometry =============================*/
  [[nodiscard]] const ScalarT& GetPointA() const noexcept { return a_; }

  [[nodiscard]] const ScalarT& GetPointB() const noexcept { return b_; }

  [[nodiscard]] const ScalarT& GetPointC() const noexcept { return c_; }

  [[nodiscard]] LineTupleT GetAsTuple() const noexcept { return {a_, b_, c_}; }

 private:
  /*============================= Fields ==============================*/
  ScalarT a_{};
  ScalarT b_{};
  ScalarT c_{};
};

class Ray {
 public:
  /*============================ Geometry =============================*/
  Ray(const Point& origin, const Point& direction) noexcept
      : origin_(origin), direction_(direction) {}

  /*============================= Getters =============================*/
  [[nodiscard]] const Point& GetOrigin() const noexcept { return origin_; }

  [[nodiscard]] const Point& GetDirection() const noexcept {
    return direction_;
  }

  /*===================== Non-member constructors =====================*/
  [[nodiscard]] static Ray FromPoints(const Point& start,
                                      const Point& through) noexcept {
    return {start, through - start};
  }

 private:
  /*============================ Geometry =============================*/
  Point origin_;
  Point direction_;
};

class HalfPlane {
 public:
  /*==================== Constructors/Destructors =====================*/
  HalfPlane() noexcept = default;

  HalfPlane(const HalfPlane& /*unused*/) noexcept = default;

  HalfPlane(HalfPlane&& /*unused*/) noexcept = default;

  ~HalfPlane() = default;

  HalfPlane(ScalarT a, ScalarT b, ScalarT c) noexcept : a_(a), b_(b), c_(c) {}

  HalfPlane(const Point& a, const Point& b,
            const Point& inside_point) noexcept {
    a_ = b.GetY() - a.GetY();
    b_ = a.GetX() - b.GetX();
    c_ = -a_ * a.GetX() - b_ * a.GetY();
    NormalizeSignByInsidePoint(inside_point);
  }
  /*=========================== Assginments ===========================*/
  HalfPlane& operator=(const HalfPlane& /*unused*/) noexcept = default;

  HalfPlane& operator=(HalfPlane&& /*unused*/) noexcept = default;

  /*============================= Getters =============================*/
  [[nodiscard]] const auto& GetA() const noexcept { return a_; }

  [[nodiscard]] const auto& GetB() const noexcept { return b_; }

  [[nodiscard]] const auto& GetC() const noexcept { return c_; }

  [[nodiscard]] auto GetAsTuple() const noexcept {
    return std::tie(a_, b_, c_);
  }

  /*====================== Non-member functions =======================*/
  static HalfPlane BuildAsPerpendicularToSegment(
      const Segment& segment, const Point& inside_point) noexcept {
    const Point midpoint =
        (segment.GetPointA() + segment.GetPointB()) / ScalarT(2);
    const Vector AB = segment.GetPointB() - segment.GetPointA();
    const ScalarT a = AB.GetX();
    const ScalarT b = AB.GetY();
    const ScalarT c = -(a * midpoint.GetX() + b * midpoint.GetY());
    return HalfPlane(a, b, c).NormalizeSignByInsidePoint(inside_point);
  }

 private:
  /*============================== Impls ==============================*/
  HalfPlane& NormalizeSignByInsidePoint(const Point& target) noexcept {
    if (a_ * target.GetX() + b_ * target.GetY() + c_ < ScalarT(0)) {
      a_ = -a_, b_ = -b_, c_ = -c_;
    }
    return *this;
  }

  /*============================= Fields ==============================*/
  ScalarT a_{}, b_{}, c_{};
};

using HalfPlanesT = std::vector<HalfPlane>;

class GrahamsScan {
 public:
  /*============================= Helpers =============================*/
  static constexpr const Orientation kOrientation =
      Orientation::CounterClockwise;

  struct PolarAnglePredicat {
    PolarAnglePredicat() = delete;
    ~PolarAnglePredicat() = default;

    PolarAnglePredicat(const PolarAnglePredicat&) noexcept = default;
    PolarAnglePredicat(PolarAnglePredicat&&) noexcept = default;

    explicit PolarAnglePredicat(const Point& pivot) noexcept : pivot_(pivot) {}

    PolarAnglePredicat& operator=(const PolarAnglePredicat&) noexcept = default;
    PolarAnglePredicat& operator=(PolarAnglePredicat&&) noexcept = default;

    [[nodiscard]] bool operator()(const Point& p1,
                                  const Point& p2) const noexcept;

   private:
    Point pivot_;
  };

  /*==================== Constructors/Destructors =====================*/
  explicit GrahamsScan(PointsT points);

  /*============================= Getters =============================*/
  [[nodiscard]] const PointsT& GetConvexHull() const noexcept;
  [[nodiscard]] PointsT&& TakeConvexHull() noexcept;

  /*============================= Helpers =============================*/
  static void SortCCW(PointsT& points, const Point& pivot);
  [[nodiscard]] static Point GetPivot(const PointsT& points) noexcept;

 private:
  /*============================= Implds ==============================*/
  void Scan(PointsT points);
  static void DeleteDublicates(PointsT& points);

  /*============================= Feilds ==============================*/
  PointsT convex_hull_;
};

class ConvexHull {
 public:
  /*=================== Usings/Constnts/HelpClasses ===================*/
  using PointIteratorT = PointsT::const_iterator;
  using SizeT = PointsT::size_type;
  static constexpr const Orientation kConvexHullOrientation =
      GrahamsScan::kOrientation;

  class EdgeIterator {
   public:
    EdgeIterator(const ConvexHull& convex_hull, SizeT pos = 0) noexcept
        : convex_hull_(&convex_hull), pos_(pos) {}

    EdgeIterator(const EdgeIterator& /*unused*/) noexcept = default;

    EdgeIterator(EdgeIterator&& /*unused*/) noexcept = default;

    EdgeIterator& operator=(const EdgeIterator& /*unused*/) noexcept = default;

    EdgeIterator& operator=(EdgeIterator&& /*unused*/) noexcept = default;

    const Segment& operator*() noexcept {
      UpdateCurrentEdge();
      return current_edge_;
    }

    const Segment* operator->() noexcept {
      UpdateCurrentEdge();
      return &current_edge_;
    }

    EdgeIterator& operator++() noexcept {
      ++pos_;
      return *this;
    }

    EdgeIterator operator++(int) noexcept {
      auto tmp = *this;
      ++*this;
      return tmp;
    }

    EdgeIterator& operator--() noexcept {
      --pos_;
      return *this;
    }

    EdgeIterator operator--(int) noexcept {
      auto tmp = *this;
      --*this;
      return tmp;
    }

    auto operator<=>(const EdgeIterator& other) const noexcept {
      return pos_ <=> other.pos_;
    }

    bool operator==(const EdgeIterator& other) const noexcept {
      return pos_ == other.pos_;
    }

   private:
    void UpdateCurrentEdge() noexcept {
      SizeT i = pos_;
      SizeT j = (pos_ + 1) % convex_hull_->GetSize();
      current_edge_ = {convex_hull_->points_[i], convex_hull_->points_[j]};
    }

    const ConvexHull* convex_hull_;
    SizeT pos_{};
    Segment current_edge_;
  };

  /*==================== Constructors/Destructors =====================*/
  ConvexHull() = default;
  ~ConvexHull() = default;

  ConvexHull(const ConvexHull& /*unused*/) = default;
  ConvexHull(ConvexHull&& /*unused*/) = default;

  ConvexHull(PointsT points);

  /*=========================== Assignments ===========================*/
  ConvexHull& operator=(const ConvexHull& /*unused*/) = default;
  ConvexHull& operator=(ConvexHull&& /*unused*/) = default;

  /*============================ Geometry =============================*/
  [[nodiscard]] ScalarT GetArea() const noexcept;

  [[nodiscard]] bool IsDegenerate() const noexcept;

  [[nodiscard]] Point GetCentroid() const noexcept;

  /*============================= Getters =============================*/
  [[nodiscard]] const PointsT& GetPoints() const noexcept;
  [[nodiscard]] PointsT&& TakePoints() noexcept;

  /*============================= LookUp ==============================*/
  [[nodiscard]] Point& operator[](SizeT pos) noexcept;

  [[nodiscard]] const Point& operator[](SizeT pos) const noexcept;

  [[nodiscard]] SizeT GetSize() const noexcept;

  [[nodiscard]] bool IsEmpty() const noexcept;

  /*============================ Iterators ============================*/
  [[nodiscard]] PointIteratorT begin() const noexcept;  // NOLINT
  [[nodiscard]] PointIteratorT end() const noexcept;    // NOLINT

  [[nodiscard]] PointIteratorT PointBegin() const noexcept;
  [[nodiscard]] PointIteratorT PointEnd() const noexcept;

  [[nodiscard]] EdgeIterator EdgeBegin() const noexcept;
  [[nodiscard]] EdgeIterator EdgeEnd() const noexcept;

  /*===================== Non-member constructors =====================*/
  [[nodiscard]] static ConvexHull InitViaArbitraryPoints(PointsT points);
  [[nodiscard]] static ConvexHull InitViaUnorderedConvexHull(PointsT points);
  [[nodiscard]] static ConvexHull InitViaUnorderedConvexHullWithDuplicates(
      PointsT points);
  [[nodiscard]] static ConvexHull InitDirect(PointsT points);

 private:
  /*============================= Feilds ==============================*/
  PointsT points_;
};

class VoronoiDiagram {
 public:
  /*============================= Usings ==============================*/
  using CellT = ConvexHull;
  using CellsT = std::vector<CellT>;

  /*==================== Constructors/Destructors =====================*/
  VoronoiDiagram(const PointsT& borders, PointsT points);

  /*============================= LookUp ==============================*/
  [[nodiscard]] const CellsT& GetCells() const noexcept;
  [[nodiscard]] CellsT&& TakeCells() noexcept { return std::move(cells_); }

 private:
  /*============================== Impls ==============================*/
  [[nodiscard]] HalfPlanesT GenerateHalfPlanes(const Point& point);

  /*============================= Feilds ==============================*/
  CellsT cells_;
  PointsT points_;
};

[[nodiscard]] ScalarT CrossProduct(const Point& a, const Point& b,
                                   const Point& c) noexcept;

[[nodiscard]] Ray RotateRayRadian(const Ray& ray, ScalarT angle_radians,
                                  bool clockwise = false) noexcept;

[[nodiscard]] Ray RotateRayDegree(const Ray& ray, ScalarT angle_degrees,
                                  bool clockwise = false) noexcept;

[[nodiscard]] Orientation CalcOrientation(const Point& a, const Point& b,
                                          const Point& c) noexcept;

[[nodiscard]] bool IsIntersect(const Point& a, const Point& b) noexcept;

[[nodiscard]] bool IsIntersect(const Point& p, const Segment& seg) noexcept;

[[nodiscard]] bool IsIntersect(const Segment& seg, const Point& p) noexcept;

[[nodiscard]] bool IsIntersect(const Point& seg_start, const Point& p,
                               const Point& seg_end) noexcept;

[[nodiscard]] bool IsIntersect(const Segment& seg_a,
                               const Segment& seg_b) noexcept;

[[nodiscard]] bool IsIntersect(const Line& line_a, const Line& line_b) noexcept;

[[nodiscard]] bool IsIntersect(const Ray& ray, const Line& line) noexcept;

[[nodiscard]] bool IsIntersect(const Line& line, const Ray& ray) noexcept;

[[nodiscard]] bool IsIntersect(const Ray& ray_a, const Ray& ray_b) noexcept;

[[nodiscard]] bool IsIntersect(const ConvexHull& ch, const Point& p) noexcept;

[[nodiscard]] bool IsIntersect(const Point& p, const ConvexHull& ch) noexcept;

[[nodiscard]] bool IsIntersectBorder(const ConvexHull& ch,
                                     const Point& p) noexcept;

[[nodiscard]] bool IsIntersectBorder(const Point& p,
                                     const ConvexHull& ch) noexcept;

[[nodiscard]] bool IsIntersect(const HalfPlane& hp, const Point& p) noexcept;

[[nodiscard]] bool IsIntersect(const Point& p, const HalfPlane& hp) noexcept;

[[nodiscard]] std::optional<Point> Intersect(const Ray& ray,
                                             const Line& line) noexcept;

[[nodiscard]] std::optional<Point> Intersect(const Line& line,
                                             const Ray& ray) noexcept;

[[nodiscard]] std::optional<Point> Intersect(const HalfPlane& hp,
                                             const Point& curr,
                                             const Point& next) noexcept;

[[nodiscard]] ConvexHull Intersect(const ConvexHull& ch, const HalfPlane& hp);

[[nodiscard]] ConvexHull Intersect(const HalfPlane& hp, const ConvexHull& ch);

[[nodiscard]] std::optional<ConvexHull> Intersect(const PointsT& borders,
                                                  const HalfPlanesT& hp);

[[nodiscard]] std::optional<ConvexHull> Intersect(const HalfPlanesT& hp,
                                                  const PointsT& borders);

[[nodiscard]] ScalarT CalcDistance(const Point& a, const Point& b) noexcept;

[[nodiscard]] ScalarT CalcDistance(const Point& p, const Segment& seg) noexcept;

[[nodiscard]] ScalarT CalcDistance(const Segment& seg, const Point& p) noexcept;

[[nodiscard]] ScalarT CalcDistance(const Segment& seg_a,
                                   const Segment& seg_b) noexcept;

[[nodiscard]] ScalarT CalcDistance(const ConvexHull& ch,
                                   const Point& p) noexcept;

[[nodiscard]] ScalarT CalcDistance(const Point& p,
                                   const ConvexHull& ch) noexcept;

[[nodiscard]] ConvexHull CalcMinkowskiDifference(const ConvexHull& pol_a,
                                                 const ConvexHull& pol_b);

};  // namespace geometry