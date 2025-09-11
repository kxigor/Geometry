#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <utility>

#include "Geometry/Geometry.hpp"

namespace geometry {
[[nodiscard]] bool GrahamsScan::PolarAnglePredicat::operator()(
    const Point& p1, const Point& p2) const noexcept {
  auto prod = CalcOrientation(pivot_, p1, p2);
  if (prod == Orientation::Collinear) {
    return (p1 - pivot_).SquaredLength() < (p2 - pivot_).SquaredLength();
  }
  return prod == kOrientation;
}

/*==================== Constructors/Destructors =====================*/
GrahamsScan::GrahamsScan(PointsT points) { Scan(std::move(points)); }

/*============================= Getters =============================*/
[[nodiscard]] const PointsT& GrahamsScan::GetConvexHull() const noexcept {
  return convex_hull_;
}
[[nodiscard]] PointsT&& GrahamsScan::TakeConvexHull() noexcept {
  return std::move(convex_hull_);
}

/*============================= Helpers =============================*/
void GrahamsScan::SortCCW(PointsT& points, const Point& pivot) {
  auto polar_pred = PolarAnglePredicat(pivot);
  std::ranges::sort(points, polar_pred);
}

[[nodiscard]] Point GrahamsScan::GetPivot(const PointsT& points) noexcept {
  static auto pivot_pred = [](const Point& p1, const Point& p2) {
    if (p1.GetY() != p2.GetY()) {
      return p1.GetY() < p2.GetY();
    }
    return p1.GetX() < p2.GetX();
  };
  return *std::ranges::min_element(points, pivot_pred);
}

/*============================= Implds ==============================*/
void GrahamsScan::Scan(PointsT points) {
  if (points.empty()) {
    return;
  }

  SortCCW(points, GetPivot(points));
  DeleteDublicates(points);
  if (points.size() <= 2) {
    convex_hull_ = points;
    return;
  }

  convex_hull_.assign({points[0], points[1]});

  for (std::size_t i = 2; i < points.size(); ++i) {
    while (convex_hull_.size() >= 2) {
      auto top = *std::prev(convex_hull_.end(), 1);
      auto toptop = *std::prev(convex_hull_.end(), 2);
      if (CalcOrientation(toptop, top, points[i]) != kOrientation) {
        convex_hull_.pop_back();
      } else {
        break;
      }
    }
    convex_hull_.push_back(points[i]);
  }
}

void GrahamsScan::DeleteDublicates(PointsT& points) {
  points.erase(std::ranges::unique(points).begin(), points.end());
}

ConvexHull::ConvexHull(PointsT points) {
  *this = InitViaArbitraryPoints(std::move(points));
}

ConvexHull::SizeT ConvexHull::GetSize() const noexcept {
  return points_.size();
}

bool ConvexHull::IsEmpty() const noexcept { return points_.empty(); }

ScalarT ConvexHull::GetArea() const noexcept {
  ScalarT area{};
  const std::size_t n = points_.size();
  for (std::size_t i = 0; i < n; ++i) {
    std::size_t j = (i + 1) % n;
    area += points_[i].VectorProduct(points_[j]);
  }
  using std::abs;
  return abs(area) / ScalarT(2);
}

Point& ConvexHull::operator[](SizeT pos) noexcept { return points_[pos]; }

const Point& ConvexHull::operator[](SizeT pos) const noexcept {
  return points_[pos];
}

bool ConvexHull::IsDegenerate() const noexcept { return points_.size() < 3; }

ConvexHull ConvexHull::ConvexHull::InitViaArbitraryPoints(PointsT points) {
  if (points.empty()) {
    return {};
  }
  ConvexHull result;
  GrahamsScan scan(std::move(points));
  result.points_ = scan.TakeConvexHull();
  return result;
}

ConvexHull ConvexHull::InitViaUnorderedConvexHull(PointsT points) {
  if (points.empty()) {
    return {};
  }
  ConvexHull result;
  result.points_ = std::move(points);
  GrahamsScan::SortCCW(result.points_, GrahamsScan::GetPivot(result.points_));
  return result;
}

ConvexHull ConvexHull::InitViaUnorderedConvexHullWithDuplicates(
    PointsT points) {
  if (points.empty()) {
    return {};
  }
  ConvexHull result;
  result.points_ = std::move(points);
  GrahamsScan::SortCCW(result.points_, GrahamsScan::GetPivot(result.points_));
  result.points_.erase(std::ranges::unique(result.points_).begin(),
                       result.points_.end());
  return result;
}

ConvexHull ConvexHull::InitDirect(PointsT points) {
  if (points.empty()) {
    return {};
  }
  ConvexHull result;
  result.points_ = std::move(points);
  return result;
}

Point ConvexHull::GetCentroid() const noexcept {
  Point center{ScalarT(0), ScalarT(0)};
  for (const auto& point : points_) {
    center += point;
  }
  center /= ScalarT(points_.size());
  return center;
}

const PointsT& ConvexHull::GetPoints() const noexcept { return points_; }

PointsT&& ConvexHull::TakePoints() noexcept { return std::move(points_); }

ConvexHull::PointIteratorT ConvexHull::PointBegin() const noexcept {
  return points_.cbegin();
}

ConvexHull::PointIteratorT ConvexHull::PointEnd() const noexcept {
  return points_.cend();
}

ConvexHull::EdgeIterator ConvexHull::EdgeBegin() const noexcept {
  return EdgeIterator(*this);
}

ConvexHull::EdgeIterator ConvexHull::EdgeEnd() const noexcept {
  return EdgeIterator(*this, this->GetSize());
}

ConvexHull::PointIteratorT ConvexHull::begin() const noexcept {
  return PointBegin();
}

ConvexHull::PointIteratorT ConvexHull::end() const noexcept {
  return PointEnd();
}

}  // namespace geometry