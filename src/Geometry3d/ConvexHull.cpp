#include <algorithm>
#include <cmath>
#include <limits>
#include <ranges>
#include <utility>

#include "Geometry/Geometry3d.hpp"
namespace geometry3d {
bool ConvexHull::ZPred::operator()(const Point& lhs,
                                   const Point& rhs) const noexcept {
  return lhs.GetZ() < rhs.GetZ();
}

bool ConvexHull::NormalAnglePred::operator()(const Point& lhs,
                                             const Point& rhs) const {
  const auto kLhsNormal = CalcPlaneNormal(lhs, p2_, p1_);
  const auto kRhsNormal = CalcPlaneNormal(rhs, p2_, p1_);
  const auto kLhsAngle = AngleBetweenVectors(kLhsNormal, normal_);
  const auto kRhsAngle = AngleBetweenVectors(kRhsNormal, normal_);
  return kLhsAngle < kRhsAngle;
}

ConvexHull::ConvexHull(PointsT points) : points_(std::move(points)) {
  DeleteNonUnique();
  Initializate();
  Scan();
}

[[nodiscard]] ScalarT ConvexHull::CalcDistance(
    const Point& point) const noexcept {
  using std::abs, std::min;
  ScalarT result = std::numeric_limits<ScalarT::DoubleT>::max();
  for (const Face& face : faces_) {
    const Vector n = face.GetNormal();
    const Point P = face.GetP();
    result = min(result, ScalarT(abs(n.ScalarProduct(point - P))));
  }
  return result;
}

const FacesT& ConvexHull::GetFaces() const noexcept { return faces_; }

void ConvexHull::DeleteNonUnique() {
  std::ranges::sort(points_);
  points_.erase(std::ranges::unique(points_).begin(),
                std::ranges::end(points_));
}

void ConvexHull::Initializate() {
  auto P = GetFirstPoint();
  auto Q = GetSecondPoint(P);
  auto R = GetThirdPoint(P, Q);
  faces_.emplace_back(P, Q, R);
  AddEdge({P, Q}, faces_[0]);
  AddEdge({Q, R}, faces_[0]);
  AddEdge({R, P}, faces_[0]);
}

[[nodiscard]] Point ConvexHull::GetFirstPoint() const noexcept {
  return GetMinNoTrashPoints(ZPred{});
}

[[nodiscard]] Point ConvexHull::GetSecondPoint(const Point& P) const noexcept {
  return GetMinNoTrashPoints(ZPred{}, P);
}

[[nodiscard]] Point ConvexHull::GetThirdPoint(const Point& P,
                                              const Point& Q) const {
  auto norm_pred = NormalAnglePred(P, Q, Oxy.GetNormal());
  return GetMinNoTrashPoints(norm_pred, P, Q);
}

void ConvexHull::Scan() {
  while (not open_edges_.empty()) {
    const auto kEdge = open_edges_.back();
    open_edges_.pop_back();
    if (closed_edges_.contains(kEdge)) {
      continue;
    }
    const auto& kFace = edges_face_.at(kEdge);
    auto pred = NormalAnglePred(kEdge.GetA(), kEdge.GetB(), kFace.GetNormal());
    auto new_point =
        GetMinNoTrashPoints(pred, kFace.GetP(), kFace.GetQ(), kFace.GetR());
    auto& new_face = AddFace(kEdge, new_point);
    AddEdge({kEdge.GetB(), new_point}, new_face);
    AddEdge({new_point, kEdge.GetA()}, new_face);
  }
}

void ConvexHull::AddEdge(const EdgeT& edge, const Face& face) {
  if (edges_face_.contains(edge)) {
    closed_edges_.insert(edge);
  } else {
    open_edges_.push_back(edge);
    edges_face_.emplace(edge, face);
  }
}

Face& ConvexHull::AddFace(const EdgeT& edge, const Point& point) {
  return faces_.emplace_back(edge.GetA(), edge.GetB(), point);
}

}  // namespace geometry3d