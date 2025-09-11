#include <ranges>
#include <utility>

#include "Geometry/Geometry.hpp"

namespace geometry {
VoronoiDiagram::VoronoiDiagram(const PointsT& borders, PointsT points)
    : points_(std::move(points)) {
  for (const auto& point : points_) {
    auto half_planes = GenerateHalfPlanes(point);
    if (auto cell_res = Intersect(borders, half_planes); cell_res.has_value()) {
      cells_.emplace_back(std::move(cell_res.value()));
    }
  }
}

[[nodiscard]] const VoronoiDiagram::CellsT& VoronoiDiagram::GetCells()
    const noexcept {
  return cells_;
}

[[nodiscard]] HalfPlanesT VoronoiDiagram::GenerateHalfPlanes(
    const Point& point) {
  HalfPlanesT half_planes;
  for (const Point& other_point :
       points_ | std::views::filter(
                     [&](const Point& element) { return element != point; })) {
    half_planes.emplace_back(
        HalfPlane::BuildAsPerpendicularToSegment({point, other_point}, point));
  }
  return half_planes;
}
};  // namespace geometry