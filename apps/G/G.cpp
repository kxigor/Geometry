#include <algorithm>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <utility>

#include "Geometry/Geometry.hpp"

namespace {
geometry::PointsT GenerateBorders(const geometry::Point& point) {
  const auto& [x, y] = point.GetAsTuple();
  return {geometry::Point{0, 0}, geometry::Point{x, 0}, geometry::Point{x, y},
          geometry::Point{0, y}};
}

geometry::PointsT SortConvexHullByMin(geometry::ConvexHull pol) {
  auto points = pol.TakePoints();
  auto min = *std::ranges::min_element(points);
  geometry::GrahamsScan::SortCCW(points, min);
  return points;
}

}  // namespace

int main() {
  geometry::Point border;
  std::size_t number_of_points{};
  std::cin >> border;
  std::cin >> number_of_points;

  geometry::PointsT points(number_of_points);
  for (auto& point : points) {
    std::cin >> point;
  }

  auto borders = GenerateBorders(border);

  geometry::VoronoiDiagram voronoi(borders, std::move(points));
  std::cout << std::fixed << std::setprecision(8);  // NOLINT
  for (auto&& cell : voronoi.TakeCells()) {
    auto sorted_cell = SortConvexHullByMin(cell);
    std::cout << sorted_cell.size() << ' ';
    for (const auto& point : sorted_cell) {
      std::cout << point << ' ';
    }
    std::cout << '\n';
  }
}