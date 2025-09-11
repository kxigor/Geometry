#include <cstddef>
#include <iomanip>
#include <iostream>
#include <utility>

#include "Geometry/Geometry3d.hpp"

int main() {
  std::size_t number_of_points{};
  std::cin >> number_of_points;
  geometry3d::PointsT points(number_of_points);
  for (auto& point : points) {
    std::cin >> point;
  }
  geometry3d::ConvexHull convex_hull(std::move(points));
  std::size_t number_or_requests{};
  std::cin >> number_or_requests;
  std::cout << std::fixed << std::setprecision(10);
  for (std::size_t i = 0; i < number_or_requests; ++i) {
    geometry3d::Point p;
    std::cin >> p;
    std::cout << convex_hull.CalcDistance(p) << '\n';
  }
}