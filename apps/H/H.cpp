#include <algorithm>
#include <cstddef>
#include <iostream>
#include <iterator>
#include <list>
#include <utility>

#include "Geometry/Geometry.hpp"

namespace {
void SwapAndDelete(auto& cont, auto it) {
  std::iter_swap(std::prev(cont.end()), it);
  cont.pop_back();
}
static std::list<geometry::ConvexHull> ConstructNestedConvexHulls(
    geometry::PointsT points);
std::list<geometry::ConvexHull> ConstructNestedConvexHulls(
    geometry::PointsT points) {
  std::list<geometry::ConvexHull> convex_hulls;
  do {  // NOLINT
    convex_hulls.emplace_back(points);
    for (auto point_it = points.begin(); point_it != points.end();) {
      if (IsIntersectBorder(convex_hulls.back(), *point_it)) {
        SwapAndDelete(points, point_it);
      } else {
        ++point_it;
      }
    }
  } while (points.size() >= 3);  // NOLINT

  return convex_hulls;
}
std::size_t CalculateNestingLevel(
    const std::list<geometry::ConvexHull>& convex_hulls,
    const geometry::Point& station) {
  std::size_t nestring_level{};
  for (const auto& convex_hull : convex_hulls) {
    if (IsIntersect(convex_hull, station)) {
      ++nestring_level;
    } else {
      break;
    }
  }
  return ((nestring_level > 0) ? nestring_level - 1 : 0);
}
geometry::PointsT ReadPoints() {
  std::size_t size{};
  std::cin >> size;
  geometry::PointsT points(size);
  std::copy_n(std::istream_iterator<geometry::Point>(std::cin), size,
              points.begin());
  return points;
}
}  // namespace

int main() {
  auto convex_hulls_points = ReadPoints();
  auto stations_points = ReadPoints();
  auto convex_hulls =
      ConstructNestedConvexHulls(std::move(convex_hulls_points));
  for (const auto& station : stations_points) {
    std::cout << CalculateNestingLevel(convex_hulls, station) << '\n';
  }
}
