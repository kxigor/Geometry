#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <utility>

#include "Geometry/Geometry.hpp"
namespace {
geometry::ConvexHull GetConvexHull(std::size_t number_of_points) {
  geometry::PointsT convex_hull(number_of_points);
  std::copy_n(std::istream_iterator<geometry::Point>{std::cin},
              number_of_points, convex_hull.begin());
  return geometry::ConvexHull::InitViaUnorderedConvexHull(
      std::move(convex_hull));
}
}  // namespace

int main() {
  std::size_t number_of_airoport_points{};
  std::size_t number_of_cloud_points{};
  std::cin >> number_of_airoport_points;
  std::cin >> number_of_cloud_points;

  auto airoport_convex_hull = GetConvexHull(number_of_airoport_points);
  auto cloud_convex_hull = GetConvexHull(number_of_cloud_points);
  auto difference =
      CalcMinkowskiDifference(airoport_convex_hull, cloud_convex_hull);

  std::cout << std::fixed << std::setprecision(20);  // NOLINT
  if (const geometry::Point kNullPoint{0, 0};
      geometry::IsIntersect(difference, kNullPoint)) {
    std::cout << std::max(geometry::CalcDistance(difference, kNullPoint) -
                              geometry::ScalarT(60.0),  // NOLINT
                          geometry::ScalarT(0));
  } else {
    std::cout << 0;
  }
}
