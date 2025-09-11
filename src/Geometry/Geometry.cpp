#include "Geometry/Geometry.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <limits>
#include <optional>
#include <ranges>
#include <unordered_set>
#include <utility>

namespace geometry {
[[nodiscard]] ScalarT CrossProduct(const Point& a, const Point& b,
                                   const Point& c) noexcept {
  return (b - a).VectorProduct(c - a);
}

[[nodiscard]] Ray RotateRayRadian(const Ray& ray, ScalarT angle_radians,
                                  bool clockwise) noexcept {
  Point origin = ray.GetOrigin();
  Point dir = ray.GetDirection();

  if (!clockwise) {
    angle_radians = -angle_radians;
  }

  // [ cosθ  sinθ ]
  // [-sinθ  cosθ ]
  using std::cos, std::sin;
  auto cos_theta = ScalarT(cos(angle_radians));
  auto sin_theta = ScalarT(sin(angle_radians));

  auto new_dir_x = dir.GetX() * cos_theta + dir.GetY() * sin_theta;
  auto new_dir_y = -dir.GetX() * sin_theta + dir.GetY() * cos_theta;

  Point new_direction(new_dir_x, new_dir_y);
  return Ray{origin, new_direction};
}

[[nodiscard]] Ray RotateRayDegree(const Ray& ray, ScalarT angle_degrees,
                                  bool clockwise) noexcept {
  return RotateRayRadian(ray, angle_degrees * ScalarT(M_PI) / ScalarT(180.0),
                         clockwise);
}

[[nodiscard]] Orientation CalcOrientation(const Point& a, const Point& b,
                                          const Point& c) noexcept {
  const ScalarT kCross = CrossProduct(a, b, c);
  if (kCross == ScalarT(0)) {
    return Orientation::Collinear;
  }
  if (kCross > ScalarT(0)) {
    return Orientation::CounterClockwise;
  }
  return Orientation::Clockwise;
}

[[nodiscard]] bool IsIntersect(const Point& a, const Point& b) noexcept {
  return a == b;
}

[[nodiscard]] bool IsIntersect(const Point& p, const Segment& seg) noexcept {
  if (CalcOrientation(seg.GetPointA(), p, seg.GetPointB()) !=
      Orientation::Collinear) {
    return false;
  }

  auto [x, y] = p.GetAsTuple();
  auto [x1, y1] = seg.GetPointA().GetAsTuple();
  auto [x2, y2] = seg.GetPointB().GetAsTuple();

  using std::max;
  using std::min;
  return x <= max(x1, x2) && x >= min(x1, x2) && y <= max(y1, y2) &&
         y >= min(y1, y2);
}

[[nodiscard]] bool IsIntersect(const Segment& seg, const Point& p) noexcept {
  return IsIntersect(p, seg);
}

[[nodiscard]] bool IsIntersect(const Point& seg_start, const Point& p,
                               const Point& seg_end) noexcept {
  return IsIntersect(Segment(seg_start, seg_end), p);
}

[[nodiscard]] bool IsIntersect(const Segment& seg_a,
                               const Segment& seg_b) noexcept {
  auto [A, B] = seg_a.GetAsTuple();
  auto [C, D] = seg_b.GetAsTuple();
  auto o1 = CalcOrientation(A, B, C);
  auto o2 = CalcOrientation(A, B, D);
  auto o3 = CalcOrientation(C, D, A);
  auto o4 = CalcOrientation(C, D, B);
  return (o1 != o2 && o3 != o4) ||
         (o1 == Orientation::Collinear && IsIntersect(A, C, B)) ||
         (o2 == Orientation::Collinear && IsIntersect(A, D, B)) ||
         (o3 == Orientation::Collinear && IsIntersect(C, A, D)) ||
         (o4 == Orientation::Collinear && IsIntersect(C, B, D));
}

[[nodiscard]] bool IsIntersect(const Line& line_a,
                               const Line& line_b) noexcept {
  auto [a1, b1, c1] = line_a.GetAsTuple();
  auto [a2, b2, c2] = line_b.GetAsTuple();
  ScalarT det = a1 * b2 - a2 * b1;
  if (det == ScalarT(0)) {
    if (b1 != ScalarT(0)) {
      return c2 * b1 == c1 * b2;
    }
    return c2 * a1 == c1 * a2;
  }
  return true;
}

[[nodiscard]] bool IsIntersect(const Ray& ray, const Line& line) noexcept {
  auto [a, b, c] = line.GetAsTuple();
  Point line_normal(a, b);

  ScalarT denom = ray.GetDirection().ScalarProduct(line_normal);

  if (denom == ScalarT(0)) {
    return (a * ray.GetOrigin().GetX() + b * ray.GetOrigin().GetY() + c) ==
           ScalarT(0);
  }

  ScalarT t = -(line_normal.ScalarProduct(ray.GetOrigin()) + c) / denom;
  return t >= ScalarT(0);
}

[[nodiscard]] bool IsIntersect(const Line& line, const Ray& ray) noexcept {
  return IsIntersect(ray, line);
}

[[nodiscard]] bool IsIntersect(const Ray& ray_a, const Ray& ray_b) noexcept {
  Point dir1 = ray_a.GetDirection();
  Point dir2 = ray_b.GetDirection();

  ScalarT det = dir2.VectorProduct(dir1);

  if (det == ScalarT(0)) {
    Point diff = ray_b.GetOrigin() - ray_a.GetOrigin();
    if (diff.VectorProduct(dir1) != ScalarT(0)) {
      return false;
    }
    return dir1.ScalarProduct(dir2) > ScalarT(0);
  }

  Point diff = ray_b.GetOrigin() - ray_a.GetOrigin();
  ScalarT t1 = diff.VectorProduct(dir2) / det;
  ScalarT t2 = diff.VectorProduct(dir1) / det;

  return t1 >= ScalarT(0) && t2 >= ScalarT(0);
}

[[nodiscard]] bool IsIntersect(const ConvexHull& ch, const Point& p) noexcept {
  if (ch.GetSize() == 1) {
    return IsIntersect(ch[0], p);
  }
  for (auto edgeIt = ch.EdgeBegin(); edgeIt != ch.EdgeEnd(); ++edgeIt) {
    if (IsIntersect(*edgeIt, p)) {
      return true;
    }
    if (CalcOrientation(edgeIt->GetPointA(), edgeIt->GetPointB(), p) !=
        ConvexHull::kConvexHullOrientation) {
      return false;
    }
  }
  return true;
}

[[nodiscard]] bool IsIntersect(const Point& p, const ConvexHull& ch) noexcept {
  return IsIntersect(ch, p);
}

[[nodiscard]] bool IsIntersectBorder(const ConvexHull& ch,
                                     const Point& p) noexcept {
  if (ch.GetSize() == 1) {
    return IsIntersect(ch[0], p);
  }
  for (auto edge_it = ch.EdgeBegin(); edge_it != ch.EdgeEnd(); ++edge_it) {
    if (IsIntersect(*edge_it, p)) {
      return true;
    }
  }
  return false;
}

[[nodiscard]] bool IsIntersectBorder(const Point& p,
                                     const ConvexHull& ch) noexcept {
  return IsIntersectBorder(ch, p);
}

[[nodiscard]] bool IsIntersect(const HalfPlane& hp, const Point& p) noexcept {
  ScalarT value = hp.GetA() * p.GetX() + hp.GetB() * p.GetY() + hp.GetC();
  return value >= ScalarT(0);
}

[[nodiscard]] bool IsIntersect(const Point& p, const HalfPlane& hp) noexcept {
  return IsIntersect(hp, p);
}

std::optional<Point> Intersect(const Ray& ray, const Line& line) noexcept {
  auto [a, b, c] = line.GetAsTuple();
  Vector line_normal(a, b);

  ScalarT denom = ray.GetDirection().ScalarProduct(line_normal);

  if (denom == ScalarT(0)) {
    return std::nullopt;
  }

  ScalarT t = -(line_normal.ScalarProduct(ray.GetOrigin()) + c) / denom;
  if (t < ScalarT(0)) {
    return std::nullopt;
  }

  return ray.GetOrigin() + ray.GetDirection() * t;
}

std::optional<Point> Intersect(const Line& line, const Ray& ray) noexcept {
  return Intersect(ray, line);
}

[[nodiscard]] std::optional<Point> Intersect(const HalfPlane& hp,
                                             const Point& curr,
                                             const Point& next) noexcept {
  ScalarT a = hp.GetA(), b = hp.GetB(), c = hp.GetC();
  ScalarT denom = Point(a, b).ScalarProduct(next - curr);
  if (denom == ScalarT(0)) {
    return std::nullopt;
  }
  ScalarT t = -(a * curr.GetX() + b * curr.GetY() + c) / denom;
  if (t < ScalarT(0) || t > ScalarT(1)) {
    return std::nullopt;
  }
  return curr + (next - curr) * t;
}

[[nodiscard]] static PointsT IntersectImpl(const PointsT& ch,
                                           const HalfPlane& hp) {
  if (ch.empty()) {
    return {};
  }

  PointsT new_convex_hull;
  Point prev = ch.back();
  bool prev_inside = IsIntersect(hp, prev);

  for (const Point& curr : ch) {
    bool curr_inside = IsIntersect(hp, curr);
    if (curr_inside != prev_inside) {
      auto intersect = Intersect(hp, prev, curr);
      if (intersect) {
        new_convex_hull.push_back(*intersect);
      }
    }
    if (curr_inside) {
      new_convex_hull.push_back(curr);
    }
    prev = curr;
    prev_inside = curr_inside;
  }

  return new_convex_hull;
}

[[nodiscard]] ConvexHull Intersect(const ConvexHull& ch, const HalfPlane& hp) {
  return ConvexHull::InitViaUnorderedConvexHullWithDuplicates(
      IntersectImpl(ch.GetPoints(), hp));
}

[[nodiscard]] ConvexHull Intersect(const HalfPlane& hp, const ConvexHull& ch) {
  return Intersect(ch, hp);
}

[[nodiscard]] std::optional<ConvexHull> Intersect(const PointsT& borders,
                                                  const HalfPlanesT& hp) {
  PointsT convex_hull =
      ConvexHull::InitViaArbitraryPoints(borders).TakePoints();
  for (const auto& hp : hp) {
    convex_hull = std::move(IntersectImpl(convex_hull, hp));
    if (convex_hull.empty()) {
      return {};
    }
  }
  return ConvexHull::InitViaUnorderedConvexHullWithDuplicates(
      std::move(convex_hull));
}

[[nodiscard]] std::optional<ConvexHull> Intersect(const HalfPlanesT& hp,
                                                  const PointsT& borders) {
  return Intersect(borders, hp);
}

[[nodiscard]] ScalarT CalcDistance(const Point& a, const Point& b) noexcept {
  auto dx = a.GetX() - b.GetX();
  auto dy = a.GetY() - b.GetY();
  using std::sqrt;
  return sqrt((dx * dx) + (dy * dy));
}

[[nodiscard]] ScalarT CalcDistance(const Point& p,
                                   const Segment& seg) noexcept {
  if (seg.IsDegenerate()) {
    return CalcDistance(p, seg.GetPointA());
    return CalcDistance(p, seg.GetPointA());
  }

  auto [A, B] = seg.GetAsTuple();

  Point AB = B - A;
  Point AP = p - A;

  ScalarT t = AP.ScalarProduct(AB) / AB.SquaredLength();
  using std::clamp;
  t = clamp(t, ScalarT(0), ScalarT(1));
  Point Q = A + AB * t;
  return CalcDistance(p, Q);
}

[[nodiscard]] ScalarT CalcDistance(const Segment& seg,
                                   const Point& p) noexcept {
  return CalcDistance(p, seg);
}

[[nodiscard]] ScalarT CalcDistance(const Segment& seg_a,
                                   const Segment& seg_b) noexcept {
  if (IsIntersect(seg_a, seg_b)) {
    return ScalarT(0);
  }

  ScalarT d1 = CalcDistance(seg_a.GetPointA(), seg_b);
  ScalarT d2 = CalcDistance(seg_a.GetPointB(), seg_b);
  ScalarT d3 = CalcDistance(seg_b.GetPointA(), seg_a);
  ScalarT d4 = CalcDistance(seg_b.GetPointB(), seg_a);

  using std::min;
  return min({d1, d2, d3, d4});
}

[[nodiscard]] ScalarT CalcDistance(const ConvexHull& ch,
                                   const Point& p) noexcept {
  if (ch.GetSize() == 1) return CalcDistance(ch[0], p);
  ScalarT distance = std::numeric_limits<ScalarT>::max();
  for (auto edgeIt = ch.EdgeBegin(); edgeIt != ch.EdgeEnd(); ++edgeIt) {
    distance = std::min(distance, CalcDistance(*edgeIt, p));
  }
  return distance;
}

[[nodiscard]] ScalarT CalcDistance(const Point& point,
                                   const ConvexHull& ch) noexcept {
  return CalcDistance(ch, point);
}

[[nodiscard]] ConvexHull CalcMinkowskiDifference(const ConvexHull& pol_a,
                                                 const ConvexHull& pol_b) {
  using std::ranges::begin, std::ranges::min_element, std::ranges::max_element,
      std::distance;
  constexpr bool kOrient =
      ConvexHull::kConvexHullOrientation == Orientation::Clockwise;
  constexpr ScalarT kSign = kOrient ? 1 : -1;

  if (pol_a.IsEmpty() || pol_b.IsEmpty()) return {};

  std::unordered_set<Point, Point::Hash> points;
  auto i = static_cast<std::size_t>(distance(
      begin(pol_a), kOrient ? min_element(pol_a) : max_element(pol_a)));
  auto j = static_cast<std::size_t>(distance(
      begin(pol_b), kOrient ? max_element(pol_b) : min_element(pol_b)));
  auto itA = ConvexHull::EdgeIterator(pol_a, i);
  auto itB = ConvexHull::EdgeIterator(pol_b, j);
  do {
    if (itA == pol_a.EdgeEnd()) itA = pol_a.EdgeBegin();
    if (itB == pol_b.EdgeEnd()) itB = pol_b.EdgeBegin();
    auto new_point = itA->GetPointA() - itB->GetPointA();
    if (points.contains(new_point)) {
      break;
    }
    points.emplace(new_point);
    const ScalarT cross = itA->GetVector().VectorProduct(itB->GetVector());
    if (cross * kSign >= ScalarT(0)) ++itA;
    if (cross * kSign <= ScalarT(0)) ++itB;
  } while (itA != pol_a.EdgeEnd() || itB != pol_b.EdgeEnd());

  return ConvexHull::InitViaUnorderedConvexHull({points.begin(), points.end()});
}
}  // namespace geometry