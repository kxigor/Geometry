#include <gtest/gtest.h>

#include "Geometry/Geometry.hpp"

using namespace testing;
using namespace geometry;

TEST(VectorTest, Constructors) {
  Vector v1;
  EXPECT_EQ(v1.GetX(), ScalarT(0));
  EXPECT_EQ(v1.GetY(), ScalarT(0));

  Vector v2(1.5, 2.5);
  EXPECT_EQ(v2.GetX(), ScalarT(1.5));
  EXPECT_EQ(v2.GetY(), ScalarT(2.5));

  Vector v3(std::make_pair(ScalarT(3.0), ScalarT(4.0)));
  EXPECT_EQ(v3.GetX(), ScalarT(3.0));
  EXPECT_EQ(v3.GetY(), ScalarT(4.0));

  Vector v4(std::make_tuple(ScalarT(5.0), ScalarT(6.0)));
  EXPECT_EQ(v4.GetX(), ScalarT(5.0));
  EXPECT_EQ(v4.GetY(), ScalarT(6.0));

  Vector v5{7.0, 8.0};
  EXPECT_EQ(v5.GetX(), ScalarT(7.0));
  EXPECT_EQ(v5.GetY(), ScalarT(8.0));

  EXPECT_THROW(Vector({1.0}), std::invalid_argument);
  EXPECT_THROW(Vector({1.0, 2.0, 3.0}), std::invalid_argument);
}

TEST(VectorTest, Comparisons) {
  Vector v1(1, 2);
  Vector v2(1, 2);
  Vector v3(2, 1);
  Vector v4(1, 1);

  EXPECT_EQ(v1, v2);
  EXPECT_NE(v1, v3);
  EXPECT_LT(v4, v1);
  EXPECT_GT(v3, v1);
}

TEST(VectorTest, ArithmeticOperations) {
  Vector v1(1, 2);
  Vector v2(3, 4);

  Vector neg = -v1;
  EXPECT_EQ(neg.GetX(), ScalarT(-1));
  EXPECT_EQ(neg.GetY(), ScalarT(-2));

  Vector sum = v1 + v2;
  EXPECT_EQ(sum.GetX(), ScalarT(4));
  EXPECT_EQ(sum.GetY(), ScalarT(6));

  Vector diff = v2 - v1;
  EXPECT_EQ(diff.GetX(), ScalarT(2));
  EXPECT_EQ(diff.GetY(), ScalarT(2));

  Vector scaled = v1 * ScalarT(3.0);
  EXPECT_EQ(scaled.GetX(), ScalarT(3));
  EXPECT_EQ(scaled.GetY(), ScalarT(6));

  Vector divided = v2 / ScalarT(2.0);
  EXPECT_EQ(divided.GetX(), ScalarT(1.5));
  EXPECT_EQ(divided.GetY(), ScalarT(2.0));
}

TEST(VectorTest, CompoundAssignments) {
  Vector v1(1, 2);
  Vector v2(3, 4);

  v1 += v2;
  EXPECT_EQ(v1.GetX(), ScalarT(4));
  EXPECT_EQ(v1.GetY(), ScalarT(6));

  v1 -= v2;
  EXPECT_EQ(v1.GetX(), ScalarT(1));
  EXPECT_EQ(v1.GetY(), ScalarT(2));

  v1 *= 5;
  EXPECT_EQ(v1.GetX(), ScalarT(5));
  EXPECT_EQ(v1.GetY(), ScalarT(10));

  v1 /= 5;
  EXPECT_EQ(v1.GetX(), ScalarT(1));
  EXPECT_EQ(v1.GetY(), ScalarT(2));
}

TEST(VectorTest, GeometryOperations) {
  Vector v1(3, 4);
  Vector v2(1, 0);
  Vector v3(0, 1);

  EXPECT_EQ(v1.Length(), ScalarT(5));
  EXPECT_EQ(v1.SquaredLength(), ScalarT(25));

  EXPECT_EQ(v2.ScalarProduct(v3), ScalarT(0));
  EXPECT_EQ(v2.ScalarProduct(v2), ScalarT(1));

  EXPECT_EQ(v2.VectorProduct(v3), ScalarT(1));
  EXPECT_EQ(v3.VectorProduct(v2), ScalarT(-1));

  Vector normalized = v1.GetNormalized();
  EXPECT_EQ(normalized.Length(), ScalarT(1.0));
  EXPECT_EQ(normalized.GetX(), ScalarT(0.6));
  EXPECT_EQ(normalized.GetY(), ScalarT(0.8));
}

TEST(VectorTest, IOOperations) {
  Vector v;
  std::istringstream input("5.5 6.6");
  input >> v;
  EXPECT_EQ(v.GetX(), ScalarT(5.5));
  EXPECT_EQ(v.GetY(), ScalarT(6.6));

  std::ostringstream output;
  output << v;
  EXPECT_EQ(output.str(), "5.5 6.6");
}

TEST(VectorTest, Hash) {
  Vector v1(1, 2);
  Vector v2(1, 2);
  Vector v3(2, 1);

  Vector::Hash hasher;
  EXPECT_EQ(hasher(v1), hasher(v2));
  EXPECT_NE(hasher(v1), hasher(v3));
}

TEST(SegmentTest, Constructors) {
  Point p1(ScalarT(1), ScalarT(2));
  Point p2(ScalarT(3), ScalarT(4));

  Segment s1;
  EXPECT_EQ(s1.GetPointA(), Point());
  EXPECT_EQ(s1.GetPointB(), Point());

  Segment s2(p1, p2);
  EXPECT_EQ(s2.GetPointA(), p1);
  EXPECT_EQ(s2.GetPointB(), p2);

  Segment s3(std::make_pair(p1, p2));
  EXPECT_EQ(s3.GetPointA(), p1);
  EXPECT_EQ(s3.GetPointB(), p2);

  Segment s4(std::make_tuple(p1, p2));
  EXPECT_EQ(s4.GetPointA(), p1);
  EXPECT_EQ(s4.GetPointB(), p2);
}

TEST(SegmentTest, IOOperations) {
  Segment s(Point(ScalarT(1), ScalarT(2)), Point(ScalarT(3), ScalarT(4)));
  std::ostringstream output;
  output << s;
  EXPECT_EQ(output.str(), "1 2 3 4");

  std::istringstream input("5 6 7 8");
  Segment s2;
  input >> s2;
  EXPECT_EQ(s2.GetPointA(), Point(ScalarT(5), ScalarT(6)));
  EXPECT_EQ(s2.GetPointB(), Point(ScalarT(7), ScalarT(8)));
}

TEST(SegmentTest, GeometryOperations) {
  Segment s1(Point(ScalarT(1), ScalarT(1)), Point(ScalarT(1), ScalarT(1)));
  Segment s2(Point(ScalarT(1), ScalarT(1)), Point(ScalarT(2), ScalarT(2)));

  EXPECT_TRUE(s1.IsDegenerate());
  EXPECT_FALSE(s2.IsDegenerate());

  EXPECT_EQ(s2.GetVector(), Point(ScalarT(1), ScalarT(1)));
  EXPECT_EQ(s2.GetAsPair(), std::make_pair(Point(ScalarT(1), ScalarT(1)),
                                           Point(ScalarT(2), ScalarT(2))));
  EXPECT_EQ(s2.GetAsTuple(), std::make_tuple(Point(ScalarT(1), ScalarT(1)),
                                             Point(ScalarT(2), ScalarT(2))));
}

TEST(LineTest, Constructors) {
  Line l1(ScalarT(1), ScalarT(2), ScalarT(3));
  EXPECT_EQ(l1.GetPointA(), ScalarT(1));
  EXPECT_EQ(l1.GetPointB(), ScalarT(2));
  EXPECT_EQ(l1.GetPointC(), ScalarT(3));
}

TEST(LineTest, Getters) {
  Line l(ScalarT(1), ScalarT(2), ScalarT(3));
  EXPECT_EQ(l.GetAsTuple(),
            std::make_tuple(ScalarT(1), ScalarT(2), ScalarT(3)));
}

TEST(RayTest, Constructors) {
  Point origin(ScalarT(1), ScalarT(1));
  Point direction(ScalarT(1), ScalarT(0));

  Ray r1(origin, direction);
  EXPECT_EQ(r1.GetOrigin(), origin);
  EXPECT_EQ(r1.GetDirection(), direction);

  Ray r2 = Ray::FromPoints(origin, Point(ScalarT(2), ScalarT(1)));
  EXPECT_EQ(r2.GetOrigin(), origin);
  EXPECT_EQ(r2.GetDirection(), Point(ScalarT(1), ScalarT(0)));
}

TEST(RayTest, Getters) {
  Point origin(ScalarT(0), ScalarT(0));
  Point direction(ScalarT(1), ScalarT(1));
  Ray r(origin, direction);

  EXPECT_EQ(r.GetOrigin(), origin);
  EXPECT_EQ(r.GetDirection(), direction);
}

#include <gtest/gtest.h>

TEST(HalfPlaneTest, Constructors) {
  // Default constructor
  HalfPlane hp1;
  EXPECT_EQ(hp1.GetA(), ScalarT(0));
  EXPECT_EQ(hp1.GetB(), ScalarT(0));
  EXPECT_EQ(hp1.GetC(), ScalarT(0));

  // Constructor with coefficients
  HalfPlane hp2(ScalarT(1), ScalarT(2), ScalarT(3));
  EXPECT_EQ(hp2.GetA(), ScalarT(1));
  EXPECT_EQ(hp2.GetB(), ScalarT(2));
  EXPECT_EQ(hp2.GetC(), ScalarT(3));

  // Constructor with points
  Point p1(ScalarT(0), ScalarT(0));
  Point p2(ScalarT(1), ScalarT(0));
  Point inside(ScalarT(0), ScalarT(1));
  HalfPlane hp3(p1, p2, inside);
  EXPECT_EQ(hp3.GetA(), ScalarT(0));
  EXPECT_EQ(hp3.GetB(), ScalarT(1));
  EXPECT_EQ(hp3.GetC(), ScalarT(0));
}

TEST(HalfPlaneTest, NormalizeSign) {
  Point p1(ScalarT(0), ScalarT(0));
  Point p2(ScalarT(1), ScalarT(0));
  Point inside1(ScalarT(0), ScalarT(1));   // Above the line
  Point inside2(ScalarT(0), ScalarT(-1));  // Below the line

  HalfPlane hp1(p1, p2, inside1);
  EXPECT_GT(
      hp1.GetA() * inside1.GetX() + hp1.GetB() * inside1.GetY() + hp1.GetC(),
      ScalarT(0));

  HalfPlane hp2(p1, p2, inside2);
  EXPECT_GT(
      hp2.GetA() * inside2.GetX() + hp2.GetB() * inside2.GetY() + hp2.GetC(),
      ScalarT(0));
}

TEST(HalfPlaneTest, BuildAsPerpendicularToSegment) {
  Segment seg(Point(ScalarT(0), ScalarT(0)), Point(ScalarT(2), ScalarT(0)));
  Point inside(ScalarT(1), ScalarT(1));

  HalfPlane hp = HalfPlane::BuildAsPerpendicularToSegment(seg, inside);
  EXPECT_EQ(hp.GetA(), ScalarT(2));
  EXPECT_EQ(hp.GetB(), ScalarT(0));
  EXPECT_EQ(hp.GetC(), ScalarT(-2));
  EXPECT_GE(hp.GetA() * inside.GetX() + hp.GetB() * inside.GetY() + hp.GetC(),
            ScalarT(0));
}

TEST(HalfPlaneTest, Getters) {
  HalfPlane hp(ScalarT(1), ScalarT(2), ScalarT(3));
  auto [a, b, c] = hp.GetAsTuple();
  EXPECT_EQ(a, ScalarT(1));
  EXPECT_EQ(b, ScalarT(2));
  EXPECT_EQ(c, ScalarT(3));
}

TEST(GrahamsScanTest, BasicFunctionality) {
  PointsT points = {
      Point(ScalarT(0), ScalarT(0)), Point(ScalarT(1), ScalarT(1)),
      Point(ScalarT(2), ScalarT(0)), Point(ScalarT(1), ScalarT(-1)),
      Point(ScalarT(1), ScalarT(0))};

  GrahamsScan gs(points);
  const PointsT& hull = gs.GetConvexHull();

  // Should contain only convex hull points
  EXPECT_EQ(hull.size(), 4);
  EXPECT_TRUE(std::find(hull.begin(), hull.end(),
                        Point(ScalarT(0), ScalarT(0))) != hull.end());
  EXPECT_TRUE(std::find(hull.begin(), hull.end(),
                        Point(ScalarT(1), ScalarT(1))) != hull.end());
  EXPECT_TRUE(std::find(hull.begin(), hull.end(),
                        Point(ScalarT(2), ScalarT(0))) != hull.end());
  EXPECT_TRUE(std::find(hull.begin(), hull.end(),
                        Point(ScalarT(1), ScalarT(-1))) != hull.end());
  EXPECT_FALSE(std::find(hull.begin(), hull.end(),
                         Point(ScalarT(1), ScalarT(0))) != hull.end());
}

TEST(GrahamsScanTest, SortCCW) {
  Point pivot(ScalarT(0), ScalarT(0));
  PointsT points = {
      Point(ScalarT(1), ScalarT(1)), Point(ScalarT(2), ScalarT(0)),
      Point(ScalarT(1), ScalarT(-1)), Point(ScalarT(0), ScalarT(-1))};

  GrahamsScan::SortCCW(points, pivot);

  Vector v1 = points[0] - pivot;
  Vector v2 = points[1] - pivot;
  Vector v3 = points[2] - pivot;
  Vector v4 = points[3] - pivot;

  EXPECT_GT(v1.VectorProduct(v2), ScalarT(0));
  EXPECT_GT(v2.VectorProduct(v3), ScalarT(0));
  EXPECT_GT(v3.VectorProduct(v4), ScalarT(0));
}

TEST(GrahamsScanTest, GetPivot) {
  PointsT points = {
      Point(ScalarT(1), ScalarT(2)), Point(ScalarT(0), ScalarT(0)),
      Point(ScalarT(2), ScalarT(0)), Point(ScalarT(1), ScalarT(1))};

  Point pivot = GrahamsScan::GetPivot(points);
  EXPECT_EQ(pivot, Point(ScalarT(0), ScalarT(0)));
}

TEST(CrossProductTest, BasicCases) {
  EXPECT_EQ(CrossProduct(geometry::Point{0, 0}, geometry::Point{1, 0},
                         geometry::Point{2, 0}),
            ScalarT(0));
  EXPECT_EQ(CrossProduct(geometry::Point{0, 0}, geometry::Point{0, 1},
                         geometry::Point{0, 2}),
            ScalarT(0));
  EXPECT_EQ(CrossProduct(geometry::Point{0, 0}, geometry::Point{1, 1},
                         geometry::Point{2, 2}),
            ScalarT(0));
}

TEST(CrossProductTest, NonZeroValues) {
  EXPECT_EQ(CrossProduct(geometry::Point{0, 0}, geometry::Point{1, 0},
                         geometry::Point{0, 1}),
            ScalarT(1));
  EXPECT_EQ(CrossProduct(geometry::Point{1, 1}, geometry::Point{3, 1},
                         geometry::Point{2, 3}),
            ScalarT(4));

  EXPECT_EQ(CrossProduct(geometry::Point{0, 0}, geometry::Point{0, 1},
                         geometry::Point{1, 0}),
            ScalarT(-1));
  EXPECT_EQ(CrossProduct(geometry::Point{1, 1}, geometry::Point{2, 3},
                         geometry::Point{3, 1}),
            ScalarT(-4));
}

TEST(CrossProductTest, DegenerateCases) {
  EXPECT_EQ(CrossProduct(geometry::Point{1, 1}, geometry::Point{1, 1},
                         geometry::Point{1, 1}),
            ScalarT(0));
  EXPECT_EQ(CrossProduct(geometry::Point{1, 1}, geometry::Point{1, 1},
                         geometry::Point{2, 2}),
            ScalarT(0));
  EXPECT_EQ(CrossProduct(geometry::Point{1, 1}, geometry::Point{2, 2},
                         geometry::Point{2, 2}),
            ScalarT(0));
  EXPECT_EQ(CrossProduct(geometry::Point{0, 0}, geometry::Point{2, 2},
                         geometry::Point{1, 1}),
            ScalarT(0));
}

TEST(CrossProductTest, PrecisionCases) {
  ScalarT big = 1e10L;
  EXPECT_EQ(CrossProduct(geometry::Point{0, 0}, geometry::Point{big, 0},
                         geometry::Point{0, big}),
            big * big);
  ScalarT small = 1e-10L;
  EXPECT_EQ(CrossProduct(geometry::Point{0, 0}, geometry::Point{small, 0},
                         geometry::Point{0, small}),
            small * small);
}

TEST(CrossProductTest, NonStandardCases) {
  EXPECT_EQ(CrossProduct(geometry::Point{-1, -1}, geometry::Point{1, -1},
                         geometry::Point{0, 1}),
            ScalarT(4));
  EXPECT_EQ(CrossProduct(geometry::Point{-2, 3}, geometry::Point{1, 1},
                         geometry::Point{4, 5}),
            ScalarT(18));
  EXPECT_EQ(CrossProduct(geometry::Point{0, 0}, geometry::Point{0, 5},
                         geometry::Point{3, 0}),
            ScalarT(-15));
  EXPECT_EQ(CrossProduct(geometry::Point{0, 0}, geometry::Point{3, 0},
                         geometry::Point{0, 5}),
            ScalarT(15));
}

TEST(CrossProductTest, AreaProperty) {
  Point a(1, 1), b(4, 2), c(3, 5);
  ScalarT area = CrossProduct(a, b, c);
  EXPECT_EQ(area, ScalarT(10));
  Point offset(10, 10);
  EXPECT_EQ(CrossProduct(a + offset, b + offset, c + offset), area);
}

TEST(RotateRayRadianTest, NoRotation) {
  Ray ray(Point(0, 0), Point(1, 0));
  Ray rotated = RotateRayRadian(ray, ScalarT(0));

  EXPECT_EQ(rotated.GetDirection().GetX(), ScalarT(1));
  EXPECT_EQ(rotated.GetDirection().GetY(), ScalarT(0));
}

TEST(RotateRayRadianTest, QuarterTurnCounterClockwise) {
  Ray ray(Point(0, 0), Point(1, 0));
  Ray rotated = RotateRayRadian(ray, ScalarT(M_PI / 2));

  EXPECT_EQ(rotated.GetDirection().GetX(), ScalarT(0));
  EXPECT_EQ(rotated.GetDirection().GetY(), ScalarT(1));
}

TEST(RotateRayRadianTest, QuarterTurnClockwise) {
  Ray ray(Point(0, 0), Point(1, 0));
  Ray rotated = RotateRayRadian(ray, ScalarT(M_PI / 2), true);

  EXPECT_EQ(rotated.GetDirection().GetX(), ScalarT(0));
  EXPECT_EQ(rotated.GetDirection().GetY(), ScalarT(-1));
}

TEST(RotateRayRadianTest, FullRotation) {
  Ray ray(Point(0, 0), Point(1, 0));
  Ray rotated = RotateRayRadian(ray, ScalarT(2 * M_PI));

  EXPECT_EQ(rotated.GetDirection().GetX(), ScalarT(1));
  EXPECT_EQ(rotated.GetDirection().GetY(), ScalarT(0));
}

TEST(RotateRayDegreeTest, NoRotation) {
  Ray ray(Point(0, 0), Point(1, 0));
  Ray rotated = RotateRayDegree(ray, ScalarT(0));

  EXPECT_EQ(rotated.GetDirection().GetX(), ScalarT(1));
  EXPECT_EQ(rotated.GetDirection().GetY(), ScalarT(0));
}

TEST(RotateRayDegreeTest, QuarterTurnCounterClockwise) {
  Ray ray(Point(0, 0), Point(1, 0));
  Ray rotated = RotateRayDegree(ray, ScalarT(90));

  EXPECT_EQ(rotated.GetDirection().GetX(), ScalarT(0));
  EXPECT_EQ(rotated.GetDirection().GetY(), ScalarT(1));
}

TEST(RotateRayDegreeTest, QuarterTurnClockwise) {
  Ray ray(Point(0, 0), Point(1, 0));
  Ray rotated = RotateRayDegree(ray, ScalarT(90), true);

  EXPECT_EQ(rotated.GetDirection().GetX(), ScalarT(0));
  EXPECT_EQ(rotated.GetDirection().GetY(), ScalarT(-1));
}

TEST(RotateRayDegreeTest, FullRotation) {
  Ray ray(Point(0, 0), Point(1, 0));
  Ray rotated = RotateRayDegree(ray, ScalarT(360));

  EXPECT_EQ(rotated.GetDirection().GetX(), ScalarT(1));
  EXPECT_EQ(rotated.GetDirection().GetY(), ScalarT(0));
}

TEST(CalcOrientationTest, Collinear) {
  Point a(0, 0);
  Point b(1, 1);
  Point c(2, 2);

  EXPECT_EQ(CalcOrientation(a, b, c), Orientation::Collinear);
}

TEST(CalcOrientationTest, CounterClockwise) {
  Point a(0, 0);
  Point b(1, 0);
  Point c(0, 1);

  EXPECT_EQ(CalcOrientation(a, b, c), Orientation::CounterClockwise);
}

TEST(CalcOrientationTest, Clockwise) {
  Point a(0, 0);
  Point b(0, 1);
  Point c(1, 0);

  EXPECT_EQ(CalcOrientation(a, b, c), Orientation::Clockwise);
}

TEST(CalcOrientationTest, CollinearVertical) {
  Point a(0, 0);
  Point b(0, 1);
  Point c(0, 2);

  EXPECT_EQ(CalcOrientation(a, b, c), Orientation::Collinear);
}

TEST(CalcOrientationTest, CollinearHorizontal) {
  Point a(0, 0);
  Point b(1, 0);
  Point c(2, 0);

  EXPECT_EQ(CalcOrientation(a, b, c), Orientation::Collinear);
}

TEST(CalcOrientationTest, CounterClockwiseComplex) {
  Point a(1, 1);
  Point b(3, 5);
  Point c(2, 4);

  EXPECT_EQ(CalcOrientation(a, b, c), Orientation::CounterClockwise);
}

TEST(CalcOrientationTest, ClockwiseComplex) {
  Point a(1, 1);
  Point b(2, 4);
  Point c(3, 5);

  EXPECT_EQ(CalcOrientation(a, b, c), Orientation::Clockwise);
}

TEST(RotateRayRadianTest, OriginPreserved) {
  Point origin(5, 5);
  Ray ray(origin, Point(1, 0));
  Ray rotated = RotateRayRadian(ray, M_PI / 4);

  EXPECT_EQ(rotated.GetOrigin(), origin);
}

TEST(RotateRayDegreeTest, OriginPreserved) {
  Point origin(5, 5);
  Ray ray(origin, Point(1, 0));
  Ray rotated = RotateRayDegree(ray, 45);

  EXPECT_EQ(rotated.GetOrigin(), origin);
}

TEST(IsIntersectTest, PointToPoint) {
  EXPECT_TRUE(IsIntersect(geometry::Point{1, 2}, geometry::Point{1, 2}));

  EXPECT_FALSE(IsIntersect(geometry::Point{1, 2}, geometry::Point{1, 3}));
  EXPECT_FALSE(IsIntersect(geometry::Point{1, 2}, geometry::Point{2, 2}));
  EXPECT_FALSE(IsIntersect(geometry::Point{1, 2}, geometry::Point{3, 4}));
}

TEST(IsIntersectTest, PointToSegment) {
  Segment seg(geometry::Point{0, 0}, geometry::Point{5, 5});

  EXPECT_TRUE(IsIntersect(geometry::Point{2.5, 2.5}, seg));
  EXPECT_TRUE(IsIntersect(geometry::Point{0, 0}, seg));
  EXPECT_TRUE(IsIntersect(geometry::Point{5, 5}, seg));

  EXPECT_FALSE(IsIntersect(geometry::Point{-1, -1}, seg));
  EXPECT_FALSE(IsIntersect(geometry::Point{6, 6}, seg));

  EXPECT_FALSE(IsIntersect(geometry::Point{2, 3}, seg));
  EXPECT_FALSE(IsIntersect(geometry::Point{3, 2}, seg));

  Segment vert_seg(geometry::Point{2, 0}, geometry::Point{2, 4});
  EXPECT_TRUE(IsIntersect(geometry::Point{2, 2}, vert_seg));
  EXPECT_FALSE(IsIntersect(geometry::Point{2, 5}, vert_seg));
  EXPECT_FALSE(IsIntersect(geometry::Point{3, 2}, vert_seg));

  Segment hor_seg(geometry::Point{0, 3}, geometry::Point{5, 3});
  EXPECT_TRUE(IsIntersect(geometry::Point{2.5, 3}, hor_seg));
  EXPECT_FALSE(IsIntersect(geometry::Point{2.5, 4}, hor_seg));
  EXPECT_FALSE(IsIntersect(geometry::Point{6, 3}, hor_seg));
}

TEST(IsIntersectTest, SegmentToPoint) {
  Segment seg(geometry::Point{1, 1}, geometry::Point{4, 4});
  Point p(2.5, 2.5);

  EXPECT_EQ(IsIntersect(p, seg), IsIntersect(seg, p));
  EXPECT_EQ(IsIntersect(geometry::Point{0, 0}, seg),
            IsIntersect(seg, geometry::Point{0, 0}));
}

TEST(IsIntersectTest, ThreePointVersion) {
  Point a(0, 0), b(3, 3), p1(1.5, 1.5), p2(4, 4);

  EXPECT_TRUE(IsIntersect(a, p1, b));
  EXPECT_FALSE(IsIntersect(a, p2, b));
  EXPECT_TRUE(IsIntersect(a, a, b));
  EXPECT_TRUE(IsIntersect(a, b, b));
}

TEST(IsIntersectTest, EdgeCases) {
  Segment point_seg(geometry::Point{2, 2}, geometry::Point{2, 2});
  EXPECT_TRUE(IsIntersect(geometry::Point{2, 2}, point_seg));
  EXPECT_FALSE(IsIntersect(geometry::Point{2, 3}, point_seg));

  Segment same_x(geometry::Point{1, 0}, geometry::Point{1, 5});
  EXPECT_TRUE(IsIntersect(geometry::Point{1, 3}, same_x));
  EXPECT_FALSE(IsIntersect(geometry::Point{2, 3}, same_x));

  Segment same_y(geometry::Point{0, 1}, geometry::Point{5, 1});
  EXPECT_TRUE(IsIntersect(geometry::Point{3, 1}, same_y));
  EXPECT_FALSE(IsIntersect(geometry::Point{3, 2}, same_y));
}

TEST(IsIntersectTest, SegmentToSegment) {
  Segment seg1(geometry::Point{0, 0}, geometry::Point{2, 2});
  Segment seg2(geometry::Point{0, 2}, geometry::Point{2, 0});
  EXPECT_TRUE(IsIntersect(seg1, seg2));

  Segment seg3(geometry::Point{0, 0}, geometry::Point{1, 1});
  Segment seg4(geometry::Point{2, 2}, geometry::Point{3, 3});
  EXPECT_FALSE(IsIntersect(seg3, seg4));

  Segment seg5(geometry::Point{0, 0}, geometry::Point{1, 1});
  Segment seg6(geometry::Point{1, 1}, geometry::Point{2, 0});
  EXPECT_TRUE(IsIntersect(seg5, seg6));

  Segment seg7(geometry::Point{0, 0}, geometry::Point{1, 1});
  Segment seg8(geometry::Point{2, 2}, geometry::Point{3, 3});
  EXPECT_FALSE(IsIntersect(seg7, seg8));

  Segment seg9(geometry::Point{0, 0}, geometry::Point{2, 2});
  Segment seg10(geometry::Point{1, 1}, geometry::Point{3, 3});
  EXPECT_TRUE(IsIntersect(seg9, seg10));
}

TEST(IsIntersectTest, LineToLine) {
  Line line1(1, -1, 0);
  Line line2(1, 1, 0);
  EXPECT_TRUE(IsIntersect(line1, line2));

  Line line3(1, -1, 0);
  Line line4(1, -1, 1);
  EXPECT_FALSE(IsIntersect(line3, line4));

  Line line5(1, -1, 0);
  Line line6(2, -2, 0);
  EXPECT_TRUE(IsIntersect(line5, line6));
}

TEST(IsIntersectTest, RayToLine) {
  Line line(1, 0, -3);
  Ray ray1(geometry::Point{0, 0}, geometry::Point{1, 0});
  EXPECT_TRUE(IsIntersect(ray1, line));

  Ray ray2(geometry::Point{0, 0}, geometry::Point{-1, 0});
  EXPECT_FALSE(IsIntersect(ray2, line));

  Ray ray3(geometry::Point{3, 0}, geometry::Point{0, 1});
  EXPECT_TRUE(IsIntersect(ray3, line));

  Ray ray4(geometry::Point{0, 0}, geometry::Point{0, 1});
  EXPECT_FALSE(IsIntersect(ray4, line));
}

TEST(IsIntersectTest, RayToRay) {
  Ray ray1(geometry::Point{0, 0}, geometry::Point{1, 0});
  Ray ray2(geometry::Point{2, 0}, geometry::Point{-1, 0});
  EXPECT_FALSE(IsIntersect(ray1, ray2));

  Ray ray3(geometry::Point{0, 0}, geometry::Point{1, 0});
  Ray ray4(geometry::Point{0, 1}, geometry::Point{1, 0});
  EXPECT_FALSE(IsIntersect(ray3, ray4));

  Ray ray5(geometry::Point{0, 0}, geometry::Point{1, 0});
  Ray ray6(geometry::Point{2, 0}, geometry::Point{1, 0});
  EXPECT_TRUE(IsIntersect(ray5, ray6));
}

TEST(IntersectTest, RayToLine) {
  Line line(1, -1, 0);
  Ray ray(geometry::Point{0, 1}, geometry::Point{1, -1});

  auto intersection = Intersect(ray, line);
  ASSERT_TRUE(intersection.has_value());
  EXPECT_EQ(intersection.value(), Point(0.5, 0.5));

  Ray ray2(geometry::Point{0, 1}, geometry::Point{1, 0});
  EXPECT_TRUE(Intersect(ray2, line).has_value());

  Ray ray3(geometry::Point{1, 1}, geometry::Point{1, 0});
  auto intersection2 = Intersect(ray3, line);
  ASSERT_TRUE(intersection2.has_value());
  EXPECT_EQ(intersection2.value(), Point(1.0L, 1.0L));
}

TEST(EdgeCasesTest, SpecialCases) {
  Segment vert1(geometry::Point{1, 0}, geometry::Point{1, 5});
  Segment vert2(geometry::Point{1, 3}, geometry::Point{1, 7});
  EXPECT_TRUE(IsIntersect(vert1, vert2));

  Segment hor1(geometry::Point{0, 2}, geometry::Point{5, 2});
  Segment hor2(geometry::Point{3, 2}, geometry::Point{7, 2});
  EXPECT_TRUE(IsIntersect(hor1, hor2));

  Segment point1(geometry::Point{2, 2}, geometry::Point{2, 2});
  Segment point2(geometry::Point{2, 2}, geometry::Point{2, 2});
  EXPECT_TRUE(IsIntersect(point1, point2));
}

TEST(IsIntersectTest, PointInConvexConvexHull) {
  ConvexHull pentagon({geometry::Point{0, 0}, geometry::Point{2, 0},
                       geometry::Point{3, 2}, geometry::Point{1, 4},
                       geometry::Point{-1, 2}});

  EXPECT_TRUE(IsIntersect(pentagon, geometry::Point{1, 2}));
  EXPECT_TRUE(IsIntersect(geometry::Point{2, 1}, pentagon));

  EXPECT_FALSE(IsIntersect(pentagon, geometry::Point{4, 4}));
  EXPECT_FALSE(IsIntersect(geometry::Point{0, -1}, pentagon));

  EXPECT_TRUE(IsIntersect(pentagon, geometry::Point{1.5, 3}));
  EXPECT_TRUE(IsIntersect(geometry::Point{0, 0}, pentagon));

  ConvexHull point_poly({geometry::Point{3, 3}});
  EXPECT_TRUE(IsIntersect(point_poly, geometry::Point{3, 3}));
  EXPECT_FALSE(IsIntersect(point_poly, geometry::Point{3, 4}));

  ConvexHull segment_poly({geometry::Point{1, 1}, geometry::Point{3, 3}});
  EXPECT_TRUE(IsIntersect(segment_poly, geometry::Point{2, 2}));
  EXPECT_FALSE(IsIntersect(segment_poly, geometry::Point{4, 4}));
}

TEST(IsIntersectBorderTest, PointOnConvexHullBorder) {
  ConvexHull quad({geometry::Point{0, 0}, geometry::Point{4, 0},
                   geometry::Point{4, 2}, geometry::Point{2, 4},
                   geometry::Point{0, 2}});

  EXPECT_TRUE(IsIntersectBorder(quad, geometry::Point{2, 0}));
  EXPECT_TRUE(IsIntersectBorder(geometry::Point{4, 1}, quad));

  EXPECT_FALSE(IsIntersectBorder(quad, geometry::Point{2, 1}));
  EXPECT_FALSE(IsIntersectBorder(geometry::Point{1, 1}, quad));

  EXPECT_FALSE(IsIntersectBorder(quad, geometry::Point{5, 5}));
  EXPECT_FALSE(IsIntersectBorder(geometry::Point{-1, 0}, quad));

  EXPECT_TRUE(IsIntersectBorder(quad, geometry::Point{0, 0}));
  EXPECT_TRUE(IsIntersectBorder(geometry::Point{2, 4}, quad));
}

TEST(IsIntersectTest, PointInConvexConvexHullEdgeCases) {
  ConvexHull thin({geometry::Point{0, 0}, geometry::Point{100, 0},
                   geometry::Point{100, 0.0001L}});
  EXPECT_TRUE(IsIntersect(thin, geometry::Point{50, 0.00005L}));
  EXPECT_FALSE(IsIntersect(thin, geometry::Point{50, 0.0002L}));

  ConvexHull collinear({geometry::Point{0, 0}, geometry::Point{1, 1},
                        geometry::Point{2, 2}, geometry::Point{3, 3}});
  EXPECT_TRUE(IsIntersect(collinear, geometry::Point{1.5, 1.5}));
  EXPECT_FALSE(IsIntersect(collinear, geometry::Point{1.5, 1.6}));
}

TEST(HalfPlaneTest, ConvexConvexHullHalfPlane) {
  HalfPlane hp(1, 0, -2);

  ConvexHull inside({geometry::Point{2, 0}, geometry::Point{4, 0},
                     geometry::Point{4, 2}, geometry::Point{2, 2}});
  EXPECT_TRUE(std::all_of(inside.begin(), inside.end(), [&hp](const Point& p) {
    return IsIntersect(hp, p);
  }));

  ConvexHull intersecting({geometry::Point{1, 0}, geometry::Point{3, 0},
                           geometry::Point{3, 2}, geometry::Point{1, 2}});
  bool has_inside = false, has_outside = false;
  for (const auto& p : intersecting) {
    if (IsIntersect(hp, p))
      has_inside = true;
    else
      has_outside = true;
  }
  EXPECT_TRUE(has_inside && has_outside);

  ConvexHull outside({geometry::Point{0, 0}, geometry::Point{1, 0},
                      geometry::Point{1, 1}, geometry::Point{0, 1}});
  EXPECT_TRUE(
      std::none_of(outside.begin(), outside.end(),
                   [&hp](const Point& p) { return IsIntersect(hp, p); }));
}

TEST(IntersectTest, RayLineIntersection) {
  Line line(1, -1, 0);
  Ray ray1(geometry::Point{0, 1}, geometry::Point{1, -1});

  auto intersect1 = Intersect(ray1, line);
  ASSERT_TRUE(intersect1.has_value());
  EXPECT_EQ(intersect1.value(), Point(0.5L, 0.5L));

  Ray ray2(geometry::Point{0, 1}, geometry::Point{1, 1});
  EXPECT_FALSE(Intersect(ray2, line).has_value());

  Ray ray3(geometry::Point{1, 1}, geometry::Point{1, 0});
  auto intersect3 = Intersect(ray3, line);
  ASSERT_TRUE(intersect3.has_value());
  EXPECT_EQ(intersect3.value(), Point(1.0L, 1.0L));

  Ray ray4(geometry::Point{0, 1}, geometry::Point{-1, 1});
  EXPECT_FALSE(Intersect(ray4, line).has_value());
}

TEST(IntersectTest, HalfPlaneSegmentIntersection) {
  HalfPlane hp(1, 0, -2);  // x >= 2

  auto intersect1 = Intersect(hp, geometry::Point{1, 0}, geometry::Point{3, 0});
  ASSERT_TRUE(intersect1.has_value());
  EXPECT_EQ(intersect1.value(), Point(2.0L, 0.0L));

  auto intersect2 = Intersect(hp, geometry::Point{3, 1}, geometry::Point{4, 2});
  EXPECT_FALSE(intersect2.has_value());

  auto intersect3 = Intersect(hp, geometry::Point{0, 0}, geometry::Point{1, 1});
  EXPECT_FALSE(intersect3.has_value());

  auto intersect4 = Intersect(hp, geometry::Point{1, 0}, geometry::Point{1, 1});
  EXPECT_FALSE(intersect4.has_value());
}

TEST(IntersectTest, ConvexHullHalfPlaneIntersection) {
  ConvexHull convex_hull({geometry::Point{0, 0}, geometry::Point{4, 0},
                          geometry::Point{4, 4}, geometry::Point{0, 4}});
  HalfPlane hp(1, 0, -2);  // x >= 2

  ConvexHull intersected = Intersect(convex_hull, hp);
  ASSERT_EQ(intersected.GetSize(), 4u);

  EXPECT_NE(std::find(intersected.begin(), intersected.end(), Point(2, 0)),
            intersected.end());
  EXPECT_NE(std::find(intersected.begin(), intersected.end(), Point(2, 4)),
            intersected.end());

  HalfPlane hp2(1, 0, -5);  // x >= 5
  ConvexHull empty = Intersect(convex_hull, hp2);
  EXPECT_TRUE(empty.IsEmpty());

  HalfPlane hp3(-1, 0, 1);  // x <= 1
  ConvexHull full = Intersect(convex_hull, hp3);
  ASSERT_EQ(full.GetSize(), convex_hull.GetSize());
}

TEST(IntersectTest, MultipleHalfPlanesIntersection) {
  PointsT initial = {geometry::Point{0, 0}, geometry::Point{10, 0},
                     geometry::Point{10, 10}, geometry::Point{0, 10}};
  HalfPlanesT half_planes = {
      {1, 0, -2},  // x >= 2
      {-1, 0, 7},  // x <= 7
      {0, 1, -3},  // y >= 3
      {0, -1, 8}   // y <= 8
  };

  auto result = Intersect(initial, half_planes);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->GetSize(), 4u);

  ConvexHull result_poly(result.value());
  EXPECT_FALSE(result_poly.IsDegenerate());

  HalfPlanesT restrictive = {{1, 0, -20}, {0, 1, -30}};
  auto empty_result = Intersect(initial, restrictive);
  EXPECT_FALSE(empty_result.has_value());
}

TEST(EdgeCasesTest, SpecialIntersectionCases) {
  PointsT segment = {geometry::Point{1, 1}, geometry::Point{3, 3}};

  HalfPlane hp(1, -1, 0);

  auto seg_intersect = Intersect(ConvexHull(std::move(segment)), hp);
  ASSERT_EQ(seg_intersect.GetSize(), 2u);
  EXPECT_EQ(seg_intersect[0], Point(1, 1));
  EXPECT_EQ(seg_intersect[1], Point(3, 3));

  PointsT point = {geometry::Point{2, 2}};
  auto point_intersect = Intersect(ConvexHull(std::move(point)), hp);
  EXPECT_EQ(point_intersect.GetSize(), 1u);

  HalfPlane tangent(1, 1, -4);
  PointsT square = {geometry::Point{0, 0}, geometry::Point{2, 0},
                    geometry::Point{2, 2}, geometry::Point{0, 2}};

  auto tangent_intersect = Intersect(ConvexHull(std::move(square)), tangent);
  ASSERT_EQ(tangent_intersect.GetSize(), 1u);
  EXPECT_NE(std::find(tangent_intersect.begin(), tangent_intersect.end(),
                      Point(2, 2)),
            tangent_intersect.end());
}

TEST(PrecisionTest, FloatingPointIntersection) {
  PointsT convex_hull = {geometry::Point{0, 0}, geometry::Point{1, 0},
                         geometry::Point{1, 1}, geometry::Point{0, 1}};
  HalfPlane hp(1, 1, -1);

  auto intersection = Intersect(ConvexHull(std::move(convex_hull)), hp);
  ASSERT_FALSE(intersection.IsEmpty());

  bool has_origin = std::any_of(
      intersection.begin(), intersection.end(),
      [](const Point& p) { return p == Point(1, 0) || p == Point(0, 1); });
  EXPECT_TRUE(has_origin);
}

TEST(CalcDistance, PointToPoint) {
  Point p1(0, 0);
  Point p2(3, 4);
  EXPECT_EQ(CalcDistance(p1, p2), ScalarT(5.0));

  Point p3(-1, -1);
  Point p4(1, 1);
  EXPECT_EQ(CalcDistance(p3, p4), ScalarT(std::sqrt(8)));

  EXPECT_EQ(CalcDistance(p1, p1), ScalarT(0.0));
}

TEST(CalcDistance, PointToSegment) {
  Segment seg(Point(0, 0), Point(4, 0));

  EXPECT_EQ(CalcDistance(Point(2, 0), seg), ScalarT(0.0));

  EXPECT_EQ(CalcDistance(Point(2, 3), seg), ScalarT(3.0));

  EXPECT_EQ(CalcDistance(Point(5, 0), seg), ScalarT(1.0));

  EXPECT_EQ(CalcDistance(Point(-1, 0), seg), ScalarT(1.0));

  Segment deg_seg(Point(1, 1), Point(1, 1));
  EXPECT_EQ(CalcDistance(Point(1, 2), deg_seg), ScalarT(1.0));
}

TEST(CalcDistance, SegmentToSegment) {
  Segment seg1(Point(0, 0), Point(2, 2));
  Segment seg2(Point(0, 2), Point(2, 0));
  EXPECT_EQ(CalcDistance(seg1, seg2), ScalarT(0.0));

  Segment seg3(Point(0, 1), Point(2, 3));  // y = x + 1 (параллелен seg1)
  EXPECT_EQ(CalcDistance(seg1, seg3), ScalarT(std::sqrt(2) / 2));

  Segment seg4(Point(3, 0), Point(3, 3));
  EXPECT_EQ(CalcDistance(seg1, seg4), ScalarT(1.0));

  Segment seg5(Point(0, 3), Point(0, 4));
  EXPECT_EQ(CalcDistance(seg1, seg5), ScalarT(std::sqrt(4.5)));
}

TEST(CalcDistance, PointToConvexHull) {
  ConvexHull triangle({Point(0, 0), Point(2, 0), Point(1, 2)});

  EXPECT_EQ(CalcDistance(Point(1, -1), triangle), ScalarT(1.0));

  EXPECT_EQ(CalcDistance(Point(3, 0), triangle), ScalarT(1.0));

  ConvexHull point_poly({Point(1, 1)});
  EXPECT_EQ(CalcDistance(Point(1, 2), point_poly), ScalarT(1.0));
}

TEST(ConvexHullTest, ConstructorsAndBasicProperties) {
  ConvexHull p1;
  EXPECT_TRUE(p1.IsEmpty());
  EXPECT_EQ(p1.GetSize(), 0);

  PointsT points = {
      Point(ScalarT(0), ScalarT(0)), Point(ScalarT(1), ScalarT(0)),
      Point(ScalarT(1), ScalarT(1)), Point(ScalarT(0), ScalarT(1))};
  ConvexHull p2(points);
  EXPECT_EQ(p2.GetSize(), 4);
  EXPECT_FALSE(p2.IsEmpty());
}

TEST(ConvexHullTest, GeometryOperations) {
  ConvexHull square(
      {Point(ScalarT(0), ScalarT(0)), Point(ScalarT(2), ScalarT(0)),
       Point(ScalarT(2), ScalarT(2)), Point(ScalarT(0), ScalarT(2))});
  EXPECT_EQ(square.GetArea(), ScalarT(4));
  EXPECT_FALSE(square.IsDegenerate());

  ConvexHull line({Point(ScalarT(0), ScalarT(0)), Point(ScalarT(1), ScalarT(1)),
                   Point(ScalarT(2), ScalarT(2))});
  EXPECT_TRUE(line.IsDegenerate());
}

TEST(ConvexHullTest, Iterators) {
  ConvexHull square(
      {Point(ScalarT(0), ScalarT(0)), Point(ScalarT(1), ScalarT(0)),
       Point(ScalarT(1), ScalarT(1)), Point(ScalarT(0), ScalarT(1))});

  EXPECT_EQ(std::distance(square.begin(), square.end()), 4);

  size_t edge_count = 0;
  for (auto it = square.EdgeBegin(); it != square.EdgeEnd(); ++it) {
    edge_count++;
  }
  EXPECT_EQ(edge_count, 4);
}

TEST(VoronoiDiagramTest, BasicConstruction) {
  PointsT borders = {
      Point(ScalarT(-10), ScalarT(-10)), Point(ScalarT(10), ScalarT(-10)),
      Point(ScalarT(10), ScalarT(10)), Point(ScalarT(-10), ScalarT(10))};

  PointsT sites = {Point(ScalarT(0), ScalarT(0)),
                   Point(ScalarT(1), ScalarT(1))};

  VoronoiDiagram vd(borders, sites);
  const auto& cells = vd.GetCells();

  EXPECT_EQ(cells.size(), 2);
  for (const auto& cell : cells) {
    EXPECT_GE(cell.GetSize(), 3);
    EXPECT_FALSE(cell.IsDegenerate());
  }
}

TEST(VoronoiDiagramTest, CellValidation) {
  PointsT borders = {
      Point(ScalarT(0), ScalarT(0)), Point(ScalarT(2), ScalarT(0)),
      Point(ScalarT(2), ScalarT(2)), Point(ScalarT(0), ScalarT(2))};

  PointsT sites = {Point(ScalarT(1), ScalarT(1))};
  VoronoiDiagram vd(borders, sites);

  EXPECT_EQ(vd.GetCells().size(), 1);
  const auto& cell = vd.GetCells()[0];
  EXPECT_GE(cell.GetSize(), 3);
  EXPECT_GT(cell.GetArea(), ScalarT(0));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
