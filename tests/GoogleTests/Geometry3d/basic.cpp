#include <gtest/gtest.h>

#include "Geometry/Geometry3d.hpp"

using namespace geometry3d;

TEST(VectorTest, ConstructorsAndGetters) {
  Vector v1;
  EXPECT_EQ(v1.GetX(), ScalarT(0));
  EXPECT_EQ(v1.GetY(), ScalarT(0));
  EXPECT_EQ(v1.GetZ(), ScalarT(0));

  Vector v2(ScalarT(1), ScalarT(2), ScalarT(3));
  EXPECT_EQ(v2.GetX(), ScalarT(1));
  EXPECT_EQ(v2.GetY(), ScalarT(2));
  EXPECT_EQ(v2.GetZ(), ScalarT(3));

  Vector::CoordTuple t = {ScalarT(4), ScalarT(5), ScalarT(6)};
  Vector v3(t);
  EXPECT_EQ(v3.GetX(), ScalarT(4));
  EXPECT_EQ(v3.GetY(), ScalarT(5));
  EXPECT_EQ(v3.GetZ(), ScalarT(6));

  Vector v4(v2);
  EXPECT_EQ(v4.GetX(), ScalarT(1));
  EXPECT_EQ(v4.GetY(), ScalarT(2));
  EXPECT_EQ(v4.GetZ(), ScalarT(3));

  Vector v5(std::move(v3));
  EXPECT_EQ(v5.GetX(), ScalarT(4));
  EXPECT_EQ(v5.GetY(), ScalarT(5));
  EXPECT_EQ(v5.GetZ(), ScalarT(6));

  auto [x, y, z] = v2.GetAsTuple();
  EXPECT_EQ(x, ScalarT(1));
  EXPECT_EQ(y, ScalarT(2));
  EXPECT_EQ(z, ScalarT(3));
}

TEST(VectorTest, AssignmentOperators) {
  Vector v1(ScalarT(1), ScalarT(2), ScalarT(3));
  Vector v2;

  v2 = v1;
  EXPECT_EQ(v2.GetX(), ScalarT(1));
  EXPECT_EQ(v2.GetY(), ScalarT(2));
  EXPECT_EQ(v2.GetZ(), ScalarT(3));

  Vector v3;
  v3 = std::move(v1);
  EXPECT_EQ(v3.GetX(), ScalarT(1));
  EXPECT_EQ(v3.GetY(), ScalarT(2));
  EXPECT_EQ(v3.GetZ(), ScalarT(3));
}

TEST(VectorTest, ArithmeticOperators) {
  Vector v1(ScalarT(1), ScalarT(2), ScalarT(3));
  Vector v2(ScalarT(4), ScalarT(5), ScalarT(6));

  Vector sum = v1 + v2;
  EXPECT_EQ(sum.GetX(), ScalarT(5));
  EXPECT_EQ(sum.GetY(), ScalarT(7));
  EXPECT_EQ(sum.GetZ(), ScalarT(9));

  Vector diff = v2 - v1;
  EXPECT_EQ(diff.GetX(), ScalarT(3));
  EXPECT_EQ(diff.GetY(), ScalarT(3));
  EXPECT_EQ(diff.GetZ(), ScalarT(3));

  Vector scaled = v1 * ScalarT(2);
  EXPECT_EQ(scaled.GetX(), ScalarT(2));
  EXPECT_EQ(scaled.GetY(), ScalarT(4));
  EXPECT_EQ(scaled.GetZ(), ScalarT(6));

  Vector divided = v2 / ScalarT(2);
  EXPECT_EQ(divided.GetX(), ScalarT(2));
  EXPECT_EQ(divided.GetY(), ScalarT(2.5));
  EXPECT_EQ(divided.GetZ(), ScalarT(3));

  Vector neg = -v1;
  EXPECT_EQ(neg.GetX(), ScalarT(-1));
  EXPECT_EQ(neg.GetY(), ScalarT(-2));
  EXPECT_EQ(neg.GetZ(), ScalarT(-3));
}

TEST(VectorTest, CompoundAssignmentOperators) {
  Vector v1(ScalarT(1), ScalarT(2), ScalarT(3));
  Vector v2(ScalarT(4), ScalarT(5), ScalarT(6));

  v1 += v2;
  EXPECT_EQ(v1.GetX(), ScalarT(5));
  EXPECT_EQ(v1.GetY(), ScalarT(7));
  EXPECT_EQ(v1.GetZ(), ScalarT(9));

  v1 -= v2;
  EXPECT_EQ(v1.GetX(), ScalarT(1));
  EXPECT_EQ(v1.GetY(), ScalarT(2));
  EXPECT_EQ(v1.GetZ(), ScalarT(3));

  v1 *= ScalarT(2);
  EXPECT_EQ(v1.GetX(), ScalarT(2));
  EXPECT_EQ(v1.GetY(), ScalarT(4));
  EXPECT_EQ(v1.GetZ(), ScalarT(6));

  v1 /= ScalarT(2);
  EXPECT_EQ(v1.GetX(), ScalarT(1));
  EXPECT_EQ(v1.GetY(), ScalarT(2));
  EXPECT_EQ(v1.GetZ(), ScalarT(3));
}

TEST(VectorTest, ComparisonOperators) {
  Vector v1(ScalarT(1), ScalarT(2), ScalarT(3));
  Vector v2(ScalarT(1), ScalarT(2), ScalarT(3));
  Vector v3(ScalarT(4), ScalarT(5), ScalarT(6));

  EXPECT_TRUE(v1 == v2);
  EXPECT_FALSE(v1 == v3);

  EXPECT_FALSE(v1 != v2);
  EXPECT_TRUE(v1 != v3);

  EXPECT_TRUE(v1 < v3);
  EXPECT_TRUE(v3 > v1);
  EXPECT_TRUE(v1 <= v2);
  EXPECT_TRUE(v1 >= v2);
}

TEST(VectorTest, IOOperators) {
  Vector v1(ScalarT(1), ScalarT(2), ScalarT(3));

  std::ostringstream out;
  out << v1;
  EXPECT_EQ(out.str(), "1 2 3");

  Vector v2;
  std::istringstream in("4 5 6");
  in >> v2;
  EXPECT_EQ(v2.GetX(), ScalarT(4));
  EXPECT_EQ(v2.GetY(), ScalarT(5));
  EXPECT_EQ(v2.GetZ(), ScalarT(6));
}

TEST(VectorTest, GeometricOperations) {
  Vector v1(ScalarT(1), ScalarT(0), ScalarT(0));
  Vector v2(ScalarT(0), ScalarT(1), ScalarT(0));
  Vector v3(ScalarT(1), ScalarT(1), ScalarT(1));

  EXPECT_EQ(v1.Length(), ScalarT(1));
  EXPECT_EQ(v3.Length(), ScalarT(std::sqrt(3)));

  EXPECT_EQ(v1.SquaredLength(), ScalarT(1));
  EXPECT_EQ(v3.SquaredLength(), ScalarT(3));

  EXPECT_EQ(v1.ScalarProduct(v2), ScalarT(0));
  EXPECT_EQ(v1.ScalarProduct(v1), ScalarT(1));

  Vector cross = v1.VectorProduct(v2);
  EXPECT_EQ(cross.GetX(), ScalarT(0));
  EXPECT_EQ(cross.GetY(), ScalarT(0));
  EXPECT_EQ(cross.GetZ(), ScalarT(1));

  Vector normalized = v3.GetNormalized();
  EXPECT_EQ(normalized.Length(), ScalarT(1));
  EXPECT_EQ(normalized.GetX(), ScalarT(1) / ScalarT(std::sqrt(3)));
}

TEST(VectorTest, Hash) {
  Vector v1(ScalarT(1), ScalarT(2), ScalarT(3));
  Vector v2(ScalarT(1), ScalarT(2), ScalarT(3));
  Vector v3(ScalarT(4), ScalarT(5), ScalarT(6));

  Vector::Hash hasher;
  EXPECT_EQ(hasher(v1), hasher(v2));
  EXPECT_NE(hasher(v1), hasher(v3));
}

TEST(GeometryUtilsTest, CalcPlaneNormalSimple) {
  Point p1(ScalarT(0), ScalarT(0), ScalarT(0));
  Point p2(ScalarT(1), ScalarT(0), ScalarT(0));
  Point p3(ScalarT(0), ScalarT(1), ScalarT(0));

  Vector normal = CalcPlaneNormal(p1, p2, p3);

  EXPECT_EQ(normal.GetX(), ScalarT(0));
  EXPECT_EQ(normal.GetY(), ScalarT(0));
  EXPECT_EQ(normal.GetZ(), ScalarT(1));

  EXPECT_EQ(normal.Length(), ScalarT(1));
}

TEST(GeometryUtilsTest, CalcPlaneNormalReversedOrder) {
  Point p1(ScalarT(0), ScalarT(0), ScalarT(0));
  Point p2(ScalarT(0), ScalarT(1), ScalarT(0));
  Point p3(ScalarT(1), ScalarT(0), ScalarT(0));

  Vector normal = CalcPlaneNormal(p1, p2, p3);

  EXPECT_EQ(normal.GetX(), ScalarT(0));
  EXPECT_EQ(normal.GetY(), ScalarT(0));
  EXPECT_EQ(normal.GetZ(), ScalarT(-1));
}

TEST(GeometryUtilsTest, CalcPlaneNormalWithCentroidInside) {
  Point a(ScalarT(0), ScalarT(0), ScalarT(0));
  Point b(ScalarT(2), ScalarT(0), ScalarT(0));
  Point c(ScalarT(0), ScalarT(2), ScalarT(0));
  Point centroid(ScalarT(0.5), ScalarT(0.5), ScalarT(1));

  Vector normal = CalcPlaneNormal(a, b, c, centroid);

  EXPECT_EQ(normal.GetX(), ScalarT(0));
  EXPECT_EQ(normal.GetY(), ScalarT(0));
  EXPECT_EQ(normal.GetZ(), ScalarT(-1));
}

TEST(GeometryUtilsTest, CalcPlaneNormalWithCentroidOutside) {
  Point a(ScalarT(0), ScalarT(0), ScalarT(0));
  Point b(ScalarT(2), ScalarT(0), ScalarT(0));
  Point c(ScalarT(0), ScalarT(2), ScalarT(0));
  Point centroid(ScalarT(0.5), ScalarT(0.5), ScalarT(-1));

  Vector normal = CalcPlaneNormal(a, b, c, centroid);

  EXPECT_EQ(normal.GetX(), ScalarT(0));
  EXPECT_EQ(normal.GetY(), ScalarT(0));
  EXPECT_EQ(normal.GetZ(), ScalarT(1));
}

TEST(GeometryUtilsTest, CalcPlaneNormalNonPlanar) {
  Point p1(ScalarT(1), ScalarT(0), ScalarT(0));
  Point p2(ScalarT(0), ScalarT(1), ScalarT(0));
  Point p3(ScalarT(0), ScalarT(0), ScalarT(1));

  Vector normal = CalcPlaneNormal(p1, p2, p3);

  Vector expected(ScalarT(1), ScalarT(1), ScalarT(1));
  expected.Normalize();

  EXPECT_EQ(normal.GetX(), expected.GetX());
  EXPECT_EQ(normal.GetY(), expected.GetY());
  EXPECT_EQ(normal.GetZ(), expected.GetZ());
}

TEST(LineTest, ConstructorsAndGetters) {
  Point origin(ScalarT(0), ScalarT(0), ScalarT(0));
  Point direction(ScalarT(1), ScalarT(0), ScalarT(0));

  Line line(origin, direction);

  EXPECT_EQ(line.GetOrigin(), origin);
  EXPECT_EQ(line.GetDirection(), direction);
}

TEST(LineTest, AssignmentOperators) {
  Line line1({ScalarT(0), ScalarT(0), ScalarT(0)},
             {ScalarT(1), ScalarT(0), ScalarT(0)});
  Line line2 = line1;

  EXPECT_EQ(line2.GetOrigin(), line1.GetOrigin());
  EXPECT_EQ(line2.GetDirection(), line1.GetDirection());
}

TEST(PlaneTest, ConstructorsAndGetters) {
  Plane plane1({ScalarT(0), ScalarT(0), ScalarT(0)},
               {ScalarT(0), ScalarT(0), ScalarT(1)});
  EXPECT_EQ(plane1.GetOrigin(), Point(ScalarT(0), ScalarT(0), ScalarT(0)));
  EXPECT_EQ(plane1.GetNormal(), Vector(ScalarT(0), ScalarT(0), ScalarT(1)));

  Plane plane2({ScalarT(0), ScalarT(0), ScalarT(0)},
               {ScalarT(1), ScalarT(0), ScalarT(0)},
               {ScalarT(0), ScalarT(1), ScalarT(0)});

  EXPECT_EQ(plane2.GetOrigin(), Point(ScalarT(0), ScalarT(0), ScalarT(0)));
  EXPECT_EQ(plane2.GetNormal().GetZ(), ScalarT(1));
}

TEST(PlaneTest, GlobalPlanes) {
  EXPECT_EQ(Oxy.GetNormal(), Vector(ScalarT(0), ScalarT(0), ScalarT(-1)));
  EXPECT_EQ(Oxz.GetNormal(), Vector(ScalarT(0), ScalarT(-1), ScalarT(0)));
  EXPECT_EQ(Oyz.GetNormal(), Vector(ScalarT(-1), ScalarT(0), ScalarT(0)));
}

TEST(FaceTest, ConstructorsAndGetters) {
  Point p(ScalarT(0), ScalarT(0), ScalarT(0));
  Point q(ScalarT(1), ScalarT(0), ScalarT(0));
  Point r(ScalarT(0), ScalarT(1), ScalarT(0));

  Face face(p, q, r);

  EXPECT_EQ(face.GetP(), p);
  EXPECT_EQ(face.GetQ(), q);
  EXPECT_EQ(face.GetR(), r);
  EXPECT_EQ(face.GetNormal(), Vector(ScalarT(0), ScalarT(0), ScalarT(1)));
}

TEST(FaceTest, AssignmentOperators) {
  Face face1({ScalarT(0), ScalarT(0), ScalarT(0)},
             {ScalarT(1), ScalarT(0), ScalarT(0)},
             {ScalarT(0), ScalarT(1), ScalarT(0)});
  Face face2 = face1;

  EXPECT_EQ(face2.GetP(), face1.GetP());
  EXPECT_EQ(face2.GetQ(), face1.GetQ());
  EXPECT_EQ(face2.GetR(), face1.GetR());
  EXPECT_EQ(face2.GetNormal(), face1.GetNormal());
}

TEST(SegmentTest, ConstructorsAndGetters) {
  Point a(ScalarT(0), ScalarT(0), ScalarT(0));
  Point b(ScalarT(1), ScalarT(1), ScalarT(1));

  Segment segment(a, b);

  EXPECT_EQ(segment.GetA(), a);
  EXPECT_EQ(segment.GetB(), b);
  auto [a1, b1] = segment.GetAsTuple();
  EXPECT_EQ(a1, a);
  EXPECT_EQ(b1, b);
}

TEST(SegmentTest, ComparisonOperators) {
  Segment seg1({ScalarT(0), ScalarT(0), ScalarT(0)},
               {ScalarT(1), ScalarT(1), ScalarT(1)});
  Segment seg2({ScalarT(1), ScalarT(1), ScalarT(1)},
               {ScalarT(0), ScalarT(0), ScalarT(0)});
  Segment seg3({ScalarT(0), ScalarT(0), ScalarT(0)},
               {ScalarT(2), ScalarT(2), ScalarT(2)});

  EXPECT_TRUE(seg1 == seg2);
  EXPECT_FALSE(seg1 == seg3);
}

TEST(SegmentTest, GeometryMethods) {
  Segment seg1({ScalarT(0), ScalarT(0), ScalarT(0)},
               {ScalarT(1), ScalarT(1), ScalarT(1)});
  Segment seg2({ScalarT(0), ScalarT(0), ScalarT(0)},
               {ScalarT(0), ScalarT(0), ScalarT(0)});

  EXPECT_FALSE(seg1.IsDegenerate());
  EXPECT_TRUE(seg2.IsDegenerate());
}

TEST(SegmentTest, Hash) {
  Segment::Hash hasher;
  Segment seg1({ScalarT(0), ScalarT(0), ScalarT(0)},
               {ScalarT(1), ScalarT(1), ScalarT(1)});
  Segment seg2({ScalarT(1), ScalarT(1), ScalarT(1)},
               {ScalarT(0), ScalarT(0), ScalarT(0)});
  Segment seg3({ScalarT(0), ScalarT(0), ScalarT(0)},
               {ScalarT(2), ScalarT(2), ScalarT(2)});

  EXPECT_EQ(hasher(seg1), hasher(seg2));
  EXPECT_NE(hasher(seg1), hasher(seg3));
}

TEST(GeometryUtilsTest, IsIntersectFacePoint) {
  // Треугольник в плоскости XY с нормалью (0,0,1)
  Face face({ScalarT(0), ScalarT(0), ScalarT(0)},
            {ScalarT(1), ScalarT(0), ScalarT(0)},
            {ScalarT(0), ScalarT(1), ScalarT(0)});

  // 1. Точка точно в плоскости треугольника и внутри
  Point p1(ScalarT(0.3), ScalarT(0.3), ScalarT(0));
  EXPECT_TRUE(IsIntersect(face, p1));
  EXPECT_TRUE(IsIntersect(p1, face));

  // 2. Точка в плоскости, но вне треугольника
  Point p2(ScalarT(1), ScalarT(1), ScalarT(0));
  EXPECT_FALSE(IsIntersect(face, p2));
  EXPECT_FALSE(IsIntersect(p2, face));

  // 3. Точка над треугольником (в направлении нормали)
  Point p3(ScalarT(0.3), ScalarT(0.3), ScalarT(1));
  // Должно быть false, так как луч от точки против нормали не пересекает
  // треугольник
  EXPECT_FALSE(IsIntersect(face, p3));

  // 4. Точка под треугольником (против нормали)
  Point p4(ScalarT(0.3), ScalarT(0.3), ScalarT(-1));
  // Должно быть true, так как луч от точки по нормали пересекает треугольник
  EXPECT_TRUE(IsIntersect(face, p4));

  // 5. Точка далеко над треугольником, но проекция внутри
  Point p5(ScalarT(0.3), ScalarT(0.3), ScalarT(100));
  EXPECT_FALSE(IsIntersect(face, p5));

  // 6. Точка далеко под треугольником, проекция внутри
  Point p6(ScalarT(0.3), ScalarT(0.3), ScalarT(-100));
  EXPECT_TRUE(IsIntersect(face, p6));
}

TEST(GeometryUtilsTest, CalcDistancePointSegment) {
  Segment seg({ScalarT(0), ScalarT(0), ScalarT(0)},
              {ScalarT(2), ScalarT(0), ScalarT(0)});

  Point p1(ScalarT(1), ScalarT(0), ScalarT(0));
  EXPECT_EQ(CalcDistance(p1, seg), ScalarT(0));
  EXPECT_EQ(CalcDistance(seg, p1), ScalarT(0));

  Point p2(ScalarT(1), ScalarT(1), ScalarT(0));
  EXPECT_EQ(CalcDistance(p2, seg), ScalarT(1));

  Point p3(ScalarT(-1), ScalarT(0), ScalarT(0));
  EXPECT_EQ(CalcDistance(p3, seg), ScalarT(1));

  Point p4(ScalarT(3), ScalarT(0), ScalarT(0));
  EXPECT_EQ(CalcDistance(p4, seg), ScalarT(1));
}

TEST(GeometryUtilsTest, CalcDistancePointFace) {
  Face face({ScalarT(0), ScalarT(0), ScalarT(0)},
            {ScalarT(2), ScalarT(0), ScalarT(0)},
            {ScalarT(0), ScalarT(2), ScalarT(0)});

  Point p1(ScalarT(1), ScalarT(1), ScalarT(3));
  EXPECT_EQ(CalcDistance(face, p1), ScalarT(3));

  Point p2(ScalarT(0.5), ScalarT(0.5), ScalarT(2));
  EXPECT_EQ(CalcDistance(face, p2), ScalarT(2));

  Point p3(ScalarT(3), ScalarT(3), ScalarT(1));
  EXPECT_EQ(CalcDistance(face, p3), ScalarT(3));

  Point p4(ScalarT(1), ScalarT(-1), ScalarT(2));
  EXPECT_EQ(CalcDistance(face, p4), ScalarT(std::sqrt(5)));
}

TEST(GeometryUtilsTest, AngleBetweenVectors) {
  Vector v1(ScalarT(1), ScalarT(0), ScalarT(0));
  Vector v2(ScalarT(0), ScalarT(1), ScalarT(0));

  EXPECT_EQ(AngleBetweenVectors(v1, v2), ScalarT(M_PI / 2));

  EXPECT_EQ(AngleBetweenVectors(v1, v1), ScalarT(0));

  EXPECT_EQ(AngleBetweenVectors(v1, -v1), ScalarT(M_PI));

  Vector v3(ScalarT(1), ScalarT(1), ScalarT(0));
  EXPECT_EQ(AngleBetweenVectors(v1, v3), ScalarT(M_PI / 4));

  Vector zero(ScalarT(0), ScalarT(0), ScalarT(0));
  EXPECT_THROW(std::ignore = AngleBetweenVectors(v1, zero),
               std::invalid_argument);
  EXPECT_THROW(std::ignore = AngleBetweenVectors(zero, v1),
               std::invalid_argument);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
