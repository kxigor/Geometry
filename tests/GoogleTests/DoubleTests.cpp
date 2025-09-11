#include <gtest/gtest.h>

#include <limits>
#include <type_traits>

#include "Double/Double.hpp"

TEST(DoubleTest, ConstructorsAndAssignments) {
  Double<double> d1{};
  EXPECT_EQ(d1.AsBasicType(), 0.0);

  Double<double> d2(3.14);
  EXPECT_EQ(d2.AsBasicType(), 3.14);

  Double<double> d3(42);
  EXPECT_EQ(d3.AsBasicType(), 42.0);

  Double<double> d4(d2);
  EXPECT_EQ(d4.AsBasicType(), 3.14);

  Double<double> d5(std::move(d2));
  EXPECT_EQ(d5.AsBasicType(), 3.14);

  d1 = 2.71;
  EXPECT_EQ(d1.AsBasicType(), 2.71);

  d1 = d4;
  EXPECT_EQ(d1.AsBasicType(), 3.14);

  d1 = std::move(d5);
  EXPECT_EQ(d1.AsBasicType(), 3.14);
}

TEST(DoubleTest, ArithmeticOperations) {
  Double<double> d1(2.5);
  Double<double> d2(1.5);

  EXPECT_EQ((d1 + d2).AsBasicType(), 4.0);
  d1 += d2;
  EXPECT_EQ(d1.AsBasicType(), 4.0);

  EXPECT_EQ((d1 - d2).AsBasicType(), 2.5);
  d1 -= d2;
  EXPECT_EQ(d1.AsBasicType(), 2.5);

  EXPECT_EQ((d1 * d2).AsBasicType(), 3.75);
  d1 *= d2;
  EXPECT_EQ(d1.AsBasicType(), 3.75);

  EXPECT_EQ((d1 / d2).AsBasicType(), 2.5);
  d1 /= d2;
  EXPECT_EQ(d1.AsBasicType(), 2.5);

  EXPECT_EQ((-d1).AsBasicType(), -2.5);
  EXPECT_EQ((+d1).AsBasicType(), 2.5);
}

TEST(DoubleTest, Comparisons) {
  Double<double> d1(1.0);
  Double<double> d2(1.0 + std::numeric_limits<double>::epsilon() / 2);
  Double<double> d3(2.0);

  EXPECT_TRUE(d1 == d2);
  EXPECT_FALSE(d1 == d3);

  EXPECT_TRUE(d1 < d3);
  EXPECT_TRUE(d3 > d1);
  EXPECT_TRUE(d1 <= d2);
  EXPECT_TRUE(d1 >= d2);
  EXPECT_TRUE(d1 <= d3);
  EXPECT_TRUE(d3 >= d1);
}

TEST(DoubleTest, IOOperations) {
  Double<double> d1, d2;

  std::istringstream input("3.14 2.71");
  input >> d1 >> d2;
  EXPECT_EQ(d1.AsBasicType(), 3.14);
  EXPECT_EQ(d2.AsBasicType(), 2.71);

  std::ostringstream output;
  output << d1 << " " << d2;
  EXPECT_EQ(output.str(), "3.14 2.71");
}

TEST(DoubleTest, Conversions) {
  Double<double> d1(3.14);

  EXPECT_EQ(d1.AsBasicType(), 3.14);

  double d = d1;
  EXPECT_EQ(d, 3.14);
}

TEST(DoubleTest, NumericLimits) {
  using D = Double<double>;

  EXPECT_TRUE(std::numeric_limits<D>::is_specialized);
  EXPECT_FALSE(std::numeric_limits<D>::is_exact);
  EXPECT_EQ(std::numeric_limits<D>::epsilon().AsBasicType(),
            std::numeric_limits<double>::epsilon());

  // Check min/max values
  EXPECT_EQ(std::numeric_limits<D>::min().AsBasicType(),
            std::numeric_limits<double>::min());
  EXPECT_EQ(std::numeric_limits<D>::max().AsBasicType(),
            std::numeric_limits<double>::max());
}

TEST(DoubleTest, Hash) {
  Double<double> d1(3.14);
  Double<double> d2(3.14);
  Double<double> d3(2.71);

  std::hash<Double<double>> hasher;
  EXPECT_EQ(hasher(d1), hasher(d2));
  EXPECT_NE(hasher(d1), hasher(d3));
}

TEST(DoubleTest, CustomEpsilon) {
  using CustomDouble = Double<double, 0.001>;
  CustomDouble d1(1.0);
  CustomDouble d2(1.0005);
  CustomDouble d3(1.002);

  EXPECT_TRUE(d1 == d2);
  EXPECT_FALSE(d1 == d3);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}