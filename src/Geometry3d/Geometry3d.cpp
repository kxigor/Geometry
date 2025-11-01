#include "Geometry/Geometry3d.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace geometry3d {
[[nodiscard]] bool IsIntersect(const Face& face, const Point& p) noexcept {
  const auto& P = face.GetP();
  const auto& Q = face.GetQ();
  const auto& R = face.GetR();
  const auto& normal = face.GetNormal();

  const Vector v0 = R - P;
  const Vector v1 = Q - P;
  const Vector v2 = p - P;

  const ScalarT dot = normal.ScalarProduct(v2);
  if (dot > ScalarT(0)) {
    return false;
  }

  const ScalarT d00 = v0.ScalarProduct(v0);
  const ScalarT d01 = v0.ScalarProduct(v1);
  const ScalarT d11 = v1.ScalarProduct(v1);
  const ScalarT d20 = v2.ScalarProduct(v0);
  const ScalarT d21 = v2.ScalarProduct(v1);
  const ScalarT denom = d00 * d11 - d01 * d01;
  const ScalarT v = (d11 * d20 - d01 * d21) / denom;
  const ScalarT w = (d00 * d21 - d01 * d20) / denom;
  const ScalarT u = ScalarT(1) - v - w;
  return (u >= ScalarT(0)) && (v >= ScalarT(0)) && (w >= ScalarT(0));
}

[[nodiscard]] bool IsIntersect(const Point& p, const Face& face) noexcept {
  return IsIntersect(face, p);
}

[[nodiscard]] ScalarT CalcDistance(const Point& p,
                                   const Segment& seg) noexcept {
  using std::clamp;
  const Point& A = seg.GetA();
  const Point& B = seg.GetB();
  const Vector AB = B - A;
  const Vector AP = p - A;

  const ScalarT t = AP.ScalarProduct(AB) / AB.SquaredLength();
  const ScalarT t_clamped = clamp(t, ScalarT(0), ScalarT(1));
  const Point Q = A + AB * t_clamped;

  return (p - Q).Length();
}

[[nodiscard]] ScalarT CalcDistance(const Segment& seg,
                                   const Point& p) noexcept {
  return CalcDistance(p, seg);
}

[[nodiscard]] ScalarT CalcDistance(const Face& face, const Point& p) noexcept {
  using std::min;

  const Vector& n = face.GetNormal();
  const Point& A = face.GetP();

  const Vector AP = p - A;
  const ScalarT t = n.ScalarProduct(AP);
  const Point P_proj = p - n * (t / n.SquaredLength());

  if (IsIntersect(face, P_proj)) {
    return (p - P_proj).Length();
  }

  const std::array edges = {Segment{face.GetP(), face.GetQ()},
                            Segment{face.GetQ(), face.GetR()},
                            Segment{face.GetR(), face.GetP()}};

  ScalarT min_dist = std::numeric_limits<ScalarT>::max();
  for (const auto& edge : edges) {
    min_dist = min(min_dist, CalcDistance(p, edge));
  }

  return min_dist;
}

[[nodiscard]] ScalarT CalcDistance(const Point& p, const Face& face) noexcept {
  return CalcDistance(face, p);
}

[[nodiscard]] ScalarT AngleBetweenVectors(const Vector& a, const Vector& b) {
  using std::clamp, std::acos;

  const ScalarT dot_product = a.ScalarProduct(b);
  const ScalarT a_length = a.Length();
  const ScalarT b_length = b.Length();

  if (a_length == ScalarT(0) || b_length == ScalarT(0)) {
    throw std::invalid_argument("Vectors must not be zero-length.");
  }

  ScalarT cos_theta = dot_product / (a_length * b_length);

  cos_theta = clamp(cos_theta, ScalarT(-1), ScalarT(1));

  return acos(cos_theta);
}

}  // namespace geometry3d
