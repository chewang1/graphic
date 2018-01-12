#pragma once

#include "Math/Math.hpp"
#include <vector>

namespace Helpers
{

struct SimplePolygon
{
  void ComputeCentroid();
  bool operator<(const SimplePolygon& rhs) const;

  std::vector<Vector3> mPoints;
  Vector3 mCentroid;
};
struct SimpleTriangle
{
  Vector3 mPoints[3];
};
void Simplify(std::vector<SimpleTriangle>& input, std::vector<SimplePolygon>& output);

}//namespace Helpers
