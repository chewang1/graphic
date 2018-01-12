///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////

#include "Math/Math.hpp"
#include <vector>

namespace Helpers
{

Vector3 ComputeCentroid(const std::vector<Vector3>& points, const Matrix4& transform);
Vector3 SupportPoints(const std::vector<Vector3>& points, const Matrix4& transform, const Vector3& direction);
void DebugDrawPoints(const std::vector<Vector3>& points, const Matrix4& transform, const Vector4& color);
Vector3 SupportSphere(const Vector3& sphereCenter, float sphereRadius, const Vector3& direction);
Vector3 SupportObb(const Vector3& obbScale, const Matrix3& obbRotation, const Vector3& obbTranslation, const Vector3& direction);

}
