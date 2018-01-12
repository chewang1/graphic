///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#include "Precompiled.hpp"


Vector3 ProjectPointOnPlane(const Vector3& point, const Vector3& normal, float planeDistance)
{
	//Make a vector from your orig point to the point of interest
	Math::Vector3 vectoPoint = point - (normal * planeDistance);
	//Take the dot product of that vector with the unit normal vector
	float DstFromPlane = vectoPoint.Dot(normal);

	//point = pop - d * n
	return point - (DstFromPlane * normal);
}

bool BarycentricCoordinates(const Vector3& point, const Vector3& a, const Vector3& b,
  float& u, float& v, float epsilon)
{
  //avoid dividing by zero
  if (a != b)
  {

    Vector3 ab = a - b;
    u = (point - b).Dot(ab) / ab.Dot(ab);
    v = 1 - u;
    if (u == Math::Clamp(u, -epsilon, 1.0f + epsilon))
      if (v == Math::Clamp(v, -epsilon, 1.0f + epsilon))
        return true;
  }
  else
  {
    u = v = 0;
  }
  return false;


}

bool BarycentricCoordinates(const Vector3& point, const Vector3& a, const Vector3& b, const Vector3& c,
                            float& u, float& v, float& w, float epsilon)
{
	if ((a != b) && (b != c) && (c != a))
	{
		//using cramer's rule
		Math::Vector3 ap = point - a;
		Math::Vector3 ac = c - a;
		Math::Vector3 ab = b - a;

		float dbb = ab.Dot(ab); //d01
		float dbc = ab.Dot(ac); //d01
		float dcc = ac.Dot(ac); //d11
		float dpb = ap.Dot(ab); //d20
		float dpc = ap.Dot(ac); //d21

		float denom = dbb * dcc - dbc * dbc; //d00 * d11 - d01 * d01;
		v = (dcc * dpb - dbc * dpc) / denom; //d11 * d20 - d01 * d21
		w = (dbb * dpc - dbc * dpb) / denom; //(d00 * d21 - d01 * d20
		u = 1 - v - w;

		if (u == Math::Clamp(u, -epsilon, 1.0f + epsilon))
			if (v == Math::Clamp(v, -epsilon, 1.0f + epsilon))
				if (w == Math::Clamp(w, -epsilon, 1.0f + epsilon))
					return true;
    }
    return false;
}

IntersectionType::Type PointPlane(const Vector3& point, const Vector4& plane, float epsilon)
{
	//plane normal
	Math::Vector3 normal = Math::Vector3(plane.x, plane.y, plane.z);
	//computing the distance from plane
	Math::Vector3 vectoPoint = point - (normal * plane.w);
	float DstFromPlane = vectoPoint.Dot(normal);

	if (DstFromPlane < -epsilon)
		return IntersectionType::Outside;
	else if (DstFromPlane > epsilon)
		return IntersectionType::Inside;
	else
		return IntersectionType::Coplanar;

}

bool PointSphere(const Vector3& point, const Vector3& sphereCenter, float sphereRadius)
{
	return (sphereCenter - point).LengthSq() <= (sphereRadius * sphereRadius);
}

bool PointAabb(const Vector3& point, const Vector3& aabbMin, const Vector3& aabbMax)
{
	Math::Vector3 isinside = point;
	isinside.x = Math::Clamp(isinside.x, aabbMin.x, aabbMax.x);
	isinside.y = Math::Clamp(isinside.y, aabbMin.y, aabbMax.y);
	isinside.z = Math::Clamp(isinside.z, aabbMin.z, aabbMax.z);

	//see if the point still the point
    return isinside == point ? true : false;
}

bool RayPlane(const Vector3& rayStart, const Vector3& rayDir,
              const Vector4& plane, float& t, float epsilon)
{
  ++Application::mStatistics.mRayPlaneTests;
  //get plane normal and compute ray distance to plane
  //ray: P = P0+ tV
  //plane: P * N +d = 0

  IntersectionType::Type type = PointPlane(rayStart, plane, epsilon);
  float myepsilon = 0.0001f;
  if (type == IntersectionType::Coplanar)
  {
    t = 0;
    return false;
  }
  Vector3 normal(plane.x, plane.y, plane.z);
  float denom = normal.Dot(rayDir);
  if (Math::Abs(denom) <= myepsilon)//perpendicular - meaning the ray is parallel to the plane
  {
    t = Math::PositiveMax();
    return false;
  }
  //to get t
  t = (plane.w - normal.Dot(rayStart)) / denom;
  if (t <= 0)
    return false;
  else
    return true;


}

bool RayTriangle(const Vector3& rayStart, const Vector3& rayDir,
                 const Vector3& triP0, const Vector3& triP1, const Vector3& triP2,
                 float& t, float triExpansionEpsilon)
{
  ++Application::mStatistics.mRayTriangleTests;
  //get triangle normal
  Math::Vector3 normal = (triP1 - triP0).Cross(triP2 - triP0);
  normal.AttemptNormalize();
	bool if_rayplane = RayPlane(rayStart, rayDir, Math::Vector4(normal.x, normal.y, normal.z, normal.Dot(triP0)), t);
  if (if_rayplane)
  {
	  Math::Vector3 pointOnPlane = (rayDir * t) + rayStart;
	  Math::Vector3 barycentric;
	  return BarycentricCoordinates(pointOnPlane, triP0, triP1, triP2, barycentric.x, barycentric.y, barycentric.z, triExpansionEpsilon);
  }
  return false;
}

bool RaySphere(const Vector3& rayStart, const Vector3& rayDir,
               const Vector3& sphereCenter, float sphereRadius,
               float& t)
{
  ++Application::mStatistics.mRaySphereTests;
  Math::Vector3 rTosphere = sphereCenter - rayStart;
	float sq_radius = sphereRadius * sphereRadius;
	float sq_length = rTosphere.LengthSq();
	if(sq_length < sq_radius)
	{
		t = 0;
		return true;
	}

	Vector3 cross = (-rTosphere).Cross(rayDir);
	float lineToCenter = cross.Dot(cross) / rayDir.Dot(rayDir);
	if (rTosphere.Dot(rayDir) < 0)
	 return false;
  if (lineToCenter > sq_radius)
	 return false;
	if(lineToCenter == sq_radius)
	{
		t = sqrt(sq_length - sq_radius);
		return true;
	}

	t = sqrt(sq_length - lineToCenter) - sqrt(sq_radius - lineToCenter);
	return true;
}

bool RayAabb(const Vector3& rayStart, const Vector3& rayDir,
             const Vector3& aabbMin, const Vector3& aabbMax, float& t)
{
  ++Application::mStatistics.mRayAabbTests;
  //compute
  Vector3 tmin, tmax;
  if(rayDir.x == 0 || rayDir.y == 0 || rayDir.z == 0)
  {
    tmin.x = tmin.y = tmin.z = Math::PositiveMin();
    tmax.x = tmax.y = tmax.z = Math::PositiveMax();
  }
  else
  {
    tmin = Vector3((aabbMin.x - rayStart.x) / rayDir.x, (aabbMin.y - rayStart.y) / rayDir.y, (aabbMin.z - rayStart.z) / rayDir.z);
    tmax = Vector3((aabbMax.x - rayStart.x) / rayDir.x, (aabbMax.y - rayStart.y) / rayDir.y, (aabbMax.z - rayStart.z) / rayDir.z);
  }

  if (tmin.x > tmax.x)
    Math::Swap(tmin.x, tmax.x);
  if (tmin.y > tmax.y)
    Math::Swap(tmin.y, tmax.y);
  if (tmin.z > tmax.z)
    Math::Swap(tmin.z, tmax.z);
  float min = tmin.x;
  if (min < tmin.y)
    min = tmin.y;
  if (min < tmin.z)
    min = tmin.z;
  t = min;
  float max = tmax.x;
  if (max > tmax.y)
    max = tmax.y;
  if (max > tmax.z)
    max = tmax.z;
  if (max < 0)
  {
    t = max;
    return false;
  }
  if (t < 0)
    t = 0;
  return min < max;
}

IntersectionType::Type PlaneTriangle(const Vector4& plane, 
                                     const Vector3& triP0, const Vector3& triP1, const Vector3& triP2,
                                     float epsilon)
{
  ++Application::mStatistics.mPlaneTriangleTests;
  //Plane vs triangle is just classifying all 3 points and determining if all the points are on one side or straddling.
  IntersectionType::Type result0 = PointPlane(triP0, plane, epsilon);
  IntersectionType::Type result1 = PointPlane(triP1, plane, epsilon);
  IntersectionType::Type result2 = PointPlane(triP2, plane, epsilon);
  if (result0 == IntersectionType::Coplanar && result1 == IntersectionType::Coplanar && result2 == IntersectionType::Coplanar)
	  return IntersectionType::Coplanar;
  else if ((result0 == IntersectionType::Inside || result0 == IntersectionType::Coplanar) &&
    (result1 == IntersectionType::Inside || result1 == IntersectionType::Coplanar) &&
    (result2 == IntersectionType::Inside || result2 == IntersectionType::Coplanar))
	  return IntersectionType::Inside;
  else if ((result0 == IntersectionType::Outside || result0 == IntersectionType::Coplanar) &&
	  (result1 == IntersectionType::Outside || result1 == IntersectionType::Coplanar) &&
	  (result2 == IntersectionType::Outside || result2 == IntersectionType::Coplanar))
	  return IntersectionType::Outside;
  else
	  return IntersectionType::Overlaps;
 
}

IntersectionType::Type PlaneSphere(const Vector4& plane,
                                   const Vector3& sphereCenter, float sphereRadius)
{
  ++Application::mStatistics.mPlaneSphereTests;
  Math::Vector3 normal = Math::Vector3(plane.x, plane.y, plane.z);
  Math::Vector3 vectoPoint = sphereCenter - ( normal* plane.w);
  //Take the dot product of that vector with the unit normal vector
  float DstFromPlane = vectoPoint.Dot(normal);
  if (DstFromPlane > sphereRadius)
  {
	  return IntersectionType::Inside;
  }
  if (DstFromPlane < -sphereRadius)
  {
	  return IntersectionType::Outside;
  }
  return IntersectionType::Overlaps;
}

Vector3 GetAabbPointInDirection(const Vector3 &aabbMin, const Vector3 &aabbMax, const Vector3& direction)
{
	Math::Vector3 center = (aabbMin + aabbMax) * 0.5f;
	Math::Vector3 halfextent = aabbMax - center;
	Math::Vector3 result;

	for (size_t i = 0; i < 3; i++)
	{
		//check x,y,z of vector
		if (direction[i] >= 0)
			result[i] = center[i] + halfextent[i];
		else
			result[i] = center[i] - halfextent[i];
	}
	return result;
}
IntersectionType::Type PlaneAabb(const Vector4& plane,
                                 const Vector3& aabbMin, const Vector3& aabbMax)
{
  ++Application::mStatistics.mPlaneAabbTests;
  Math::Vector3 normal = Math::Vector3(plane.x, plane.y, plane.z);
  Math::Vector3 result = GetAabbPointInDirection(aabbMin, aabbMax, normal);
  Math::Vector3 neg_result = GetAabbPointInDirection(aabbMin, aabbMax, -normal);
  float myepsilon = 0.0001f;
	IntersectionType::Type t_aabbMax = PointPlane(aabbMax, plane, myepsilon);
	IntersectionType::Type t_aabbMin = PointPlane(aabbMin, plane, myepsilon);
  if (t_aabbMax == IntersectionType::Inside &&
			t_aabbMin == IntersectionType::Inside)
  {
	  if (PointPlane(result, plane, myepsilon) == IntersectionType::Outside ||
		  PointPlane(neg_result, plane, myepsilon) == IntersectionType::Outside)
		  return IntersectionType::Overlaps;
	  else
		  return IntersectionType::Inside;
  }
  else if (t_aabbMin == IntersectionType::Outside &&
					t_aabbMax == IntersectionType::Outside)
  {
	  if (PointPlane(result, plane, myepsilon) == IntersectionType::Inside ||
		  PointPlane(neg_result, plane, myepsilon) == IntersectionType::Inside)
		  return IntersectionType::Overlaps;
	  else
		  return IntersectionType::Outside;
  }	  
  else
	  return IntersectionType::Overlaps;
}

IntersectionType::Type FrustumTriangle(const Vector4 planes[6],
                                       const Vector3& triP0, const Vector3& triP1, const Vector3& triP2,
                                       float epsilon)
{
  ++Application::mStatistics.mFrustumTriangleTests;
   std::vector<IntersectionType::Type> result;
  for (int i = 0; i < 6; i++)
	  result.push_back(PlaneTriangle(planes[i], triP0, triP1, triP2, epsilon));

  //as long as one of planes is outside, then it is outside
  auto outside = std::find(result.begin(), result.end(), IntersectionType::Outside);
  if (outside != result.end())
	  return IntersectionType::Outside;
  //and then check for overlaps
  auto overlap = std::find(result.begin(), result.end(), IntersectionType::Overlaps);
  if (overlap != result.end())
	  return IntersectionType::Overlaps;
  return IntersectionType::Inside;

}

IntersectionType::Type FrustumSphere(const Vector4 planes[6],
                                     const Vector3& sphereCenter, float sphereRadius, size_t& lastAxis)
{
  ++Application::mStatistics.mFrustumSphereTests;
  std::vector<IntersectionType::Type> result;
  for (int i = 0; i < 6; i++)
	  result.push_back(PlaneSphere(planes[i], sphereCenter, sphereRadius));

  auto outside = std::find(result.begin(), result.end(), IntersectionType::Outside);
  if (outside != result.end())
	  return IntersectionType::Outside;
  //and then check for overlaps
  auto overlap = std::find(result.begin(), result.end(), IntersectionType::Overlaps);
  if (overlap != result.end())
	  return IntersectionType::Overlaps;
  return IntersectionType::Inside;
}

IntersectionType::Type FrustumAabb(const Vector4 planes[6],
                                   const Vector3& aabbMin, const Vector3& aabbMax, size_t& lastAxis)
{
	/*Same as sphere, test all 6 planes:
		If outside any return outside
		If inside all return inside
		Otherwise return overlaps
	*/
  ++Application::mStatistics.mFrustumAabbTests;
  std::vector<IntersectionType::Type> result;
  for (int i = 0; i < 6; i++)
	  result.push_back(PlaneAabb(planes[i], aabbMin, aabbMax));

  auto outside = std::find(result.begin(), result.end(), IntersectionType::Outside);
  if (outside != result.end())
	  return IntersectionType::Outside;
  //and then check for overlaps
  auto overlap = std::find(result.begin(), result.end(), IntersectionType::Overlaps);
  if (overlap != result.end())
	  return IntersectionType::Overlaps;
  return IntersectionType::Inside;
  //int inside_plane_num = 0, total_plane = 6;
  //for (int i = 0; i < total_plane; i++)
  //{ 
  //  int check_planeID = (lastAxis + i) % 6; //stay in the plane axis which we have last axis
  //  IntersectionType::Type t = PlaneAabb(planes[check_planeID], aabbMin, aabbMax);
  //  if (t == IntersectionType::Outside)
  //  {
  //    lastAxis = i;
  //    return t;
  //  }
  //  else if (t == IntersectionType::Inside)
  //  {
  //    ++inside_plane_num;
  //  }
  //}
  //if (inside_plane_num == total_plane)
  //{
  //  return IntersectionType::Inside;
  //}
  //return IntersectionType::Overlaps;
}

bool SphereSphere(const Vector3& sphereCenter0, float sphereRadius0,
                  const Vector3& sphereCenter1, float sphereRadius1)
{
  ++Application::mStatistics.mSphereSphereTests;
  float vecLength = (sphereCenter0 - sphereCenter1).LengthSq();
  float radius_sqr = (sphereRadius0 + sphereRadius1) * (sphereRadius0 + sphereRadius1);
  return (vecLength > radius_sqr) ? false : true;
}

bool AabbAabb(const Vector3& aabbMin0, const Vector3& aabbMax0,
              const Vector3& aabbMin1, const Vector3& aabbMax1)
{
  ++Application::mStatistics.mAabbAabbTests;
  for (int i = 0; i < 3; i++)
  {
	  if (aabbMin1[i] > aabbMax0[i] || aabbMin0[i] > aabbMax1[i])
		  return false;
  }	 
  return true;
}
