///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#include "Precompiled.hpp"

//-----------------------------------------------------------------------------SupportShape
Vector3 SupportShape::GetCenter(const std::vector<Vector3>& localPoints, const Matrix4& transform) const
{
  //this is from assignment2 shapes.cpp Sphere::computercentroid
  Vector3 center = Vector3::cZero;
  Vector3 aabbMax = localPoints.front();
  Vector3 aabbMin = localPoints.front();
  std::vector<Vector3>::const_iterator iter;
  for (iter = localPoints.begin(); iter != localPoints.end(); ++iter)
  {
    if (iter->x > aabbMax.x)
      aabbMax.x = iter->x;
    else if (iter->x < aabbMin.x)
      aabbMin.x = iter->x;

    if (iter->y > aabbMax.y)
      aabbMax.y = iter->y;
    else if (iter->y < aabbMin.y)
      aabbMin.y = iter->y;

    if (iter->z > aabbMax.z)
      aabbMax.z = iter->z;
    else if (iter->z < aabbMin.z)
      aabbMin.z = iter->z;
  }
  center = Math::TransformPoint(transform, 0.5f * (aabbMin + aabbMax));
  return center;

}

Vector3 SupportShape::Support(const Vector3& worldDirection, const std::vector<Vector3>& localPoints, const Matrix4& localToWorldTransform) const
{
	//find the furest point along the direction vec
	//convert worlddireciton to local
	Vector4 localdir = Math::Transform(localToWorldTransform.Inverted(), Vector4(worldDirection.x, worldDirection.y, worldDirection.z, 0));
	Vector3 dir(localdir.x, localdir.y, localdir.z);
	Vector3 result = Vector3::cZero;
	float max = -Math::PositiveMax();
	int index = 0;
	for (size_t i = 0; i < localPoints.size(); i++)
	{
		float dot = localPoints[i].Dot(dir);
		if(dot > max)
		{
			max = dot;
			index = i;
		}
	}
	//transform the resultant point into world space
	result = Math::TransformPoint(localToWorldTransform, localPoints[index]);
  return result;
}

void SupportShape::DebugDraw(const std::vector<Vector3>& localPoints, const Matrix4& localToWorldTransform, const Vector4& color) const
{
  /******Student:Assignment5******/
  for (size_t i =0; i < localPoints.size(); i++)
  {
    Vector3 transformed_pts = Math::TransformPoint(localToWorldTransform, localPoints[i]);
    DebugShape& shape = gDebugDrawer->DrawPoint(transformed_pts);
    shape.Color(color);
  }
}

//-----------------------------------------------------------------------------ModelSupportShape
Vector3 ModelSupportShape::GetCenter() const
{
  return SupportShape::GetCenter(mModel->mMesh->mVertices, mModel->mOwner->has(Transform)->GetTransform());
}

Vector3 ModelSupportShape::Support(const Vector3& worldDirection) const
{
  return SupportShape::Support(worldDirection, mModel->mMesh->mVertices, mModel->mOwner->has(Transform)->GetTransform());
}

void ModelSupportShape::DebugDraw(const Vector4& color) const
{
  SupportShape::DebugDraw(mModel->mMesh->mVertices, mModel->mOwner->has(Transform)->GetTransform());
}

//-----------------------------------------------------------------------------PointsSupportShape
PointsSupportShape::PointsSupportShape()
{
  mScale = Vector3(1);
  mRotation = Matrix3::cIdentity;
  mTranslation = Vector3::cZero;
}

Vector3 PointsSupportShape::GetCenter() const
{
  Matrix4 transform = Math::BuildTransform(mTranslation, mRotation, mScale);
  return SupportShape::GetCenter(mLocalSpacePoints, transform);
}

Vector3 PointsSupportShape::Support(const Vector3& worldDirection) const
{
  Matrix4 transform = Math::BuildTransform(mTranslation, mRotation, mScale);
  return SupportShape::Support(worldDirection, mLocalSpacePoints, transform);
}

void PointsSupportShape::DebugDraw(const Vector4& color) const
{
  Matrix4 transform = Math::BuildTransform(mTranslation, mRotation, mScale);
  SupportShape::DebugDraw(mLocalSpacePoints, transform, color);
}

//-----------------------------------------------------------------------------SphereSupportShape
Vector3 SphereSupportShape::GetCenter() const
{
  return mSphere.mCenter;
}

Vector3 SphereSupportShape::Support(const Vector3& worldDirection) const
{
	//find the point on the sphere furthest in the direction passed in
  return mSphere.mCenter + worldDirection.Normalized() * mSphere.mRadius;
}

void SphereSupportShape::DebugDraw(const Vector4& color) const
{
  DebugShape& shape = gDebugDrawer->DrawSphere(mSphere);
  shape.Color(color);
}

//-----------------------------------------------------------------------------ObbSupportShape
Vector3 ObbSupportShape::GetCenter() const
{
  return mTranslation;
}

Vector3 ObbSupportShape::Support(const Vector3& worldDirection) const
{
  /******Student:Assignment5******/
  // Note: A unit obb spans from [-0.5, to 0.5]. Make sure to properly account for this.
	Vector3 result = mTranslation;
	Vector3 localDir = Math::Transform(mRotation.Inverted(), worldDirection);
	for (size_t i = 0; i < 3; i++)
    result += Math::GetSign(localDir[i]) * (mScale[i] *0.5f) * mRotation.Basis(i);

  return result;
}

void ObbSupportShape::DebugDraw(const Vector4& color) const
{
  Matrix4 transform = Math::BuildTransform(mTranslation, mRotation, mScale);
  DebugShape& shape = gDebugDrawer->DrawAabb(Aabb(Vector3(-0.5f), Vector3(0.5f)));
  shape.Color(color);
  shape.SetTransform(transform);
}

//------------------------------------------------------------ Voronoi Region Tests
VoronoiRegion::Type Gjk::IdentifyVoronoiRegion(const Vector3& q, const Vector3& p0,
  size_t& newSize, int newIndices[4],
  Vector3& closestPoint, Vector3& searchDirection)
{
	closestPoint = p0;
  newSize = 1;
  newIndices[0] = 0;
  searchDirection = q - p0;
  return VoronoiRegion::Point0;
}

VoronoiRegion::Type Gjk::IdentifyVoronoiRegion(const Vector3& q, const Vector3& p0, const Vector3& p1,
  size_t& newSize, int newIndices[4],
  Vector3& closestPoint, Vector3& searchDirection)
{
	float u, v;
	BarycentricCoordinates(q, p0, p1, u, v);
	if (v <= 0)   //p---s0----s1
	{
		closestPoint = p0;
    newSize = 1;
    newIndices[0] = 0;
    searchDirection = q - closestPoint;
		return VoronoiRegion::Point0;
	}	
	else if (u <= 0)  //s0----s1---p
	{
		closestPoint = p1;
    newSize = 1;
    newIndices[0] = 1;
    searchDirection = q - closestPoint;;
		return VoronoiRegion::Point1;
	}
	//s0----p----s1
	closestPoint =  u * p0 + v* p1;
  newSize = 2;
  newIndices[0] = 0;
  newIndices[1] = 1;
  searchDirection = q - closestPoint;
  return VoronoiRegion::Edge01;
}

VoronoiRegion::Type Gjk::IdentifyVoronoiRegion(const Vector3& q, const Vector3& p0, const Vector3& p1, const Vector3& p2,
  size_t& newSize, int newIndices[4],
  Vector3& closestPoint, Vector3& searchDirection)
{
  float epsilon = 0.0001f;
  float u_p0p1, v_p0p1, u_p1p2, v_p1p2, u_p0p2, v_p0p2;
  BarycentricCoordinates(q, p0, p1, u_p0p1, v_p0p1);
  BarycentricCoordinates(q, p1, p2, u_p1p2, v_p1p2);
  BarycentricCoordinates(q, p2, p0, u_p0p2, v_p0p2);
  float u, v, w;
  bool triangle = BarycentricCoordinates(q, p0, p1, p2, u, v, w);
  // point case
  if(v_p0p1 <= 0 && u_p0p2 <= 0)
  {
    SetInfo(q, closestPoint, searchDirection, newSize, p0);
    newIndices[0] = 0;
    return VoronoiRegion::Point0;
  }
  else if(v_p0p2 <= 0 && u_p1p2 <= 0)
  {
    SetInfo(q, closestPoint, searchDirection, newSize, p2);
    newIndices[0] = 2;
    return VoronoiRegion::Point2;
  }
  else if(v_p1p2 <= 0 && u_p0p1 <= 0)
  {
    SetInfo(q, closestPoint, searchDirection, newSize, p1);
    newIndices[0] = 1;
    return VoronoiRegion::Point1;
  }
  else if(triangle) //inside triangle
  {
    float triangleArea = (p2 - p0).Cross(p1 - p0).Length() / 2;
    if (w - u > epsilon && w - v > epsilon ||
		  	u - v > epsilon && u - w > epsilon ||
		  	v - w > epsilon && v - u > epsilon) //appproach p1, p2, p3
      closestPoint = (u *p0 + v * p1 + w * p2);
    else //centroid
      closestPoint = (p0 + p1 + p2) / 3;
    newSize = 3;
    for (size_t i = 0; i < newSize; i++)
      newIndices[i] = i;
    searchDirection = q - closestPoint;

    return VoronoiRegion::Triangle012;
  }
  else //edge case
  {
    if(w < 0 && w < u && w < v)
    {
      SetInfo(q, closestPoint, searchDirection, newSize, p0, p1, u_p0p1, v_p0p1);
      newIndices[0] = 0;
      newIndices[1] = 1;
      return VoronoiRegion::Edge01;
    }
    if(u < 0 && u < w && u < v)
    {
      SetInfo(q, closestPoint, searchDirection, newSize, p1, p2, u_p1p2, v_p1p2);
      newIndices[0] = 1;
      newIndices[1] = 2;
      return VoronoiRegion::Edge12;
    }
    if (v < 0 && v < w && v < u)
    {
      SetInfo(q, closestPoint, searchDirection, newSize, p2, p0, u_p0p2, v_p0p2);
      newIndices[0] = 0;
      newIndices[1] = 2;
      return VoronoiRegion::Edge02;
    }
  }
  return VoronoiRegion::Unknown;
}

VoronoiRegion::Type Gjk::IdentifyVoronoiRegion(const Vector3& q, const Vector3& p0, const Vector3& p1, const Vector3& p2, const Vector3& p3,
  size_t& newSize, int newIndices[4],
  Vector3& closestPoint, Vector3& searchDirection)
{
  //line
  float u_p0p1, v_p0p1, u_p3p0, v_p3p0, u_p1p3, v_p1p3;
  bool result01 = BarycentricCoordinates(q, p0, p1, u_p0p1, v_p0p1); //line 01
  bool result13 = BarycentricCoordinates(q, p1, p3, u_p1p3, v_p1p3); //line 13
  bool result03 = BarycentricCoordinates(q, p3, p0, u_p3p0, v_p3p0); //line 03
  //triangle 123
  float u_p1p2, v_p1p2, u_p2p3, v_p2p3;
  bool result12 = BarycentricCoordinates(q, p1, p2, u_p1p2, v_p1p2); //line 12
  bool result23 = BarycentricCoordinates(q, p2, p3, u_p2p3, v_p2p3); //line 23
  float u_p0p2, v_p0p2;
  bool result02 = BarycentricCoordinates(q, p0, p2, u_p0p2, v_p0p2); //line 02
  //triangle 013 and triangle 012
  float u_013, v_013, w_013;
  bool triangle013 = BarycentricCoordinates(q, p0, p1, p3, u_013, v_013, w_013);
  float u_012, v_012, w_012;
  bool triangle012 = BarycentricCoordinates(q, p0, p1, p2, u_012, v_012, w_012);
  float u_023, v_023, w_023; //triangle023
  bool triangle023 = BarycentricCoordinates(q, p0, p2, p3, u_023, v_023, w_023);
  float u_123, v_123, w_123; //triangle123
  bool triangle123 = BarycentricCoordinates(q, p1, p2, p3, u_123, v_123, w_123);


  /////////////////////////////
  //region S0
  if(v_p0p1 < 0 && v_p0p2 < 0 && u_p3p0 < 0) 
  {
    SetInfo(q, closestPoint, searchDirection, newSize, p0);
    newIndices[0] = 0;
    return VoronoiRegion::Point0;
  } //region S2
  else if(u_p0p2 < 0 && u_p1p2  < 0&& v_p2p3 < 0)
  {
    SetInfo(q, closestPoint, searchDirection, newSize, p2);
    newIndices[0] = 2;
    return VoronoiRegion::Point2;
  }// region S1
  else if(u_p0p1 < 0 && v_p1p2 < 0 && v_p1p3 < 0)
  {
    SetInfo(q, closestPoint, searchDirection, newSize, p1);
    newIndices[0] = 1;
    return VoronoiRegion::Point1;
  }
  else if(v_p3p0 < 0 && u_p2p3 < 0 && u_p1p3 < 0) // region S3
  {
    SetInfo(q, closestPoint, searchDirection, newSize, p3);
    newIndices[0] = 3;
    return VoronoiRegion::Point3;
  }

  //////////////////////////////////////////////////////////////////////////////////////////
  
  //edge regions

  if (u_p0p1 > 0 && v_p0p1 > 0 && !triangle012 && !triangle013 && w_012 < 0 && w_013 < 0) //edge 01
  {
    SetInfo(q, closestPoint, searchDirection, newSize, p0, p1, u_p0p1, v_p0p1);
    newIndices[0] = 0;
    newIndices[1] = 1;
    return VoronoiRegion::Edge01;
  }
  else if (u_p0p2 > 0 && v_p0p2 > 0 && !triangle012 && !triangle023 && v_012 < 0 && w_023 < 0) //edge 02
  {
    SetInfo(q, closestPoint, searchDirection, newSize, p0, p2, u_p0p2, v_p0p2);
    newIndices[0] = 0;
    newIndices[1] = 2;
    return VoronoiRegion::Edge02;
  }
  else if (u_p3p0 > 0 && v_p3p0 > 0 && !triangle013 && !triangle023 && v_013 < 0 && v_023 < 0) //edge 03
  {
    SetInfo(q, closestPoint, searchDirection, newSize, p3, p0, u_p3p0, v_p3p0);
    newIndices[0] = 0;
    newIndices[1] = 3;
    return VoronoiRegion::Edge03;
  }
  else if (u_p1p2 > 0 && v_p1p2 > 0 && !triangle012 && !triangle123 && u_012 < 0 && w_123 < 0) //edge 12
  {
    SetInfo(q, closestPoint, searchDirection, newSize, p1, p2, u_p1p2, v_p1p2);
    newIndices[0] = 1;
    newIndices[1] = 2;
    return VoronoiRegion::Edge12;
  }
  else if (u_p1p3 > 0 && v_p1p3 > 0 && !triangle013 && !triangle123 && v_123 < 0 && u_013 < 0) //edge 13
  {
    SetInfo(q, closestPoint, searchDirection, newSize, p1, p3, u_p1p3, v_p1p3);
    newIndices[0] = 1;
    newIndices[1] = 3;
    return VoronoiRegion::Edge13;
  }
  else if (u_p2p3 > 0 && v_p2p3 > 0 && !triangle023 && !triangle123 && u_023 < 0 && u_123 < 0) //edge 23
  {
    SetInfo(q, closestPoint, searchDirection, newSize, p2, p3, u_p2p3, v_p2p3);
    newIndices[0] = 2;
    newIndices[1] = 3;
    return VoronoiRegion::Edge23;
  }
  /////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////
  //triangle regions

  Vector3 normal012 = TriNoraml(p0, p1, p2, p3);
  Vector3 normal013 = TriNoraml(p0, p1, p3, p2);
  Vector3 normal023 = TriNoraml(p0, p2, p3, p1);
  Vector3 normal123 = TriNoraml(p1, p3, p2, p0);
  if (triangle012 && normal012.Dot(q - p0) >= 0)
  {
    closestPoint = u_012 * p0 + v_012 * p1 + w_012 * p2;
    newSize = 3;
    newIndices[0] = 0;
    newIndices[1] = 1;
    newIndices[2] = 2;
    searchDirection = q - closestPoint;
    return VoronoiRegion::Triangle012;
  }
  else if (triangle023 && normal023.Dot(q - p0) >= 0)
  {
    closestPoint = u_023 * p0 + v_023 * p2 + w_023 * p3;
    newSize = 3;
    newIndices[0] = 0;
    newIndices[1] = 2;
    newIndices[2] = 3;
    searchDirection = q - closestPoint;
    return VoronoiRegion::Triangle023;
  }
  else if (triangle013 && normal013.Dot(q - p1) >= 0)
  {
    closestPoint = u_013 * p0 + v_013 * p1 + w_013 * p3;
    newSize = 3;
    newIndices[0] = 0;
    newIndices[1] = 1;
    newIndices[2] = 3;
    searchDirection = q - closestPoint;
    return VoronoiRegion::Triangle013;
  }
  else if (triangle123 && normal123.Dot(q - p1) >= 0)
  {
    closestPoint = u_123 * p1 + v_123 * p2 + w_123 * p3;
    newSize = 3;
    newIndices[0] = 1;
    newIndices[1] = 2;
    newIndices[2] = 3;
    searchDirection = q - closestPoint;
    return VoronoiRegion::Triangle123;
  }
  ///////////////////////////////////////////////////////////////////////////////////////////
  //test this last //Tetrahedra case
	//if (triangle012 && triangle013 && triangle023 && triangle123) 
		//if(u_012 > 0 && v_012 > 0 && w_012 > 0 && u_013 > 0 && v_013 > 0 && w_013 > 0 &&
		//u_023 > 0 && v_023 > 0 && w_023 > 0 && u_123 > 0 && v_123 > 0 && w_123 > 0)
	{
    closestPoint = q;
		newSize = 4;
		newIndices[0] = 0;
		newIndices[1] = 1;
		newIndices[2] = 2;
		newIndices[3] = 3;
    searchDirection = q - closestPoint;
		return VoronoiRegion::Tetrahedra0123;
	}
  
  //return VoronoiRegion::Unknown;
}


Vector3 Gjk::TriNoraml(const Vector3& p0, const Vector3& p1, const Vector3& p2, const Vector3& p3)
{
  Vector3 normal = Math::Cross(p1 - p0, p2 - p1);
  if (Math::Dot(normal, p3 - p0) >= 0)
    normal *= -1;
  return normal;
}
void Gjk::SetInfo(const Vector3& q, Vector3& closestPoint, Vector3& searchDirection, size_t& newSize, const Vector3& point)
{
  closestPoint = point;
  newSize = 1;
  searchDirection = q - closestPoint;
}
void Gjk::SetInfo(const Vector3& q, Vector3& closestPoint, Vector3& searchDirection, size_t& newSize, const Vector3& point1, const Vector3& point2, float u, float v)
{
  closestPoint = u * point1 + v * point2;
  newSize = 2;
  searchDirection = q - closestPoint;
}
Gjk::Gjk()
{
}

bool Gjk::Intersect(const SupportShape* shapeA, const SupportShape* shapeB, unsigned int maxIterations, CsoPoint& closestPoint, float epsilon, int debuggingIndex, bool debugDraw)
{
	//unfinished..
	//Initialize the simplex (to one point for us) by searching in a random direction (difference of centers)
	Vector3 centerA = shapeA->GetCenter();
	Vector3 centerB = shapeB->GetCenter();
	Vector3 searchdir = centerB - centerA;
	if (searchdir == Vector3::cZero)
		searchdir = Vector3(-1.0f, 0.0f, 0.0f);
	VoronoiRegion::Type region;
	int newIndices[4];
	size_t newSize  = 0;
	std::vector<Vector3> simplex;
	closestPoint = ComputeSupport(shapeA, shapeB, searchdir);
	//searchdir = -searchdir; //negate direction
	simplex.push_back(closestPoint.mCsoPoint);
  
  if (simplex[0].Dot(searchdir) < 0)
  {
    return false;
  }
  //searchdir *= -1;
	//simplex.push_back(centerA);
	//Compute P by projecting Q onto the new simplex
	//If P is equal to Q then terminate
	//Compute the new search direction (Q-P ) and search for a new point
	//If the new point is no further than P in the search direction then terminate. The length of the vector (Q-P)is the separation distance
	//Add the new point to the simplex and go to 2.
	unsigned int i = maxIterations;
	while(i != 0)
	{	   
    closestPoint = ComputeSupport(shapeA, shapeB, -searchdir);
    if (searchdir == Vector3::cZero)
      searchdir = Vector3(-1.0f, 0.0f, 0.0f);
		if(simplex.back().Dot(searchdir) < 0)
		{
			return true;
		}
    simplex.push_back(closestPoint.mCsoPoint);
		//2.Determine which voronoi region Q is in and reduce to the smallest simplex
		/*if(simplex.size() == 2)
		{
			region = IdentifyVoronoiRegion(simplex[0], simplex[1], newSize, newIndices, closestPoint.mCsoPoint, searchdir);
			if (region == VoronoiRegion::Point0 || region == VoronoiRegion::Point1)
				return true;
		}*/
	  if(simplex.size() == 3) //triangle case
		{
			region = IdentifyVoronoiRegion(simplex[0], simplex[1], simplex[2], newSize, newIndices, closestPoint.mCsoPoint, searchdir);
			if (region == VoronoiRegion::Point0 || region == VoronoiRegion::Point1 || region == VoronoiRegion::Point2)
				return false;
			else if (region == VoronoiRegion::Edge01)
			{
				simplex.pop_back();
			}
			else if (region == VoronoiRegion::Edge02)
			{
				simplex.erase(simplex.begin() + 1);
			}
			else if (region == VoronoiRegion::Edge12)
			{
				simplex.erase(simplex.begin());
			}
			else
				return true;
		}
		else if(simplex.size() == 4) //tetrahedron case
		{
			region = IdentifyVoronoiRegion(simplex[0], simplex[1], simplex[2], simplex[3], newSize, newIndices, closestPoint.mCsoPoint, searchdir);
		}
		i--;
	}
  return false;
}

Gjk::CsoPoint Gjk::ComputeSupport(const SupportShape* shapeA, const SupportShape* shapeB, const Vector3& direction)
{
  /******Student:Assignment5******/
  CsoPoint result;
	//support(A-B, d) = Support(A, d) - Support(B, -d)
	result.mPointA = shapeA->Support(direction);
	result.mPointB = shapeB->Support(-direction);
	result.mCsoPoint = result.mPointA - result.mPointB;
  return result;
}

