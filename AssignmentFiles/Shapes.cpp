///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#include "Precompiled.hpp"

//-----------------------------------------------------------------------------LineSegment
LineSegment::LineSegment()
{
  mStart = mEnd = Vector3::cZero;
}

LineSegment::LineSegment(Math::Vec3Param start, Math::Vec3Param end)
{
  mStart = start;
  mEnd = end;
}

DebugShape& LineSegment::DebugDraw() const
{
  return gDebugDrawer->DrawLine(*this);
}

//-----------------------------------------------------------------------------Ray
Ray::Ray()
{
  mStart = mDirection = Vector3::cZero;
}

Ray::Ray(Math::Vec3Param start, Math::Vec3Param dir)
{
  mStart = start;
  mDirection = dir;
}

Ray Ray::Transform(const Math::Matrix4& transform) const
{
  Ray transformedRay;
  transformedRay.mStart = Math::TransformPoint(transform, mStart);
  transformedRay.mDirection = Math::TransformNormal(transform, mDirection);
  return transformedRay;
}

Vector3 Ray::GetPoint(float t) const
{
  return mStart + mDirection * t;
}

DebugShape& Ray::DebugDraw(float t) const
{
  return gDebugDrawer->DrawRay(*this, t);
}

void Mean(const std::vector<Vector3>& points, float &meanx, float &meany, float &meanz)
{
  meanx = meany = meanz = 0.0f;
  if (points.size() == 0)
    return;
  for (size_t i = 0; i < points.size(); i++)
  {
    meanx += points[i].x;
    meany += points[i].y;
    meanz += points[i].z;
  } 
  meanx /= points.size();
  meany /= points.size();
  meanz /= points.size();
}
//-----------------------------------------------------------------------------PCA Helpers
Matrix3 ComputeCovarianceMatrix(const std::vector<Vector3>& points)
{
  float meanx = 0.0f , meany = 0.0f, meanz = 0.0f;
  Mean(points, meanx, meany, meanz);
  Matrix3 conv(0,0,0,0,0,0,0,0,0);
  for (size_t i = 0; i < points.size(); i++)
  {
    float v0 = points[i].x - meanx;
    float v1 = points[i].y - meany;
    float v2 = points[i].z - meanz;
    conv.m00 += v0 * v0;
    conv.m01 += v0 * v1;
    conv.m02 += v0 * v2;
    conv.m10 += v0 * v1;
    conv.m11 += v1 * v1;
    conv.m12 += v1 * v2;
    conv.m20 += v0 * v2;
    conv.m21 += v1 * v2;
    conv.m22 += v2 * v2;
  }
  
  conv /= (float)points.size();
  return conv;
}

void ComputeBeta(float app, float aqq, float apq, float &cos, float &sin)
{
  float tangent;
  float beta = (aqq - app) / (2 * apq);
  if (beta < 0)
    tangent = -1 / (Math::Abs(beta) + sqrt(beta * beta + 1));
  else
    tangent = 1 / (Math::Abs(beta) + sqrt(beta * beta + 1));

  cos = 1 / sqrt(tangent * tangent + 1);
  sin = cos * tangent;
}
Matrix3 ComputeJacobiRotation(const Matrix3& matrix)
{
  // Compute the jacobi rotation matrix that will turn the largest (magnitude) off-diagonal element of the input
  // matrix into zero. Note: the input matrix should always be (near) symmetric.
  Matrix3 mat = matrix;  
  //find the largest off diagonal element in matrix3(symmetry) for determine which matrix we want to rotate
  float m01 = Math::Abs(mat[0][1]);
  float m02 = Math::Abs(mat[0][2]);
  float m12 = Math::Abs(mat[1][2]);
  float c, s;
  if(m01 >= m02 && m01 >= m12)
  {
    //m01 is largest
    float app = mat[0][0], aqq = mat[1][1], apq = mat[0][1];
    ComputeBeta(app, aqq, apq, c, s);
    mat = Matrix3(c, s, 0, -s, c, 0, 0, 0, 1);
  }else if(m02 >= m01 && m02 >= m12)
  {
    //m02 is largest
    float app = mat[0][0], aqq = mat[2][2], apq = mat[0][2];
    ComputeBeta(app, aqq, apq, c, s);
    mat = Matrix3(c, 0, s, 0, 1, 0, -s, 0, c);
  }
  else if (m12 >= m01 && m12 >= m02)
  {
    //m02 is largest
    float app = mat[1][1], aqq = mat[2][2], apq = mat[1][2];
    ComputeBeta(app, aqq, apq, c, s);
    mat = Matrix3(1, 0, 0, 0, c, s, 0, -s, c);
  }
  return mat;
}

void ComputeEigenValuesAndVectors(const Matrix3& covariance, Vector3& eigenValues, Matrix3& eigenVectors, int maxIterations)
{
  // Iteratively rotate off the largest off-diagonal elements until the resultant matrix is diagonal or maxIterations.
  Matrix3 b = covariance;
  Matrix3 jacobi;
  jacobi.SetIdentity();
  for (int i = 0; i < maxIterations; i++)
  {
    //The matrix of eigenvectors can be computed by product of the transformed matrices. 
    jacobi = jacobi * ComputeJacobiRotation(b);
    b = ComputeJacobiRotation(b).Inverted() * b * ComputeJacobiRotation(b);  
  }
  eigenValues = Vector3(b.m00, b.m11, b.m22);
  eigenVectors = jacobi;

}


//-----------------------------------------------------------------------------Sphere
Sphere::Sphere()
{
  mCenter = Vector3::cZero;
  mRadius = 0;
}

Sphere::Sphere(const Vector3& center, float radius)
{
  mCenter = center;
  mRadius = radius;
}


void Sphere::ComputeCentroid(const std::vector<Vector3>& points)
{
  // The centroid method is roughly describe as: find the centroid (not mean) of all
  // points and then find the furthest away point from the centroid.
  Vector3 aabbMax = points.front();
  Vector3 aabbMin = points.front();
  std::vector<Vector3>::const_iterator iter;
  for (iter = points.begin(); iter != points.end(); ++iter)
  {
    if (iter->x > aabbMax.x)
      aabbMax.x = iter->x;
    else if(iter->x < aabbMin.x)
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
  mCenter = 0.5f * (aabbMin + aabbMax);
  
  //loop through all points to find the furthest points with center
  Vector3 temp = points.front();
  float distance = Length(mCenter - temp);
  for (iter = points.begin(); iter != points.end(); ++iter)
  {
    float newdistance = Length(mCenter - *iter);
    if (newdistance > distance)
      distance = newdistance;
  }
  mRadius = distance;
}

float Euclidean(const Vector3 &p1, const Vector3 &p2)
{
  return sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}
void Sphere::ComputeRitter(const std::vector<Vector3>& points)
{
  // The ritter method:
  // Find the largest spread on each axis.
  // Find which axis' pair of points are the furthest (euclidean distance) apart.
  // Choose the center of this line as the sphere center. Now incrementally expand the sphere.
  Vector3 xmax = points[0], xmin = points[0];
  Vector3 ymax = points[0], ymin = points[0];
  Vector3 zmax = points[0], zmin = points[0];
  for (size_t i = 0; i < points.size(); i++)
  {
    if (xmax.x < points[i].x)
      xmax = points[i];
    else if (xmin.x > points[i].x)
      xmin = points[i];

    if (ymax.y < points[i].y)
      ymax = points[i];
    else if (ymin.y > points[i].y)
      ymin = points[i];

    if (zmax.z < points[i].z)
      zmax = points[i];
    else if (zmin.z > points[i].z)
      zmin = points[i];
  }
  //find largest axis and init BS
  float distx = Euclidean(xmax, xmin);
  float disty = Euclidean(ymax, ymin);
  float distz = Euclidean(zmax, zmin);
  if (distx >= disty && distx >= distz)
  {
    mCenter = (xmax + xmin) * 0.5f;
    mRadius = distx * 0.5f;
  }   
  else if (disty >= distz && disty >= distx)
  {
    mCenter = (ymax + ymin) * 0.5f;
    mRadius = disty * 0.5f;
  }   
  else if (distz >= distx && distz >= disty)
  {
    mCenter = (zmax + zmin) * 0.5f;
    mRadius = distz * 0.5f;
  }
  //loop through all points
  for (size_t i = 0; i < points.size(); i++)
  {
    //ignore the radius smaller than axis radius to expand
    if(ContainsPoint(points[i]))
      continue;
    //compute back point b = c - r * ((p-c) / |p-c|)
    Vector3 pc = (points[i] - mCenter);
    pc.AttemptNormalize();
    Vector3 b = mCenter - (mRadius *  pc);
    mCenter = 0.5f * (b + points[i]);
    mRadius = Euclidean(points[i], b) * 0.5f;
  }
}

void FindFarestpointsInAxis(Vector3 &direction, const std::vector<Vector3>& points, int &imin, int &imax)
{
	float minproj = FLT_MAX, maxproj = -FLT_MAX;
	for (size_t i = 0; i < points.size(); i++)
	{
		float projection = points[i].Dot(direction);
		if(projection < minproj)
		{
			minproj = projection;
			imin = i;
		}
		if(projection > maxproj)
		{
			maxproj = projection;
			imax = i;
		}
	}
}
void Sphere::ComputePCA(const std::vector<Vector3>& points)
{
  // The PCA method:
  // Compute the eigen values and vectors. Take the largest eigen vector as the axis of largest spread.
  // Compute the sphere center as the center of this axis then expand by all points.
  
  //get the covariance matrix
  Matrix3 covar = ComputeCovarianceMatrix(points);
  Vector3 eigenvalues;
  Matrix3 eigenvectors;
  ComputeEigenValuesAndVectors(covar, eigenvalues, eigenvectors, 50);
  //find the largest in eigenvalue
	Vector3 chosen_axis;
	if (eigenvalues.x >= eigenvalues.y && eigenvalues.x >= eigenvalues.z)
		chosen_axis = eigenvectors.BasisX();
	else if (eigenvalues.y >= eigenvalues.x && eigenvalues.y >= eigenvalues.z)
		chosen_axis = eigenvectors.BasisY();
	else if (eigenvalues.z >= eigenvalues.x && eigenvalues.z >= eigenvalues.y)
		chosen_axis = eigenvectors.BasisZ();

	Vector3 maxpoint, minpoint;
	int max, min;
	FindFarestpointsInAxis(chosen_axis, points, min, max);
	maxpoint = points[max];
	minpoint = points[min];
  mCenter = (maxpoint + minpoint) * 0.5f;
  mRadius = Euclidean(maxpoint, mCenter);
  //expand the sphere

  for (size_t i = 0; i < points.size(); i++)
  {
    //expand all points
    if (ContainsPoint(points[i]))
      continue;
    //compute back point b = c - r * ((p-c) / |p-c|)
    Vector3 pc = (points[i] - mCenter);
    pc.AttemptNormalize();
    Vector3 b = mCenter - (mRadius *  pc);
    mCenter = 0.5f * (b + points[i]);
    mRadius = Euclidean(points[i], b) * 0.5f;
  }
 
}

bool Sphere::ContainsPoint(const Vector3& point)
{
  return PointSphere(point, mCenter, mRadius);
}

Vector3 Sphere::GetCenter() const
{
  return mCenter;
}

float Sphere::GetRadius() const
{
  return mRadius;
}

bool Sphere::Compare(const Sphere& rhs, float epsilon) const
{
  float posDiff = Math::Length(mCenter - rhs.mCenter);
  float radiusDiff = Math::Abs(mRadius - rhs.mRadius);

  return posDiff < epsilon && radiusDiff < epsilon;
}

DebugShape& Sphere::DebugDraw() const
{
  return gDebugDrawer->DrawSphere(*this);
}

//-----------------------------------------------------------------------------Aabb
Aabb::Aabb()
{
  //set the aabb to an initial bad value (where the min is smaller than the max)
  mMin.Splat(Math::PositiveMax());
  mMax.Splat(-Math::PositiveMax());
}

Aabb::Aabb(const Vector3& min, const Vector3& max)
{
  mMin = min;
  mMax = max;
}

Aabb Aabb::BuildFromCenterAndHalfExtents(const Vector3& center, const Vector3& halfExtents)
{
  return Aabb(center - halfExtents, center + halfExtents);
}

float Aabb::GetVolume() const
{
  // Return the aabb's volume
  Vector3 diff = mMax - mMin;
  return (diff.x * diff.y * diff.z);
}

float Aabb::GetSurfaceArea() const
{
  // Return the aabb's surface area
  Vector3 diff = mMax - mMin;
  return 2.0f * (diff.x * diff.y + diff.y * diff.z + diff.z * diff.x);
}

bool Aabb::Contains(const Aabb& aabb) const
{
  // Return if aabb is completely contained in this
  if (aabb.mMax.x <= mMax.x && aabb.mMax.y <= mMax.y && aabb.mMax.z <= mMax.z &&
    aabb.mMin.x >= mMin.x && aabb.mMin.y >= mMin.y && aabb.mMin.z >= mMin.z)
    return true;
  
  return false;
}

void Aabb::Expand(const Vector3& point)
{
  for(size_t i = 0; i < 3; ++i)
  {
    mMin[i] = Math::Min(mMin[i], point[i]);
    mMax[i] = Math::Max(mMax[i], point[i]);
  }
}

Aabb Aabb::Combine(const Aabb& lhs, const Aabb& rhs)
{
  Aabb result;
  for(size_t i = 0; i < 3; ++i)
  {
    result.mMin[i] = Math::Min(lhs.mMin[i], rhs.mMin[i]);
    result.mMax[i] = Math::Max(lhs.mMax[i], rhs.mMax[i]);
  }
  return result;
}

bool Aabb::Compare(const Aabb& rhs, float epsilon) const
{
  float pos1Diff = Math::Length(mMin - rhs.mMin);
  float pos2Diff = Math::Length(mMax - rhs.mMax);

  return pos1Diff < epsilon && pos2Diff < epsilon;
}

void Aabb::Transform(const Vector3& scale, const Matrix3& rotation, const Vector3& translation)
{
  // Compute aabb of the this aabb after it is transformed.
  // You should use the optimize method discussed in class (not transforming all 8 points).

  //r
  Vector3 halfsize = GetHalfSize();
  Vector3 center = GetCenter();
  //abs rotation matrix
  Matrix3 absrot = rotation;
  //abs rot matrix
  for (size_t i = 0; i < 9; i++)
    absrot.array[i] = Math::Abs(rotation.array[i]);
  //r prime = |M| * (s*r)
  Vector3 newDir = Math::Transform(absrot, scale * halfsize);
  //c prime = t + M* (s*c)
  center = translation + Math::Transform(rotation, scale * center);
  mMax = center + newDir;
  mMin = center - newDir;
}

Vector3 Aabb::GetMin() const
{
  return mMin;
}

Vector3 Aabb::GetMax() const
{
  return mMax;
}

Vector3 Aabb::GetCenter() const
{
  return (mMin + mMax) * 0.5f;
}

Vector3 Aabb::GetHalfSize() const
{
  return (mMax - mMin) * 0.5f;
}

DebugShape& Aabb::DebugDraw() const
{
  return gDebugDrawer->DrawAabb(*this);
}

//-----------------------------------------------------------------------------Triangle
Triangle::Triangle()
{
  mPoints[0] = mPoints[1] = mPoints[2] = Vector3::cZero;
}

Triangle::Triangle(const Vector3& p0, const Vector3& p1, const Vector3& p2)
{
  mPoints[0] = p0;
  mPoints[1] = p1;
  mPoints[2] = p2;
}

DebugShape& Triangle::DebugDraw() const
{
  return gDebugDrawer->DrawTriangle(*this);
}

//-----------------------------------------------------------------------------Plane
Plane::Plane()
{
  mData = Vector4::cZero;
}

Plane::Plane(const Vector3& p0, const Vector3& p1, const Vector3& p2)
{
  Set(p0, p1, p2);
}

Plane::Plane(const Vector3& normal, const Vector3& point)
{
  Set(normal, point);
}

void Plane::Set(const Vector3& p0, const Vector3& p1, const Vector3& p2)
{
  // Set mData from the 3 points. Note: You should most likely normalize the plane normal.
	Math::Vector3 normal = (p1 - p0).Cross(p2 - p0);
	normal.AttemptNormalize();
	Set(normal, p0);
}

void Plane::Set(const Vector3& normal, const Vector3& point)
{
	Math::Vector3 normalizedNormal = normal;
	normalizedNormal.AttemptNormalize();
	mData.x = normalizedNormal.x;
	mData.y = normalizedNormal.y;
	mData.z = normalizedNormal.z;
	mData.w = normalizedNormal.Dot(point);
	
}

Vector3 Plane::GetNormal() const
{
  return Vector3(mData.x, mData.y, mData.z);
}

float Plane::GetDistance() const
{
  return mData.w;
}

DebugShape& Plane::DebugDraw(float size) const
{
  return DebugDraw(size, size);
}

DebugShape& Plane::DebugDraw(float sizeX, float sizeY) const
{
  return gDebugDrawer->DrawPlane(*this, sizeX, sizeY);
}

//-----------------------------------------------------------------------------Frustum
void Frustum::Set(const Vector3& lbn, const Vector3& rbn, const Vector3& rtn, const Vector3& ltn,
                  const Vector3& lbf, const Vector3& rbf, const Vector3& rtf, const Vector3& ltf)
{
  mPoints[0] = lbn;
  mPoints[1] = rbn;
  mPoints[2] = rtn;
  mPoints[3] = ltn;
  mPoints[4] = lbf;
  mPoints[5] = rbf;
  mPoints[6] = rtf;
  mPoints[7] = ltf;

  //left
  mPlanes[0].Set(lbf, ltf, lbn);
  //right
  mPlanes[1].Set(rbn, rtf, rbf);
  //top
  mPlanes[2].Set(ltn, ltf, rtn);
  //bot
  mPlanes[3].Set(rbn, lbf, lbn);
  //near
  mPlanes[4].Set(lbn, ltn, rbn);
  //far
  mPlanes[5].Set(rbf, rtf, lbf);
}

Math::Vector4* Frustum::GetPlanes() const
{
  return (Vector4*)mPlanes;
}

DebugShape& Frustum::DebugDraw() const
{
  return gDebugDrawer->DrawFrustum(*this);
}
