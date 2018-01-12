///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#include "Precompiled.hpp"

#define ShowDebugDrawWarnings true

DebugDrawer* gDebugDrawer = new DebugDrawer();

//-----------------------------------------------------------------------------DebugShape
DebugShape::DebugShape()
{
  mColor = Vector4(.6f);
  mMask = (unsigned int)-1;
  mTimer = 0;
  mOnTop = false;
  mTransform.SetIdentity();
}

DebugShape& DebugShape::Color(const Vector4& color)
{
  mColor = color;
  return *this;
}

DebugShape& DebugShape::OnTop(bool state)
{
  mOnTop = state;
  return *this;
}

DebugShape& DebugShape::Time(float time)
{
  mTimer = time;
  return *this;
}

DebugShape& DebugShape::SetMaskBit(int bitIndex)
{
  mMask = 1 << bitIndex;
  return *this;
}

DebugShape& DebugShape::SetTransform(const Matrix4& transform)
{
  mTransform = transform;
  return *this;
}

//-----------------------------------------------------------------------------DebugDrawer
DebugDrawer::DebugDrawer()
{
  mActiveMask = (unsigned int)-1;
  mApplication = NULL;
}

void DebugDrawer::Update(float dt)
{
  std::vector<DebugShape> newShapes;
  for(size_t i = 0; i < mShapes.size(); ++i)
  {
    DebugShape& shape = mShapes[i];
    shape.mTimer -= dt;

    // If the shape still has time left then add it to the list of shapes to keep drawing,
    // anything that has a timer that ran out will not be in the new list
    if(shape.mTimer >= 0)
      newShapes.push_back(shape);
  }

  mShapes.swap(newShapes);
}

void DebugDrawer::Draw()
{
  for(size_t i = 0; i < mShapes.size(); ++i)
  {
    DebugShape& shape = mShapes[i];

    // If the shape doesn't have one of the active mask bits set then don't draw it
    if((shape.mMask & mActiveMask) == 0)
      continue;
    
    // If this shape always draws on top then disable depth testing
    if(shape.mOnTop)
      glDisable(GL_DEPTH_TEST);


    // Decompose the matrix to set the gl transform (too lazy to properly transform the matrix between formats)
    float radians;
    Vector3 scale, translation, axis;
    Matrix3 rotationMat;
    shape.mTransform.Decompose(&scale, &rotationMat, &translation);
    Math::ToAxisAngle(Math::ToQuaternion(rotationMat), &axis, &radians);
    glPushMatrix();
    // Set the transform
    glTranslatef(translation.x, translation.y, translation.z);
    glRotatef(Math::RadToDeg(radians), axis.x, axis.y, axis.z);
    glScalef(scale.x, scale.y, scale.z);

    glBegin(GL_LINES);
    glColor3fv(shape.mColor.array);

    // Draw all of the line segments of this shape
    for(size_t j = 0; j < shape.mSegments.size(); ++j)
    {
      LineSegment& segment = shape.mSegments[j];

      glVertex3fv(segment.mStart.array);
      glVertex3fv(segment.mEnd.array);
    }

    glEnd();
    glPopMatrix();

    // Make sure to re-enable depth testing
    if(shape.mOnTop)
      glEnable(GL_DEPTH_TEST);
  }
}

DebugShape& DebugDrawer::GetNewShape()
{
  mShapes.push_back(DebugShape());
  return mShapes.back();
}


DebugShape& DebugDrawer::DrawPoint(const Vector3& point)
{
  return DrawSphere(Sphere(point, 0.1f));
}

DebugShape& DebugDrawer::DrawLine(const LineSegment& line)
{
  // Draw a simple line
  DebugShape& shape = GetNewShape();
  shape.mSegments.push_back(line);
  return shape;
}

DebugShape& DebugDrawer::DrawRay(const Ray& ray, float t)
{
  // Draw a ray to a given t-length. The ray must have an arrow head for visualization
  DebugShape& shape = GetNewShape();
  std::vector<LineSegment> lines;
  Math::Vector3 rayEnd = ray.mStart + (ray.mDirection * t);
  Math::Vector3 right, up;
  Math::DebugGenerateOrthonormalBasis(ray.mDirection, &up, &right);

  //arbitrary points on the ray from end
  Math::Vector3 arrowhead = rayEnd - ray.mDirection * 0.5f;
  //making arrows
  lines.push_back(LineSegment(arrowhead + right * 0.2f, rayEnd));
  lines.push_back(LineSegment(arrowhead - right * 0.2f, rayEnd));
  lines.push_back(LineSegment(arrowhead + up * 0.2f, rayEnd));
  lines.push_back(LineSegment(arrowhead - up * 0.2f, rayEnd));

  lines.push_back(LineSegment(ray.mStart, rayEnd));
  shape.mSegments = lines;
  return shape;
}

std::vector<LineSegment> DrawCircle(Math::Vector3 center, Math::Vector3 &normal, float radius)
{
	Math::Vector3 up = Math::Vector3(0.0f, 1.0f, 0.0f);
	Math::Vector3 right;
	Math::DebugGenerateOrthonormalBasis(normal, &up, &right);

	std::vector<Math::Vector3> points;
	//compute angles for get points on each angle
	float angle = Math::cTwoPi / 60.0f;
	for (float counts = 0; counts < Math::cTwoPi; counts+= angle)
	{
		points.push_back(center + (radius * up * Math::Cos(counts) + radius * right * Math::Sin(counts)));
	}

	std::vector<LineSegment> lines;
	for (unsigned i = 0; i < points.size()-1; i++)
	{
		lines.push_back(LineSegment(points[i], points[i + 1]));
	}
	lines.push_back(LineSegment(points[points.size() - 1], points[0]));
	return lines;
}
DebugShape& DebugDrawer::DrawSphere(const Sphere& sphere)
{
  // Draw a sphere with 4 rings: x-axis, y-axis, z-axis, and the horizon disc.
  // Note: To access the camera's position for the horizon disc calculation use mApplication->mCamera.mTranslation
  DebugShape& shape = GetNewShape();
  Math::Vector3 up(0.0f, 1.0f, 0.0f), right(1.0f, 0.0f, 0.0f), forward(0.0f, 0.0f, 1.0f);
  std::vector<LineSegment> lines[4];
  //with 4 rings
  lines[0] = DrawCircle(sphere.mCenter, right, sphere.mRadius);
  lines[1] = DrawCircle(sphere.mCenter, up, sphere.mRadius);
  lines[2] = DrawCircle(sphere.mCenter, forward, sphere.mRadius);

  //horizon
  Math::Vector3 hvec = sphere.mCenter - mApplication->mCamera.mTranslation;   //c- e ; d
  Math::Vector3 hvecNorm = hvec; //(c-e).normal
  hvecNorm.AttemptNormalize();
  float radius_sqr = sphere.mRadius * sphere.mRadius; //r^2
  float lLen = Math::Sqrt(hvec.LengthSq() - radius_sqr); //l = sqrt(d^2 - r^2)
  float real_r = (sphere.mRadius * lLen) / hvec.Length(); //r' = r * l /d
  float z = Math::Sqrt(radius_sqr - (real_r * real_r)); //z = sqrt(r^2 - r'^2)
  Math::Vector3 newCenter = sphere.mCenter - z * hvecNorm; //c' = c- z* (c-e).normal
  lines[3] = DrawCircle(newCenter, hvecNorm, real_r);

  for (int i = 0; i < 4; i++)
  {
	  shape.mSegments.insert(shape.mSegments.end(), lines[i].begin(), lines[i].end());
  }
  return shape;
}

DebugShape& DebugDrawer::DrawAabb(const Aabb& aabb)
{
  // Draw all edges of an aabb. Make sure to not mis-match edges!
  DebugShape& shape = GetNewShape();  
  Math::Vector3 aabbMin = aabb.GetMin();
  Math::Vector3 aabbMax = aabb.GetMax();

  /*		 z
             |1___________3 
						/|		    	 /|
				   / |	        / |
		  	 2/__|min___max/  |6___________>x
			   |	/	        |  /
	       |/___________| /
	       /5           4/
        /y
  */
  //use these 2 points to find out the other 6 points which is..
  Math::Vector3 point1(aabbMin.x, aabbMin.y, aabbMax.z);
  Math::Vector3 point2(aabbMin.x, aabbMax.y, aabbMax.z);
  Math::Vector3 point3(aabbMax.x, aabbMin.y, aabbMax.z);
  Math::Vector3 point4(aabbMax.x, aabbMax.y, aabbMin.z);
  Math::Vector3 point5(aabbMin.x, aabbMax.y, aabbMin.z);
  Math::Vector3 point6(aabbMax.x, aabbMin.y, aabbMin.z);

  //then make line segment
  shape.mSegments.push_back(LineSegment(aabbMin, point6));
  shape.mSegments.push_back(LineSegment(aabbMin, point5));
  shape.mSegments.push_back(LineSegment(aabbMin, point1));
  shape.mSegments.push_back(LineSegment(aabbMax, point2));
  shape.mSegments.push_back(LineSegment(aabbMax, point3));
  shape.mSegments.push_back(LineSegment(aabbMax, point4));

  shape.mSegments.push_back(LineSegment(point2, point5));
  shape.mSegments.push_back(LineSegment(point1, point3));
  shape.mSegments.push_back(LineSegment(point4, point5));
  shape.mSegments.push_back(LineSegment(point4, point6));
  shape.mSegments.push_back(LineSegment(point3, point6));
  shape.mSegments.push_back(LineSegment(point1, point2));
  return shape;
}

DebugShape& DebugDrawer::DrawTriangle(const Triangle& triangle)
{
  // Draw the 3 edges of a triangles
  DebugShape& shape = GetNewShape();
  shape.mSegments.push_back(LineSegment(triangle.mPoints[0], triangle.mPoints[1]));
  shape.mSegments.push_back(LineSegment(triangle.mPoints[1], triangle.mPoints[2]));
  shape.mSegments.push_back(LineSegment(triangle.mPoints[2], triangle.mPoints[0]));
  return shape;
}

DebugShape& DebugDrawer::DrawPlane(const Plane& plane, float sizeX, float sizeY)
{
  // Draw a quad with a normal at the plane's center.
  DebugShape& shape = GetNewShape();
  Math::Vector3 right, up;
  //create ordinates to get right and up
  Math::DebugGenerateOrthonormalBasis(plane.GetNormal(), &up, &right);
	//center points on the plane
	Math::Vector3 center = plane.GetNormal() * plane.mData.w;
	Math::Vector3 rayEnd = center + plane.GetNormal();
	Math::Vector3 arrowhead = rayEnd - plane.GetNormal() * 0.3f;
	//making arrows and draw normal
	shape.mSegments.push_back(LineSegment(arrowhead + right * 0.2f, rayEnd));
	shape.mSegments.push_back(LineSegment(arrowhead - right * 0.2f, rayEnd));
	shape.mSegments.push_back(LineSegment(arrowhead + up * 0.2f, rayEnd));
	shape.mSegments.push_back(LineSegment(arrowhead - up * 0.2f, rayEnd));
	shape.mSegments.push_back(LineSegment(center, rayEnd));
	//draw plane
  shape.mSegments.push_back(LineSegment(center - (up*(sizeY / 2.0f)), center - (right*(sizeX / 2.0f))));
  shape.mSegments.push_back(LineSegment(center - (up*(sizeY / 2.0f)), center + (right*(sizeX / 2.0f))));
  shape.mSegments.push_back(LineSegment(center + (up*(sizeY / 2.0f)), center - (right*(sizeX / 2.0f))));
  shape.mSegments.push_back(LineSegment(center + (up*(sizeY / 2.0f)), center + (right*(sizeX / 2.0f))));
  return shape;
}

DebugShape& DebugDrawer::DrawQuad(const Vector3& p0, const Vector3& p1, const Vector3& p2, const Vector3& p3)
{
  // Draw the4 edges of a quad. Make sure to look at this and make sure the quad is not bow-tied.
  DebugShape& shape = GetNewShape();
  shape.mSegments.push_back(LineSegment(p0, p1));
  shape.mSegments.push_back(LineSegment(p1, p2));
  shape.mSegments.push_back(LineSegment(p2, p3));
  shape.mSegments.push_back(LineSegment(p3, p0));
  return shape;
}

DebugShape& DebugDrawer::DrawFrustum(const Frustum& frustum)
{
  // Draw the 6 faces of the frustum using the 8 frustum points.
  // See Frustum.Set for the point order. For example, Points[4] is left-bottom-front.
	/*
	0 is lbn, 
	1 is rbn,
	2 is rtn,
	3 is ltn,
	4 is lbf,
	5 is rbf,
	6 is rtf,
	7 is ltf
	*/
  DebugShape& shape = GetNewShape();
  shape.mSegments.push_back(LineSegment(frustum.mPoints[0], frustum.mPoints[1]));
  shape.mSegments.push_back(LineSegment(frustum.mPoints[1], frustum.mPoints[2]));
  shape.mSegments.push_back(LineSegment(frustum.mPoints[2], frustum.mPoints[3]));
  shape.mSegments.push_back(LineSegment(frustum.mPoints[3], frustum.mPoints[0]));

  shape.mSegments.push_back(LineSegment(frustum.mPoints[2], frustum.mPoints[6]));
  shape.mSegments.push_back(LineSegment(frustum.mPoints[3], frustum.mPoints[7]));
  shape.mSegments.push_back(LineSegment(frustum.mPoints[6], frustum.mPoints[7]));
  
  shape.mSegments.push_back(LineSegment(frustum.mPoints[0], frustum.mPoints[4]));
  shape.mSegments.push_back(LineSegment(frustum.mPoints[1], frustum.mPoints[5]));
  shape.mSegments.push_back(LineSegment(frustum.mPoints[4], frustum.mPoints[5]));

  shape.mSegments.push_back(LineSegment(frustum.mPoints[4], frustum.mPoints[7]));
  shape.mSegments.push_back(LineSegment(frustum.mPoints[5], frustum.mPoints[6]));

  return shape;
}
