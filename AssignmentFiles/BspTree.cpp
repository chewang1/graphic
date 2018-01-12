///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#include "Precompiled.hpp"
#include <stack>
BspTreeQueryData::BspTreeQueryData()
{
  mDepth = 0;
}

BspTree::BspTree()
{
  m_root = nullptr;
}

BspTree::~BspTree()
{
	DeleteNode(m_root);
  m_root = nullptr;
}

void BspTree::DeleteNode(BspNode* node)
{
	if(node != nullptr)
	{
		DeleteNode(node->mInsideNode);
		DeleteNode(node->mOutsideNode);
		node->mTriangles.clear();
		node->mPlane.mData = Vector4::cZero;
		delete node;
	}
}
void BspTree::SplitTriangle(const Plane& plane, const Triangle& tri, TriangleList& coplanarFront, TriangleList& coplanarBack, TriangleList& front, TriangleList& back, float epsilon)
{
  IntersectionType::Type type = PlaneTriangle(plane.mData, tri.mPoints[0], tri.mPoints[1], tri.mPoints[2], epsilon);

  if (type == IntersectionType::Inside)
    front.push_back(tri);
  else if (type == IntersectionType::Outside)
    back.push_back(tri);
  else if(type == IntersectionType::Coplanar)
  {
    Vector3 planeNormal = plane.GetNormal();
    Vector3 triNormal = (tri.mPoints[1] - tri.mPoints[0]).Cross((tri.mPoints[2] - tri.mPoints[0]));
		triNormal.AttemptNormalize();

    //check triangle into the plane
		if (planeNormal.Dot(triNormal) > 0) //same dir
			coplanarFront.push_back(tri);
		else 
			coplanarBack.push_back(tri); 
  }
  else //overlap then cut triangle
  {
    std::vector<Vector3> backcollection, frontcollection;
    for (int i = 0; i < 3; i++)
    {
      Vector3 curpoint = tri.mPoints[i];
      Vector3 nextpoint = tri.mPoints[(i + 1) % 3];
      Vector3 intersect = IntersectionPoint(curpoint, nextpoint, plane);;
      IntersectionType::Type t_cur = PointPlane(curpoint, plane.mData, epsilon);
      IntersectionType::Type t_next = PointPlane(nextpoint, plane.mData, epsilon);
      
      if(t_next == IntersectionType::Inside)
      {
        if(t_cur == IntersectionType::Inside) //case 1
          frontcollection.push_back(nextpoint);
        else if(t_cur == IntersectionType::Coplanar) //case2
          frontcollection.push_back(nextpoint);
        else //case 3
        {
          frontcollection.push_back(intersect);
          frontcollection.push_back(nextpoint);
          backcollection.push_back(intersect);
        }
      }
      else if(t_next == IntersectionType::Coplanar)
      {
        if (t_cur == IntersectionType::Inside) //case 4
          frontcollection.push_back(nextpoint);
        else if (t_cur == IntersectionType::Coplanar) //case 5
          frontcollection.push_back(nextpoint);
        else //case 6
        {
          frontcollection.push_back(nextpoint);
          backcollection.push_back(nextpoint);
        }
      }
      else
      {
        if (t_cur == IntersectionType::Inside) //case 7
        {
          frontcollection.push_back(intersect);
          backcollection.push_back(intersect);
          backcollection.push_back(nextpoint);
        }     
        else if (t_cur == IntersectionType::Coplanar) //case 8
        {
          backcollection.push_back(curpoint);
          backcollection.push_back(nextpoint);
        }     
        else //case 9
          backcollection.push_back(nextpoint);
      }
    }

    if (frontcollection.size() == 3) //triangle then make triangle
    {
      front.push_back(Triangle(frontcollection[0], frontcollection[1], frontcollection[2]));
    }
    else if(frontcollection.size() == 4) //polygon then split
    {
      front.push_back(Triangle(frontcollection[0], frontcollection[1], frontcollection[2]));
      front.push_back(Triangle(frontcollection[0], frontcollection[2], frontcollection[3]));
    }
    
    if (backcollection.size() == 3)
    {
      back.push_back(Triangle(backcollection[0], backcollection[1], backcollection[2]));
    }
    else if(backcollection.size() == 4)
    {
      back.push_back(Triangle(backcollection[0], backcollection[1], backcollection[2]));
      back.push_back(Triangle(backcollection[0], backcollection[2], backcollection[3]));
    }
  }
}

float BspTree::CalculateScore(const TriangleList& triangles, size_t testIndex, float k, float epsilon)
{
  
  //build a split plane from testIndex
  Plane splitplane;
  splitplane.Set(triangles[testIndex].mPoints[0], triangles[testIndex].mPoints[1], triangles[testIndex].mPoints[2]);
  //degenerate triangles causing degenerate plane normals
  bool isdegenerated = IsDegenerated(triangles[testIndex].mPoints[0], triangles[testIndex].mPoints[1], triangles[testIndex].mPoints[2], epsilon);
  if (isdegenerated)
    return Math::PositiveMax();
  int nf = 0, nb = 0, ns = 0;
  IntersectionType::Type type;
  for (size_t i = 0; i < triangles.size(); i++)
  {
    //Do not include coplanar triangles in the score
    if(i == testIndex)
      continue;
    type = PlaneTriangle(splitplane.mData, triangles[i].mPoints[0], triangles[i].mPoints[1], triangles[i].mPoints[2], epsilon);
    if (type == IntersectionType::Inside)
      nf++;
    else if (type == IntersectionType::Outside)
      nb++;
    else if (type == IntersectionType::Overlaps)
      ns++;
  }
  //score = k * Ns + (1 - K) * Abs(Nf - Nb)
   float score = k * ns + (1 - k) * Math::Abs(nf - nb);
   return score;
}

size_t BspTree::PickSplitPlane(const TriangleList& triangles, float k, float epsilon)
{
  float lowest = Math::PositiveMax();
  float score = 0.0f;
  size_t index = 0;
  for (size_t i = 0; i < triangles.size(); i++)
  {
    score = CalculateScore(triangles, i, k, epsilon);
    if (score < lowest)
    {
      lowest = score;
      index = i;
    }     
  }
  return index;
}

void BspTree::Construct(const TriangleList& triangles, float k, float epsilon)
{
	//check tree
	if(m_root != nullptr)
	{
		DeleteNode(m_root);
		m_root = nullptr;
	}
	m_root = new BspNode();
  BuildTree(m_root, triangles, k, epsilon);
}

void BspTree::BuildTree(BspNode*& pnode, const TriangleList& triangles, float k, float epsilon)
{
  if (triangles.empty())
    return;
	
	size_t index = PickSplitPlane(triangles, k, epsilon);
	Plane splitplane(triangles[index].mPoints[0], triangles[index].mPoints[1], triangles[index].mPoints[2]);
	TriangleList coplanarFront, coplanarBack, front, back;
	for (size_t i = 0; i < triangles.size(); i++)
	{
		Triangle tri = triangles[i];
		SplitTriangle(splitplane, tri, coplanarFront, coplanarBack, front, back, epsilon);
	}
	pnode->mTriangles = coplanarFront;
	pnode->mTriangles.insert(pnode->mTriangles.end(), coplanarBack.begin(), coplanarBack.end());
	pnode->mPlane = splitplane;
	if(!front.empty())
	{
		if (!pnode->mInsideNode)
			pnode->mInsideNode = new BspNode();
		BuildTree(pnode->mInsideNode, front, k, epsilon);
	}
	if (!back.empty())
	{
		if (!pnode->mOutsideNode)
			pnode->mOutsideNode = new BspNode();
		BuildTree(pnode->mOutsideNode, back, k, epsilon);
	}
}


bool BspTree::RayCast(const Ray& ray, float& t, float planeThicknessEpsilon, float triExpansionEpsilon, int debuggingIndex)
{	
	float tmin = 0.0f, tmax = Math::PositiveMax();
  return RayCast_Node(m_root, ray, t, tmin, tmax, planeThicknessEpsilon, triExpansionEpsilon, debuggingIndex);
}


bool BspTree::RayCast_Node(BspNode* node, const Ray& ray, float& t, float tmin, float tmax, float planeThicknessEpsilon, float triExpansionEpsilon, int debuggingInde)
{
  
  if(node)
  {
		bool hit = false;
		float tplane = Math::PositiveMax();
		//check the ray mstart in which side of plane
		IntersectionType::Type side = PointPlane(ray.mStart, node->mPlane.mData, planeThicknessEpsilon);
		BspNode* near_node = nullptr,  *far_node = nullptr;
    Vector3 normalize_ray = ray.mDirection.Normalized();
    
    float cosine = node->mPlane.GetNormal().Dot(normalize_ray);
    if (Math::Abs(cosine) < 0.0001f)
      cosine = 1;

    float te = Math::Abs(planeThicknessEpsilon / (cosine));
    //float tepsilon = std::abs(planeThicknessEpsilon / (std::abs(cos_theta) < 0.001f ? 1 : cos_theta));
		//Determine Near/Far sides
		bool hitplane = false;
    if(side == IntersectionType::Inside) // inside
    {
			near_node = node->mInsideNode;
			far_node = node->mOutsideNode;
    }
		else if(side == IntersectionType::Outside)
		{
			near_node = node->mOutsideNode;
			far_node = node->mInsideNode;
			
		}
		else if(side == IntersectionType::Coplanar)
		{	
			//Case 1: The ray start is coplanar
      float temptmin = Math::PositiveMax();
			near_node = node->mInsideNode;
			far_node = node->mOutsideNode;	
			if(RayCast_Node(near_node, ray, t, tmin, tmax, planeThicknessEpsilon, triExpansionEpsilon, debuggingInde))
        hit = true; 
      if (temptmin > t)
        temptmin = t;
      float temp_t = Math::PositiveMax();
      for (auto& i : node->mTriangles)
      {
        if (RayTriangle(ray.mStart, ray.mDirection, i.mPoints[0], i.mPoints[1], i.mPoints[2], t, triExpansionEpsilon))
        {
          if (t < temp_t)
            temp_t = t;
          hit = true;
        }
      }
      t = temp_t;
      if (temptmin > t)
        temptmin = t;
      if (RayCast_Node(far_node, ray, t, tmin, tmax, planeThicknessEpsilon, triExpansionEpsilon, debuggingInde))
        hit = true;
      if (temptmin > t)
        temptmin = t;
      t = temptmin;
      return hit;
			//Check the geometry in the plane.
		}
    hitplane = RayPlane(ray.mStart, ray.mDirection, node->mPlane.mData, tplane, planeThicknessEpsilon);
			//If we hit the split plane :
    if(hitplane)
    {
      if (tmin - te <= tplane && tplane <= tmax + te) //tmin---tplane---tmax
      {
        //Recurse down the near side	[tmin, tplane]
        float temptmin = Math::PositiveMax();
        if (RayCast_Node(near_node, ray, t, tmin, tplane + te, planeThicknessEpsilon, triExpansionEpsilon, debuggingInde))
          hit = true;
        if (temptmin > t)
          temptmin = t;
        float temp_t = Math::PositiveMax();
        for (auto& i : node->mTriangles)
        {
          if (RayTriangle(ray.mStart, ray.mDirection, i.mPoints[0], i.mPoints[1], i.mPoints[2], t, triExpansionEpsilon))
          {
            if (t < temp_t)
              temp_t = t;
            hit = true;
          }
        }
        t = temp_t;
        if (temptmin > t)
          temptmin = t;
        if (RayCast_Node(far_node, ray, t, tplane - te, tmax, planeThicknessEpsilon, triExpansionEpsilon, debuggingInde))
          hit = true;
        if (temptmin > t)
          temptmin = t;
        t = temptmin;
        return hit;
      }
      else if (tplane < 0) //tplane -----0 
      {
        //only traverse near side
        hit = RayCast_Node(near_node, ray, t, tmin, tmax, planeThicknessEpsilon, triExpansionEpsilon, debuggingInde);
      }
      else if (tmax < tplane)
      {
        //only traverse near side
        hit = RayCast_Node(near_node, ray, t, tmin, tmax + te, planeThicknessEpsilon, triExpansionEpsilon, debuggingInde);
      }
      else if (0 < tplane && tplane < tmin) //case 4
      {
        //Only traverse the far side
        hit = RayCast_Node(far_node, ray, t, tplane- te, tmax, planeThicknessEpsilon, triExpansionEpsilon, debuggingInde);
      }
      return hit;
    }

      return RayCast_Node(near_node, ray, t, tmin, tmax, planeThicknessEpsilon, triExpansionEpsilon, debuggingInde);
  }
  return false;
}


void BspTree::AllTriangles(TriangleList& triangles) const
{
	AppendAllTriangles(m_root, triangles);
}

void BspTree::Invert()
{
	InvertTree(m_root);
}

void BspTree::InvertTree(BspNode* node)
{
  if (node == nullptr)
    return;
  //flip plane
  node->mPlane.mData *= -1;
  //flip triangles
  for (auto& i : node->mTriangles)
  {
    //Math::Swap(i.mPoints[0], i.mPoints[2]);
    Math::Swap(i.mPoints[0], i.mPoints[1]);
    //Math::Swap(i.mPoints[1], i.mPoints[2]);
  }

	if(node->mInsideNode)
		InvertTree(node->mInsideNode);
	if (node->mOutsideNode)
		InvertTree(node->mOutsideNode);
	BspNode* temp = node->mInsideNode;
	node->mInsideNode = node->mOutsideNode;
	node->mOutsideNode = temp;

}

void BspTree::ClipTo(BspTree* tree, float epsilon)
{
	CliptoNode(m_root, tree->m_root, epsilon);
}

void BspTree::CliptoNode(BspNode* thisnode, BspNode* node, float epsilon)
{
	if(thisnode)
	{
		thisnode->mTriangles = ClipTriangles(node, thisnode->mTriangles, epsilon);
		if (thisnode->mInsideNode)
			CliptoNode(thisnode->mInsideNode, node, epsilon);
		if (thisnode->mOutsideNode)
			CliptoNode(thisnode->mOutsideNode, node, epsilon);
	}
}

TriangleList BspTree::ClipTriangles(BspNode* node, TriangleList& thislist, float epsilon)
{
	TriangleList fronts, backs;
	//if (node->mPlane.GetNormal().Length() < epsilon)
	//{
	//	//add all triangles
	//	fronts.insert(fronts.end(), thislist.begin(), thislist.end());
	//	AllTriangles(fronts);
	//	return fronts;
	//}
	for (size_t i = 0; i < thislist.size(); i++)
		SplitTriangle(node->mPlane, thislist[i], fronts, backs, fronts, backs, epsilon);

	if (node->mInsideNode)
		fronts = ClipTriangles(node->mInsideNode, fronts, epsilon);
	if (node->mOutsideNode)
		backs = ClipTriangles(node->mOutsideNode, backs, epsilon);
	else
		backs.clear();
	fronts.insert(fronts.end(), backs.begin(), backs.end());
	//RemoveSameTriangle(fronts, epsilon);
	return fronts;
}


void BspTree::RemoveSameTriangle(TriangleList& list, float epsilon)
{
	if (list.size() <= 1)
		return;
	for (size_t i = 0; i < list.size(); i++)
	{
		Triangle tri = list[i];
		for (size_t j = i + 1; j < list.size(); j++)
		{
			Triangle nexttri = list[j];
			Vector3 diff0 = Vector3(Math::Abs(tri.mPoints[0] - nexttri.mPoints[0]));
			Vector3 diff1 = Vector3(Math::Abs(tri.mPoints[1] - nexttri.mPoints[1]));
			Vector3 diff2 = Vector3(Math::Abs(tri.mPoints[2] - nexttri.mPoints[2]));
			if(diff0.x < epsilon && diff0.y < epsilon && diff0.z < epsilon &&
				diff1.x < epsilon && diff1.y < epsilon && diff1.z < epsilon &&
				diff2.x < epsilon && diff2.y < epsilon && diff2.z < epsilon)
			{
				list.erase(list.begin() + j);
				j--;
			}
		}
	}
}
void BspTree::Union(BspTree* tree, float k, float epsilon)
{
	TriangleList results;
	ClipTo(tree, epsilon); //a clipto b
	tree->ClipTo(this, epsilon); //b clipto a
	//remove coplanar faces
	tree->Invert(); //~b
	tree->ClipTo(this, epsilon); //b clipto a
	tree->Invert(); //~b
	AllTriangles(results);
	tree->AllTriangles(results);
	Construct(results, k, epsilon);
}

void BspTree::Intersection(BspTree* tree, float k, float epsilon)
{
	//A&B = ~(~A|~B)
  TriangleList results;
	Invert(); //~A	
	tree->Invert(); //~B
	Union(tree, k, epsilon); // ~A|~B
	Invert(); //~(~A|~B)  
	AllTriangles(results);
	Construct(results, k, epsilon);
}

void BspTree::Subtract(BspTree* tree, float k, float epsilon)
{
	//TriangleList results;
	//tree->Invert(); //~B
	//Intersection(tree, k, epsilon); //A&~B
	//AllTriangles(results);
	//Construct(results, k, epsilon);
	TriangleList results;
	Invert();
	ClipTo(tree, epsilon);
	tree->ClipTo(this, epsilon);
	tree->Invert();
	tree->ClipTo(this, epsilon);
	tree->Invert();
	AllTriangles(results);
	tree->AllTriangles(results);
	Construct(results, k, epsilon);
	Invert();
	
}

void BspTree::FilloutData(std::vector<BspTreeQueryData>& results) const
{
  PrintPreorder(m_root, results);
}

void BspTree::DebugDraw(int level, const Vector4& color, int bitMask)
{
  DrawTree(level, color, bitMask, m_root);
}

void BspTree::DrawTree(int level, const Vector4& color, int bitMask, BspNode* node)
{
  if (node)
  {
    DebugShape& shape = gDebugDrawer->GetNewShape();
    shape.Color(color);
    shape.SetMaskBit(bitMask);
    if (level == 0) //0 is root
    {
      for (size_t i = 0; i < node->mTriangles.size(); i++)
        gDebugDrawer->DrawTriangle(node->mTriangles[i]); 
    }     
    else if (level == -1) // draw entire tree
    {
      for (size_t i = 0; i < node->mTriangles.size(); i++)
        gDebugDrawer->DrawTriangle(node->mTriangles[i]);
      DrawTree(level, color, bitMask, node->mInsideNode);
      DrawTree(level, color, bitMask, node->mOutsideNode);
    }
    else if (level > 0)
      DrawTreeByLevel(m_root, level, 0);
  }
}

void BspTree::DrawTreeByLevel(BspNode* node, int level, int depth)
{
  if (node != nullptr)
  {
    if (depth < level)
    {
      DrawTreeByLevel(node->mInsideNode, level, depth + 1);
      DrawTreeByLevel(node->mOutsideNode, level, depth + 1);
    }
    //when we get the height then start to draw
    if (depth == level)
    {
      for (size_t i = 0; i < node->mTriangles.size(); i++)
        gDebugDrawer->DrawTriangle(node->mTriangles[i]);
    }
  }
}
Vector3 BspTree::IntersectionPoint(Vector3 x1, Vector3 x2, Plane plane)
{
	Vector3 normal = plane.GetNormal();
	float t = (plane.mData.w - normal.Dot(x2) ) / normal.Dot(x1 - x2);
	return (t * x1 + (1 - t) * x2);
}

bool BspTree::IsDegenerated(Vector3 a, Vector3 b, Vector3 c, float epsilon)
{
  float side_1 = (b - a).Length();
  float side_2 = (c - a).Length();
  if (side_1 == 0 || side_2 == 0 || side_1 < epsilon || side_2 < epsilon)
    return true;
  float theta = (b - a).Dot(c - a) / (side_1 * side_2);
  return (theta == 1 || theta == -1) ? true : false;
  
}

void BspTree::PrintPreorder(BspNode* root, std::vector<BspTreeQueryData>& results, unsigned depth)
{
  BspTreeQueryData data;
  if (root == nullptr)
    return;
  //print root
  data.mDepth = depth;
  data.mTriangles = root->mTriangles;
  results.push_back(data);
  //left
  PrintPreorder(root->mInsideNode, results, depth + 1);
  //right
  PrintPreorder(root->mOutsideNode, results, depth + 1);
}

void BspTree::AppendAllTriangles(BspNode* node, TriangleList& list) const
{
	if(node != nullptr)
	{
		list.insert(list.end(), node->mTriangles.begin(), node->mTriangles.end());
		AppendAllTriangles(node->mInsideNode, list);
		AppendAllTriangles(node->mOutsideNode, list);
	}
	
}

BspNode::BspNode()
{
	mInsideNode = nullptr;
	mOutsideNode = nullptr;
	mParent = nullptr;
}

BspNode::BspNode(TriangleList tri)
{
  mTriangles = tri;
  mInsideNode = nullptr;
  mOutsideNode = nullptr;
  mParent = nullptr;
	if(!mTriangles.empty())
		mPlane.Set(mTriangles[0].mPoints[0], mTriangles[0].mPoints[1], mTriangles[0].mPoints[2]);
}

bool BspNode::IsLeaf()
{
  return mInsideNode == nullptr && mOutsideNode == nullptr;
}






