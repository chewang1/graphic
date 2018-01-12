///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#include "Precompiled.hpp"
#include "queue"
const float DynamicAabbTree::mFatteningFactor = 1.1f;

DynamicAabbTree::DynamicAabbTree()
{
  mType = SpatialPartitionTypes::AabbTree;
	m_root = nullptr;
	nodes.clear();
	mykey = 0;
}

DynamicAabbTree::~DynamicAabbTree()
{
	for (auto& i : nodes)
	{
		if(i.first)
		{
      delete i.second;
      i.second = nullptr;
		}	
	}
	m_root = nullptr;
}

void DynamicAabbTree::InsertData(SpatialPartitionKey& key, const SpatialPartitionData& data)
{
	key.mUIntKey = mykey;
	
	if(m_root)
	{
		Node *node = new Node();
		node->SetLeaf(key.mUIntKey, data);
    node->UpdateAABB(mFatteningFactor);  
		InsertNode(node, &m_root);
		nodes[key.mUIntKey] = node;
	}
	else
	{
		//empty so make new node
		m_root = new Node();
		m_root->SetLeaf(key.mUIntKey, data);
    m_root->UpdateAABB(mFatteningFactor);
		nodes[key.mUIntKey] = m_root;
	}
	mykey++;
}

void DynamicAabbTree::UpdateData(SpatialPartitionKey& key, const SpatialPartitionData& data)
{
  //update data
	Node *oldnode = nodes[key.mUIntKey];
	Node *newnode = new Node();
	newnode->SetLeaf(key.mUIntKey, data);

	//No need to update if aabb is still within its fattened AABB
	if (oldnode->mAabb.Contains(newnode->mAabb))
    return;

	// Remove the current leaf.
	RemoveNode(oldnode);
  newnode->UpdateAABB(mFatteningFactor);
	//insert new node
	InsertNode(newnode, &m_root);
  
}

void DynamicAabbTree::RemoveData(SpatialPartitionKey& key)
{
  //use key to find node O(1)
	Node *node = nodes[key.mUIntKey];
	RemoveNode(node);
	nodes.erase(key.mUIntKey);

}

void DynamicAabbTree::DebugDraw(int level, const Math::Matrix4& transform, const Vector4& color, int bitMask)
{	
	DrawTree(level, transform, color, bitMask, m_root);
}

void DynamicAabbTree::DrawTree(int level, const Math::Matrix4& transform, const Vector4& color, int bitMask, Node *node)
{
	if(node)
	{
		DebugShape& shape = gDebugDrawer->GetNewShape();
		shape.SetTransform(transform);
		shape.Color(color);
		shape.SetMaskBit(bitMask);		
		if (level == 0) //0 is root
			gDebugDrawer->DrawAabb(node->mAabb);
		else if (level == -1) // draw entire tree
		{
			gDebugDrawer->DrawAabb(node->mAabb);
			DrawTree(level, transform, color, bitMask, node->mLeft);
			DrawTree(level, transform, color, bitMask, node->mRight);
		}
		else if(level > 0)
			DrawTreeByLevel(m_root, level, 0);
	}
}

void DynamicAabbTree::DrawTreeByLevel(Node* node, int level, int depth)
{
	if(node != nullptr)
	{
		if(depth < level)
		{
			DrawTreeByLevel(node->mLeft, level, depth + 1);
			DrawTreeByLevel(node->mRight, level, depth + 1);
		}
		//when we get the height then start to draw
		if(depth == level)
			gDebugDrawer->DrawAabb(node->mAabb);
	}
}
void DynamicAabbTree::CastRay(const Ray& ray, CastResults& results)
{
  RayCast(m_root, ray, results);
}

void DynamicAabbTree::CastFrustum(const Frustum& frustum, CastResults& results)
{
  FrustumCast(m_root, frustum, results);
}

void DynamicAabbTree::SelfQuery(QueryResults& results)
{
	SelfQuery(m_root, results);
}

void DynamicAabbTree::FilloutData(std::vector<SpatialPartitionQueryData>& results) const
{
	//DFS +pre order
		PrintPreorder(m_root, results);
}

void DynamicAabbTree::RayCast(Node* node, const Ray& ray, CastResults& results)
{
  float t = 0.0f;
  CastResult result;
  //find the leaf, else do recursion
  if (RayAabb(ray.mStart, ray.mDirection, node->mAabb.mMin, node->mAabb.mMax, t))
  {
    if (node->IsLeaf())
    {
      result.mClientData = node->mClientData;
      result.mTime = t;
      results.AddResult(result);
    }
    else
    {
      RayCast(node->mLeft, ray, results);
      RayCast(node->mRight, ray, results);
    }
  }
}

void DynamicAabbTree::FrustumCast(Node* node, const Frustum& frustum, CastResults& results)
{

  if(node != nullptr)
  {
    IntersectionType::Type t = FrustumAabb(frustum.GetPlanes(), node->mAabb.mMin, node->mAabb.mMax, node->last_axis);
    if (t == IntersectionType::Outside) //If an aabb is outside the frustum then return
      return;
    else if (t == IntersectionType::Inside) //an aabb is full contained then add all children
      AddAllChildResult(node, results);
    else if(t == IntersectionType::Overlaps) //otherwise recursion
    {
      if (node->IsLeaf())
        AddAllChildResult(node, results);
      else
      {
        FrustumCast(node->mLeft, frustum, results);
        FrustumCast(node->mRight, frustum, results);
      }
    }
    
  }
}

void DynamicAabbTree::AddAllChildResult(Node *node, CastResults& results)
{
  if(node != nullptr)
  {
    //if get the child node then write
    if (node->IsLeaf())
      results.AddResult(CastResult(node->mClientData));
    else //start recursion to find leaf
    {
      AddAllChildResult(node->mLeft, results);
      AddAllChildResult(node->mRight, results);
    }    
  }
}
void DynamicAabbTree::PrintPreorder(Node* root, std::vector<SpatialPartitionQueryData>& results, unsigned depth) const
{
  SpatialPartitionQueryData data;
  if (root == nullptr)
    return;
  //print root
  data.mClientData = root->mClientData;
  data.mAabb = root->mAabb;
	data.mDepth = depth;
  results.push_back(data);
  //left
  PrintPreorder(root->mLeft, results, depth+1);
  //right
  PrintPreorder(root->mRight, results, depth+1);
}
void DynamicAabbTree::InsertNode(Node *newnode, Node **parent)
{
	Node *pNode = *parent;
  
	if(pNode->IsLeaf())
	{
    //split leaf node
		Node *newParent = new Node();
		newParent->mParent = pNode->mParent;
		newParent->mLeft = pNode;
		newParent->mRight = newnode;
		if (pNode->mParent != nullptr)
		{
			if (pNode->mParent->mLeft == pNode)
				pNode->mParent->mLeft = newParent;
			else
				pNode->mParent->mRight = newParent;
		}
		else
			m_root = newParent;
		pNode->mParent = newParent;
		newnode->mParent = newParent;
    //rebuild and balance
    Balance(pNode);
    return;
	}
	else
	{	
		float costLeft, costRight;
		
		Aabb aabb1 = aabb1.Combine(newnode->mAabb, pNode->mLeft->mAabb);
		float oldArea1 = pNode->mLeft->mAabb.GetSurfaceArea();
		float newArea1 = aabb1.GetSurfaceArea();
		costLeft = (newArea1 - oldArea1);
		
		Aabb aabb2 = aabb2.Combine(newnode->mAabb, pNode->mRight->mAabb);
		float oldArea2 = pNode->mRight->mAabb.GetSurfaceArea();
		float newArea2 = aabb2.GetSurfaceArea();
		costRight = (newArea2 - oldArea2);
		//if left small increase surface area < right then insert to left
			if (costLeft < costRight)
				InsertNode(newnode, &pNode->mLeft);
			else
				InsertNode(newnode, &pNode->mRight);
	}
	
}

void DynamicAabbTree::RemoveNode(Node* node)
{
  // replace parent with sibling, remove parent node
  Node *parent = node->mParent; //parent
  Node *sibling = node->GetSibling(); //silbing
  if(parent)
  {  	
		Node *grandparent = parent->mParent;
		//grandparent
    if(grandparent) //root
    {     
      // update links
      sibling->mParent = grandparent; //change parent to root
      if (parent == parent->mParent->mLeft)
				grandparent->mLeft = sibling;   
      else //parent is right so make sibling became grandparent's right
				grandparent->mRight = sibling;
    }
    else
    {
      // make sibling root
      m_root = sibling;
      sibling->mParent = nullptr;
    }
    
    delete node;
    delete parent;
    Balance(sibling, sibling->mHeight);
  }
  else //node is m_root, so parent is null
  {
    m_root = nullptr;
    delete node;
  }
  
}
void DynamicAabbTree::LeftRotate(Node*& oldparent, int &height)
{
  Node *pivot = oldparent->mRight;
  Node *smallchild = nullptr;
  if (pivot->mLeft->mHeight < pivot->mRight->mHeight)
    smallchild = pivot->mLeft;
  else
    smallchild = pivot->mRight;
  smallchild->mParent = oldparent;
  oldparent->mRight = smallchild;
  //swap oldparent and pivot and double link them
  pivot->mParent = oldparent->mParent;
  oldparent->mParent = pivot;
  if (pivot->mLeft == smallchild)
    pivot->mLeft = oldparent;
  else if (pivot->mRight == smallchild)
    pivot->mRight = oldparent;

  //deal grandparent
  if (pivot->mParent == nullptr)
    m_root = pivot;
  else
  {
    //double link with grandparent
    if (pivot->mParent->mLeft == oldparent)
      pivot->mParent->mLeft = pivot;
    else
      pivot->mParent->mRight = pivot;
  }
  oldparent->mHeight = smallchild->mHeight + 1;
  oldparent->mAabb = oldparent->mAabb.Combine(oldparent->mRight->mAabb, oldparent->mLeft->mAabb);
  oldparent = pivot;
  height = oldparent->mHeight;
}


void DynamicAabbTree::RightRotate(Node*& oldparent, int &height)
{
  Node *pivot = oldparent->mLeft;
  Node *smallchild = nullptr;
  if (pivot->mLeft->mHeight < pivot->mRight->mHeight)
    smallchild = pivot->mLeft;
  else
    smallchild = pivot->mRight;
  smallchild->mParent = oldparent;
  oldparent->mLeft = smallchild;
  //swap oldparent and pivot and double link them
  pivot->mParent = oldparent->mParent;
  oldparent->mParent = pivot;
  if (pivot->mLeft == smallchild)
    pivot->mLeft = oldparent;
  else if(pivot->mRight == smallchild)
    pivot->mRight = oldparent;

  //deal grandparent
  if (pivot->mParent == nullptr)
    m_root = pivot;
  else
  {
    //double link with grandparent
    if (pivot->mParent->mLeft == oldparent)
      pivot->mParent->mLeft = pivot;
    else
      pivot->mParent->mRight = pivot;
  }
  oldparent->mHeight = smallchild->mHeight + 1;
  oldparent->mAabb = oldparent->mAabb.Combine(oldparent->mRight->mAabb, oldparent->mLeft->mAabb);
  oldparent = pivot;
  height = oldparent->mHeight;	
}

void DynamicAabbTree::Balance(Node* node, int height)
{
  
  if(node != nullptr)
  {
    node->mHeight = height;
    if (!node->IsLeaf())
    {
      if (node->mLeft->mHeight > node->mRight->mHeight + 1)
        RightRotate(node, height);
      else if (node->mLeft->mHeight + 1 < node->mRight->mHeight)
        LeftRotate(node, height);
      node->mAabb = node->mAabb.Combine(node->mLeft->mAabb, node->mRight->mAabb);
    }
    Node *sibling = node->GetSibling();
    if (sibling)
    {
      if (sibling->mHeight > node->mHeight)
        Balance(node->mParent, sibling->mHeight + 1);
      else
        Balance(node->mParent, ++height);
    }
    else
      Balance(node->mParent, ++height);
  }
  
}

int DynamicAabbTree::TreeHeight(Node *node)
{
  if (node == nullptr)
    return 0;
  //get tree height
  return 1 + Math::Max(TreeHeight(node->mLeft), TreeHeight(node->mRight));
}

void DynamicAabbTree::SelfQuery(Node* node, QueryResults& results)
{
	if (node->IsLeaf())
		return;

	//check the two children against eeach other
	SelfQuery(node->mLeft, node->mRight, results);
	
	SelfQuery(node->mLeft, results);
	SelfQuery(node->mRight, results);
}
void DynamicAabbTree::SelfQuery(Node* a, Node* b, QueryResults& results)
{
	if(a != nullptr && b != nullptr)
	{
		if (AabbAabb(a->mAabb.mMin, a->mAabb.mMax, b->mAabb.mMin, b->mAabb.mMax))
		{
			if (a->IsLeaf() && b->IsLeaf()) //case 1 both leaf
			{
				QueryResult result = QueryResult(a->mClientData, b->mClientData);
				results.AddResult(result);
			}
				
			else if (a->IsLeaf() && !b->IsLeaf())
			{
				SelfQuery(a, b->mLeft, results);
				SelfQuery(a, b->mRight, results);
			}
			else if (!a->IsLeaf() && b->IsLeaf()) //case2 one internal, one leaf
			{
				SelfQuery(a->mLeft, b, results);
				SelfQuery(a->mRight, b, results);
			}
			else //case 3 both internal, split nodes
				SplitNodes(a, b, results);
		}
	}
}
void DynamicAabbTree::SplitNodes(Node *nodeA, Node* nodeB, QueryResults& results)
{
	if(nodeA != nullptr && nodeB != nullptr)
	{
		if (nodeA->mAabb.GetVolume() < nodeB->mAabb.GetVolume())
		{
			SelfQuery(nodeA, nodeB->mLeft, results);
			SelfQuery(nodeA, nodeB->mRight, results);
		}
		else
		{
			SelfQuery(nodeA->mLeft, nodeB, results);
			SelfQuery(nodeA->mRight, nodeB, results);
		}
	}
}
Node::Node()
{
	mClientData = nullptr;
	mLeft = nullptr;
	mRight = nullptr;
	mParent = nullptr;
	mHeight = 0;
  last_axis = 0;
}

bool Node::IsLeaf()
{
	return mRight == nullptr && mLeft == nullptr;
}

void Node::SetLeaf(const unsigned int &key, const SpatialPartitionData& data)
{
  this->mClientData = data.mClientData;
  this->mKey = key;
  this->mAabb = data.mAabb;
	mLeft = nullptr;
	mRight = nullptr;
}

void Node::SetBranch(Node* n1, Node* n2)
{
	//n1 pnode, n2 newnode
  mLeft = n1;
  mRight = n2;
	n1->mParent = this;
	n2->mParent = this;
	n1->mHeight = n2->mHeight;

}

void Node::UpdateAABB(float margin)
{
    Vector3 center = mAabb.GetCenter();
    float l = (mAabb.mMax.x - mAabb.mMin.x) * 0.5f * margin;
    float w = (mAabb.mMax.y - mAabb.mMin.y) * 0.5f * margin;
    float h = (mAabb.mMax.z - mAabb.mMin.z) * 0.5f * margin;
    mAabb.mMin = Vector3(center.x - l, center.y - w, center.z - h);
    mAabb.mMax = Vector3(center.x + l, center.y + w, center.z + h);
}

Node *Node::GetSibling(void) const
{
	if (mParent == nullptr)
		return nullptr;
  return
    this == mParent->mLeft ? mParent->mRight : mParent->mLeft;
}

