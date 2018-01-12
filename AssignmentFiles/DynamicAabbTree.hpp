///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#pragma once

#include "SpatialPartition.hpp"
#include "Shapes.hpp"
class Node;
/******Student:Assignment3******/
/// You must implement a dynamic aabb tree as we discussed in class.
class DynamicAabbTree : public SpatialPartition
{
public:
  DynamicAabbTree();
  ~DynamicAabbTree();

  // Spatial Partition Interface
  void InsertData(SpatialPartitionKey& key, const SpatialPartitionData& data) override;
  void UpdateData(SpatialPartitionKey& key, const SpatialPartitionData& data) override;
  void RemoveData(SpatialPartitionKey& key) override;

	void DebugDraw(int level, const Math::Matrix4& transform, const Vector4& color = Vector4(1), int bitMask = 0) override;
	


	void CastRay(const Ray& ray, CastResults& results) override;
  void CastFrustum(const Frustum& frustum, CastResults& results) override;

  void SelfQuery(QueryResults& results) override;

  void FilloutData(std::vector<SpatialPartitionQueryData>& results) const override;
  void RayCast(Node* node, const Ray& ray, CastResults& results);
  void FrustumCast(Node* node, const Frustum& frustum, CastResults& results);
  void AddAllChildResult(Node* node, CastResults& results);


  static const float mFatteningFactor;

  // Add your implementation here
  void PrintPreorder(Node* root, std::vector<SpatialPartitionQueryData>& results, unsigned depth = 0) const;
  void InsertNode(Node *newnode, Node **parent);
	void RemoveNode(Node *node);
	void LeftRotate(Node*& oldparent, int &height);
	void RightRotate(Node*& oldparent, int &height);
	void Balance(Node* node, int height = 0);
  int TreeHeight(Node* node);
	void SelfQuery(Node* node, QueryResults& results);
	void SelfQuery(Node* a, Node* b, QueryResults& results);
	void SplitNodes(Node* nodeA, Node* nodeB, QueryResults& results);
	void DrawTree(int level, const Math::Matrix4& transform, const Vector4& color, int bitMask, Node* node);
	void DrawTreeByLevel(Node* node, int level, int depth);
	Node *m_root;
	std::unordered_map<unsigned int, Node*> nodes;
	unsigned int mykey;
};

class Node
{
public:
	Node();
	bool IsLeaf();
	void SetLeaf(const unsigned int &key, const SpatialPartitionData& data);
	void SetBranch(Node *n1, Node *n2);
  void UpdateAABB(float margin);
  Node* GetSibling() const;
  //nodes contain rigibody data, and aabb is forming for two objects or 1 object(leaf)
	Aabb mAabb;
	void* mClientData;
	Node* mLeft;
	Node* mRight;
	Node* mParent;
	size_t mHeight;
  //SpatialPartitionKey mKey;
	unsigned int mKey;
  size_t last_axis;
};