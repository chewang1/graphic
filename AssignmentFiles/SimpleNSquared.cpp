///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#include "Precompiled.hpp"

//-----------------------------------------------------------------------------NSquaredSpatialPartition
NSquaredSpatialPartition::NSquaredSpatialPartition()
{
  mType = SpatialPartitionTypes::NSquared;
}

void NSquaredSpatialPartition::InsertData(SpatialPartitionKey& key, const SpatialPartitionData& data)
{
  // Doing this lazily (and bad, but it's n-squared...).
  // Just store as the key what the client data is so we can look it up later.
  key.mVoidKey = data.mClientData;
  mData.push_back(data.mClientData);
}

void NSquaredSpatialPartition::UpdateData(SpatialPartitionKey& key, const SpatialPartitionData& data)
{
  // Nothing to do here, update doesn't do anything
}

void NSquaredSpatialPartition::RemoveData(SpatialPartitionKey& key)
{
  // Find the key data and remove it
  for(size_t i = 0; i < mData.size(); ++i)
  {
    if(mData[i] == key.mVoidKey)
    {
      mData[i] = mData.back();
      mData.pop_back();
      break;
    }
  }
}

void NSquaredSpatialPartition::DebugDraw(int level, const Math::Matrix4& transform, const Vector4& color, int bitMask)
{
  // Nothing to debug draw
}

void NSquaredSpatialPartition::CastRay(const Ray& ray, CastResults& results)
{
  // Add everything
  for(size_t i = 0; i < mData.size(); ++i)
  {
    CastResult result;
    result.mClientData = mData[i];
    results.AddResult(result);
  }
}

void NSquaredSpatialPartition::CastFrustum(const Frustum& frustum, CastResults& results)
{
  // Add everything
  for(size_t i = 0; i < mData.size(); ++i)
  {
    CastResult result;
    result.mClientData = mData[i];
    results.AddResult(result);
  }
}

void NSquaredSpatialPartition::SelfQuery(QueryResults& results)
{
  // Add everything
  for(size_t i = 0; i < mData.size(); ++i)
  {
    for(size_t j = i + 1; j < mData.size(); ++j)
    {
      results.AddResult(QueryResult(mData[i], mData[j]));
    }
  }
}

void NSquaredSpatialPartition::GetDataFromKey(const SpatialPartitionKey& key, SpatialPartitionData& data) const
{
  data.mClientData = key.mVoidKey;
}

void NSquaredSpatialPartition::FilloutData(std::vector<SpatialPartitionQueryData>& results) const
{
  for(size_t i = 0; i < mData.size(); ++i)
  {
    SpatialPartitionQueryData data;
    data.mClientData = mData[i];
    results.push_back(data);
  }
}

//-----------------------------------------------------------------------------BoundingSphereSpatialPartition
BoundingSphereSpatialPartition::BoundingSphereSpatialPartition()
{
  mType = SpatialPartitionTypes::NSquaredSphere;
}

void BoundingSphereSpatialPartition::InsertData(SpatialPartitionKey& key, const SpatialPartitionData& data)
{
  //key.mVoidKey = data.mClientData;
  key.mUIntKey = mykey++;
  mData[key.mUIntKey] = data;
}

void BoundingSphereSpatialPartition::UpdateData(SpatialPartitionKey& key, const SpatialPartitionData& data)
{
  mData[key.mUIntKey] = data;
}

void BoundingSphereSpatialPartition::RemoveData(SpatialPartitionKey& key)
{
  
  mData.erase(key.mUIntKey);
  mykey--;
}

void BoundingSphereSpatialPartition::DebugDraw(int level, const Math::Matrix4& transform, const Vector4& color, int bitMask)
{
	for (auto iter = mData.begin(); iter != mData.end(); iter++)
	{
		DebugShape& shape = gDebugDrawer->GetNewShape();
		shape.SetTransform(transform);
		shape.Color(color);
		shape.SetMaskBit(bitMask);
		gDebugDrawer->DrawSphere(iter->second.mBoundingSphere);
	}
}

void BoundingSphereSpatialPartition::CastRay(const Ray& ray, CastResults& results)
{
	bool intersect;
	float t;
	for (auto iter = mData.begin(); iter != mData.end(); iter++)
	{
		CastResult result;
		intersect = RaySphere(ray.mStart, ray.mDirection, iter->second.mBoundingSphere.mCenter, iter->second.mBoundingSphere.mRadius, t);
		result.mClientData = iter->second.mClientData;
		result.mTime = t;
		if(intersect)
				results.AddResult(result);
	}
}

void BoundingSphereSpatialPartition::CastFrustum(const Frustum& frustum, CastResults& results)
{
	IntersectionType::Type intersect;
	size_t last_axis;
	for (auto iter = mData.begin(); iter != mData.end(); iter++)
	{
		CastResult result;
		intersect = FrustumSphere(frustum.GetPlanes(), iter->second.mBoundingSphere.mCenter, iter->second.mBoundingSphere.mRadius, last_axis);
		result.mClientData = iter->second.mClientData;
		if (intersect != IntersectionType::Type::Outside)
			results.AddResult(result);
	}
}

void BoundingSphereSpatialPartition::SelfQuery(QueryResults& results)
{
	for (auto iter = mData.begin(); iter != mData.end(); ++iter)
	{
		for (auto iter2 = iter; iter2 != mData.end(); ++iter2)
		{				
			//prevent out of range and repeat
			if (iter2 == iter)
				continue;		
			QueryResult result(iter->second.mClientData, iter2->second.mClientData);
			bool collide = SphereSphere(iter->second.mBoundingSphere.mCenter, iter->second.mBoundingSphere.mRadius, iter2->second.mBoundingSphere.mCenter, iter2->second.mBoundingSphere.mRadius);
			if (collide)
				results.AddResult(result);			
		}
	}
}

void BoundingSphereSpatialPartition::FilloutData(std::vector<SpatialPartitionQueryData>& results) const
{
  for (auto iter = mData.begin(); iter != mData.end(); iter++)
  {
    SpatialPartitionQueryData data;
    data = iter->second;
    results.push_back(data);
  }
}
