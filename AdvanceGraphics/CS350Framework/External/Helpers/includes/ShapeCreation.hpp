///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#pragma once

#include <string>
#include <vector>
#include "Math/Math.hpp"

namespace Helpers
{

void CreateCubeMesh(std::vector<Vector3>& vertices, std::vector<size_t>& indices);
void CreateSphereMesh(size_t stacks, size_t slices, float radius, std::vector<Vector3>& vertices, std::vector<size_t>& indices);
void CreateCylinderMesh(size_t slices, float height, float radius, std::vector<Vector3>& vertices, std::vector<size_t>& indices);

}//namespace Helpers
