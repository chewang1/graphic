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

void LoadObjFile(std::string fileData, std::vector<Vector3>& outVertices, std::vector<size_t>& outIndices);
void SaveObjFile(std::string filePath, std::vector<Vector3>& vertices, std::vector<size_t>& indices);

}//namespace Helpers
