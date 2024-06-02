#pragma once
#include "Vertex.h"

struct FACE_GROUP {
	uint32_t* pIndices; // numTriangles * 3
	uint32_t  numTriangles;
	std::wstring diffuseMapName;
};

struct MeshData {
	BasicVertex* pBasicVertices = nullptr;
	SkinnedVertex* pSkinnedVertices = nullptr;
	FACE_GROUP* pFaceGroups = nullptr;

	uint32_t           numTriangles = 0;
	uint32_t           numVertices = 0;
	uint32_t           numFaceGroups = 0;
};