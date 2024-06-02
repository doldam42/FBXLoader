#pragma once

#include "fbxsdk.h"
#include "Vertex.h"

// 조건1. skeleton의 bone 개수와 AnimationClips의 Bone 개수는 동일하다.
class FBXLoader
{
	FbxManager* m_manager;
	FbxScene* m_scene;
	std::wstring m_basePath;
	bool m_hasAnimation;

	uint32_t m_numVertices;
	BasicVertex* m_pBasicVertices;
	SkinnedVertex* m_pSkinnedVertices;

	uint32_t m_numFaces;
	FaceGroup* m_Faces;

	Skeleton m_skeleton;

	uint32_t m_numAnimations;
	AnimationClip* m_pAnimations;
};

