#pragma once

#include "string"
#include "DirectXTK/SimpleMath.h"

using namespace DirectX::SimpleMath;
struct BasicVertex {
	Vector3 position;
	Vector3 normal;
	Vector2 texcoord;
};

struct SkinnedVertex {
	Vector3 position;
	Vector3 normal;
	Vector2 texcoord;

	float blendWeights[4] = { 0.0f, 0.0f, 0.0f,0.0f };
	uint8_t boneIndices[4] = { 0, 0, 0, 0 };
};

struct FaceGroup {
	uint32_t* pIndices;
	UINT numTriangles;
	std::wstring diffuseMapName;
};

struct Keyframe
{
	uint32_t frameNum;
	Matrix globalTransform;
	Keyframe* pNext = nullptr;
};

struct Joint
{
	std::wstring name;
	int parentIndex = -1;
	Matrix globalBindposeInverse = Matrix::Identity;
};

struct Skeleton
{
	Joint* pJoints;
	uint32_t numJoints;
};

struct AnimationClip {
	Keyframe** ppKeys; // keys[boneIdx][frameIdx]
	std::wstring name;
	double duration;
	double ticksPerSec;
};

Keyframe* CreateKeyFrames(uint32_t numFrame);
void DeleteKeyFrames(Keyframe* pkeys);