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
	std::vector<Joint> joints;
};

struct AnimationClip {
	Keyframe** ppKeys; // keys[boneIdx][frameIdx]
	std::wstring name;
	double duration;
	double ticksPerSec;
};

Keyframe* CreateKeyFrames(uint32_t numFrame);
void DeleteKeyFrames(Keyframe* pkeys);

struct Material {
	Vector3 ambient = Vector3(0.1f);  // 12
	float   shininess = 1.0f;         // 4
	Vector3 diffuse = Vector3(0.5f);  // 12
	float   dummy1;                   // 4
	Vector3 specular = Vector3(0.5f); // 12
	float   dummy2;                   // 4
};
