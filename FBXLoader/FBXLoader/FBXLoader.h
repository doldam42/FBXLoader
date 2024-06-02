#include <fbxsdk.h>
#include <string>
#include <vector>
#include <unordered_map>

#include "GameUtils.h"
#include "Vertex.h"

using namespace std;

class FBXLoader {
	FbxManager* m_pManager = nullptr;
	FbxScene* m_pScene = nullptr;
	FbxGeometryConverter* m_pGeoConverter = nullptr;

	Skeleton* m_pSkeleton = nullptr;
	wstring m_basePath;

	unordered_map<unsigned int, Material*> m_materialMap;

private:
	void LoadScene(const wchar_t* basePath, const wchar_t* filename);
	void ProcessScene();
	void ProcessNode(FbxNode* pNode, FbxNodeAttribute::EType attribute);
	void ProcessMesh(FbxNode* pNode);
	void ProcessMaterial();
	void ProcessMaterialAttribute(FbxSurfaceMaterial* inMaterial, unsigned int inMaterialIndex);
	void ProcessMaterialTexture(FbxSurfaceMaterial* inMaterial);
	void ProcessAnimations();
	void ProcessAnimation(FbxNode* pNode, wstring takeName, float frameRate, float start, float end);
	void ProcessSkeleton(FbxNode* pNode);

	void ProcessBoneWeights(FbxMesh* pMesh);
	void ProcessBoneWeights(FbxSkin* pSkin);

	void Cleanup();
public:
	void Initialize();
	void Load(const wchar_t* basePath, const wchar_t* filename);

	FBXLoader() = default;
	~FBXLoader();
};