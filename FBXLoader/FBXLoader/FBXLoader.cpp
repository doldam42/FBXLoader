#include "FBXLoader.h"

void FBXLoader::Initialize()
{
	m_pManager = FbxManager::Create();
	if (!m_pManager) {
		__debugbreak();
	}

	m_pScene = FbxScene::Create(m_pManager, "");
	if (!m_pScene) {
		__debugbreak();
	}

	m_pGeoConverter = new FbxGeometryConverter(m_pManager);
}

void FBXLoader::Load(const wchar_t* basePath, const wchar_t* filename)
{
	LoadScene(basePath, filename);
}

void FBXLoader::LoadScene(const wchar_t* basePath, const wchar_t* filename)
{
    m_basePath = wstring(basePath);
    std::string path = GameUtils::ws2s(m_basePath + wstring(filename));
    
    FbxImporter* fbxImporter = FbxImporter::Create(m_pManager, "myImporter");
    if (!fbxImporter)
    {
        __debugbreak();
    }
    if (!fbxImporter->Initialize(path.c_str(), -1, m_pManager->GetIOSettings()))
    {
        __debugbreak();
    }

    if (!fbxImporter->Import(m_pScene))
    {
        __debugbreak();
    }

    m_pGeoConverter->Triangulate(m_pScene, true);

    fbxImporter->Destroy();
}

void FBXLoader::ProcessScene()
{
    ProcessMaterial();
    ProcessNode(m_pScene->GetRootNode(), FbxNodeAttribute::eSkeleton);
    ProcessNode(m_pScene->GetRootNode(), FbxNodeAttribute::eMesh);
    ProcessAnimations();
}

void FBXLoader::ProcessAnimations()
{
    for (int i = 0; i < m_pScene->GetMaterialCount(); i++) {
        FbxSurfaceMaterial* tmp = m_pScene->GetMaterial(i);
        Material* material = new Material;

    }
}
