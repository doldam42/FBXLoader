#pragma once
#include <fbxsdk.h>

#include <map>
#include <string>
#include <vector>

#include "Vertex.h"
#include "MeshData.h"
#include "GameUtils.h"

    // https://github.com/lang1991/FBXExporter
class fbxmanager;
class fbxscene;
class fbxmesh;
class fbxnode;
class fbxsurfacematerial;
class FBXLoader
{
    FbxManager           *m_pManager = nullptr;
    FbxScene             *m_pScene = nullptr;
    FbxGeometryConverter *m_pGeoConverter = nullptr;

  public:
    std::wstring            m_basePath;
    std::vector<MeshData *> m_meshes;

    bool     m_isSkinned = true;
    Skeleton m_skeleton;
    AnimationClip m_animation;

    std::unordered_map<unsigned int, Material *> m_materialLookUp;

  private:
    void Cleanup();

  public:
    bool Initialize();

    void Load(const wchar_t *basePath, const wchar_t *filename);

    void ReadUV(FbxMesh *inMesh, int inCtrlPointIndex, int inTextureUVIndex, int inUVLayer, Vector2 &outUV);
    void ReadNormal(FbxMesh *inMesh, int inCtrlPointIndex, int inVertexCounter, Vector3 &outNormal);
    void ReadBinormal(FbxMesh *inMesh, int inCtrlPointIndex, int inVertexCounter, Vector3 &outBinormal);
    void ReadTangent(FbxMesh *inMesh, int inCtrlPointIndex, int inVertexCounter, Vector3 &outTangent);

    UINT FindJointIndexUsingName(const std::wstring &inJointName);

    void ProcessSkeletonHierarchy(FbxNode *inRootNode);
    void ProcessSkeletonHierarchyRecursively(FbxNode *inNode, int inDepth, int myIndex, int inParentIndex);
    void ProcessJointsAndAnimations(MeshData *outMesh, FbxNode *inNode);
    void ProcessNode(FbxNode *pNode, Matrix tr, FbxNodeAttribute::EType nodeType);
    void ProcessControlPoints(Vector3 *outPositions, FbxNode *inNode);
    void ProcessMesh(FbxNode *inNode);
    void ProcessFaceGroups(MeshData *outMesh, FbxNode *inNode);
    void ProcessMaterials(MeshData *outMesh, FbxNode *inNode);
    void ProcessMaterialAttribute(FbxSurfaceMaterial *inMaterial, unsigned int inMaterialIndex);
    void ProcessMaterialTexture(FACE_GROUP *outFaceGroup, FbxSurfaceMaterial *inMaterial);

    FBXLoader() = default;
    ~FBXLoader();
};
