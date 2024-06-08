#include "FBXLoader.h"
#include "filesystem"

static FbxAMatrix GetGeometryTransformation(FbxNode *inNode)
{
    if (!inNode)
    {
        __debugbreak();
    }
    const FbxVector4 lT = inNode->GetGeometricTranslation(FbxNode::eSourcePivot);
    const FbxVector4 lR = inNode->GetGeometricRotation(FbxNode::eSourcePivot);
    const FbxVector4 lS = inNode->GetGeometricScaling(FbxNode::eSourcePivot);

    return FbxAMatrix(lT, lR, lS);
}

static Matrix toMatrix(const FbxAMatrix &inMatrix)
{
    float m00 = static_cast<float>(inMatrix.Get(0, 0));
    float m01 = static_cast<float>(inMatrix.Get(0, 1));
    float m02 = static_cast<float>(inMatrix.Get(0, 2));
    float m03 = static_cast<float>(inMatrix.Get(0, 3));

    float m10 = static_cast<float>(inMatrix.Get(1, 0));
    float m11 = static_cast<float>(inMatrix.Get(1, 1));
    float m12 = static_cast<float>(inMatrix.Get(1, 2));
    float m13 = static_cast<float>(inMatrix.Get(1, 3));

    float m20 = static_cast<float>(inMatrix.Get(2, 0));
    float m21 = static_cast<float>(inMatrix.Get(2, 1));
    float m22 = static_cast<float>(inMatrix.Get(2, 2));
    float m23 = static_cast<float>(inMatrix.Get(2, 3));

    float m30 = static_cast<float>(inMatrix.Get(3, 0));
    float m31 = static_cast<float>(inMatrix.Get(3, 1));
    float m32 = static_cast<float>(inMatrix.Get(3, 2));
    float m33 = static_cast<float>(inMatrix.Get(3, 3));

    return Matrix(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33);
}

void FBXLoader::ProcessNode(FbxNode *inNode, Matrix tr, FbxNodeAttribute::EType nodeType)
{
    FbxNodeAttribute *attr = inNode->GetNodeAttribute();
    if (attr != NULL && attr->GetAttributeType() == nodeType)
    {
        switch (inNode->GetNodeAttribute()->GetAttributeType())
        {
        case FbxNodeAttribute::eMesh:
            ProcessMesh(inNode);
            break;
        case FbxNodeAttribute::eSkeleton:

            break;
        }
    }

    for (int i = 0; i < inNode->GetChildCount(); ++i)
    {
        ProcessNode(inNode->GetChild(i), tr, nodeType);
    }
}

void FBXLoader::ProcessSkeletonHierarchy(FbxNode *inRootNode)
{
    for (int childIndex = 0; childIndex < inRootNode->GetChildCount(); ++childIndex)
    {
        FbxNode *currNode = inRootNode->GetChild(childIndex);
        ProcessSkeletonHierarchyRecursively(currNode, 0, 0, -1);
    }
}

void FBXLoader::ProcessSkeletonHierarchyRecursively(FbxNode *inNode, int inDepth, int myIndex, int inParentIndex)
{
    if (inNode->GetNodeAttribute() && inNode->GetNodeAttribute()->GetAttributeType() &&
        inNode->GetNodeAttribute()->GetAttributeType() == FbxNodeAttribute::eSkeleton)
    {
        const char *cName = inNode->GetName();

        Joint currJoint;
        currJoint.parentIndex = inParentIndex;
        currJoint.name = std::wstring(cName, cName + strlen(cName));
        m_skeleton.joints.push_back(currJoint);
    }
    for (int i = 0; i < inNode->GetChildCount(); i++)
    {
        ProcessSkeletonHierarchyRecursively(inNode->GetChild(i), inDepth + 1, m_skeleton.joints.size(), myIndex);
    }
}

void FBXLoader::ProcessJointsAndAnimations(MeshData *outMesh, FbxNode *inNode)
{
    FbxMesh     *currMesh = inNode->GetMesh();
    unsigned int numOfDeformers = currMesh->GetDeformerCount();
    // This geometry transform is something I cannot understand
    // I think it is from MotionBuilder
    // If you are using Maya for your models, 99% this is just an
    // identity matrix
    // But I am taking it into account anyways......
    FbxAMatrix   geometryTransform = GetGeometryTransformation(inNode);

    std::vector<std::vector<uint8_t>> boneIndices(outMesh->numVertices);
    std::vector<std::vector<float>>   boneWeights(outMesh->numVertices);

    // A deformer is a FBX thing, which contains some clusters
    // A cluster contains a link, which is basically a joint
    // Normally, there is only one deformer in a mesh
    for (unsigned int deformerIndex = 0; deformerIndex < numOfDeformers; ++deformerIndex)
    {
        // There are many types of deformers in Maya,
        // We are using only skins, so we see if this is a skin
        FbxSkin *currSkin = reinterpret_cast<FbxSkin *>(currMesh->GetDeformer(deformerIndex, FbxDeformer::eSkin));
        if (!currSkin)
        {
            continue;
        }

        unsigned int numOfClusters = currSkin->GetClusterCount();
        for (unsigned int clusterIndex = 0; clusterIndex < numOfClusters; ++clusterIndex)
        {
            FbxCluster  *currCluster = currSkin->GetCluster(clusterIndex);
            std::wstring currJointName = GameUtils::s2ws(currCluster->GetLink()->GetName());
            unsigned int currJointIndex = FindJointIndexUsingName(currJointName);
            FbxAMatrix   transformMatrix;
            FbxAMatrix   transformLinkMatrix;
            FbxAMatrix   globalBindposeInverseMatrix;

            currCluster->GetTransformMatrix(transformMatrix);         // The transformation of the mesh at binding time
            currCluster->GetTransformLinkMatrix(transformLinkMatrix); // The transformation of the cluster(joint) at
                                                                      // binding time from joint space to world space
            globalBindposeInverseMatrix = transformLinkMatrix.Inverse() * transformMatrix * geometryTransform;

            // Update the information in mSkeleton
            m_skeleton.joints[currJointIndex].globalBindposeInverse = toMatrix(globalBindposeInverseMatrix);

            // Associate each joint with the control points it affects
            unsigned int numOfIndices = currCluster->GetControlPointIndicesCount();

            for (unsigned int i = 0; i < numOfIndices; ++i)
            {
                int vertexIdx = currCluster->GetControlPointIndices()[i];
                boneIndices[vertexIdx].push_back(currJointIndex);
                boneWeights[vertexIdx].push_back(currCluster->GetControlPointWeights()[i]);
            }

            // Get animation information
            // Now only supports one take

            FbxAnimStack *currAnimStack = m_pScene->GetSrcObject<FbxAnimStack>(0);
            FbxString     animStackName = currAnimStack->GetName();
            m_animation.name = GameUtils::s2ws(animStackName.Buffer());
            FbxTakeInfo *takeInfo = m_pScene->GetTakeInfo(animStackName);
            FbxTime      start = takeInfo->mLocalTimeSpan.GetStart();
            FbxTime      end = takeInfo->mLocalTimeSpan.GetStop();
            uint32_t frameCount = end.GetFrameCount(FbxTime::eFrames24) - start.GetFrameCount(FbxTime::eFrames24) + 1;

            Keyframe *currAnim = CreateKeyFrames(frameCount);

            for (FbxLongLong i = start.GetFrameCount(FbxTime::eFrames24); i <= end.GetFrameCount(FbxTime::eFrames24);
                 ++i)
            {
                FbxTime currTime;
                currTime.SetFrame(i, FbxTime::eFrames24);
                currAnim->frameNum = i;
                FbxAMatrix currentTransformOffset = inNode->EvaluateGlobalTransform(currTime) * geometryTransform;
                currAnim->globalTransform = toMatrix(currentTransformOffset.Inverse() *
                                                     currCluster->GetLink()->EvaluateGlobalTransform(currTime));
                currAnim = currAnim->pNext;
            }
            m_animation.ppKeys[currJointIndex] = currAnim;
        }
    }


    int argSortedIndices[16];
    for (int vertexIdx = 0; vertexIdx < outMesh->numVertices; vertexIdx++)
    {
        /*outMesh->pSkinnedVertices[vertexIdx].blendWeights = ;
        outMesh->pSkinnedVertices[vertexIdx].boneIndices = ;*/
        // 만약 본 가중치 개수가 4개보다 크면 4개로 고정한다.
        // 정렬해서 가장 작은 Weight를 가진
        // 귀찮은데 버블 정렬 해버려? 어쩌피 4개 이한데...
        int numIndices = boneWeights[vertexIdx].size();
        SkinnedVertex &vertex = outMesh->pSkinnedVertices[vertexIdx];
        
        for (int i = 0; i < 16; i++)
        {
            argSortedIndices[i] = i;
        }

        if (numIndices > 4)
        {
            std::vector<float> &v = boneWeights[vertexIdx];
            std::sort(argSortedIndices, argSortedIndices + numIndices,
                      [&v](size_t i1, size_t i2) { return v[i1] > v[i2]; });

            for (int i = 0; i < 4; i++)
            {
                vertex.blendWeights[i] = v[argSortedIndices[i]];
                vertex.boneIndices[i] = boneIndices[vertexIdx][argSortedIndices[i]];
            }
            for (int i = 4; i < numIndices; i++)
            {
                vertex.blendWeights[0] += v[argSortedIndices[i]];
            }

        }
        else
        {
            for (int i = 0; i < numIndices; i++)
            {
                vertex.blendWeights[i] = boneWeights[vertexIdx][i];
                vertex.boneIndices[i] = boneIndices[vertexIdx][i];
            }
        }
    }
}

void FBXLoader::Cleanup()
{
    if (m_pGeoConverter)
    {
        delete m_pGeoConverter;
        m_pGeoConverter = nullptr;
    }
    if (m_pScene)
    {
        m_pScene->Destroy();
        m_pScene = nullptr;
    }
    if (m_pManager)
    {
        m_pManager->Destroy();
        m_pManager = nullptr;
    }
    m_meshes.clear();
    for (auto itr = m_materialLookUp.begin(); itr != m_materialLookUp.end(); ++itr)
    {
        delete itr->second;
    }
    m_materialLookUp.clear();
}

bool FBXLoader::Initialize()
{
    m_pManager = FbxManager::Create();
    if (!m_pManager)
    {
        return false;
    }

    FbxIOSettings *fbxIOSettings = FbxIOSettings::Create(m_pManager, IOSROOT);

    m_pManager->SetIOSettings(fbxIOSettings);

    m_pScene = FbxScene::Create(m_pManager, "myScene");

    m_pGeoConverter = new FbxGeometryConverter(m_pManager);

    return true;
}

void FBXLoader::Load(const wchar_t *basePath, const wchar_t *filename)
{
    this->m_basePath = std::wstring(basePath);
    std::string path(m_basePath.begin(), m_basePath.end());
    path += std::string(filename, filename + wcslen(filename));

    FbxImporter *fbxImporter = FbxImporter::Create(m_pManager, "myImporter");
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

    ProcessSkeletonHierarchy(m_pScene->GetRootNode());
    if (m_skeleton.joints.empty())
    {
        m_isSkinned = false;
    }

    if (m_isSkinned)
    {
        m_animation.ppKeys = new Keyframe *[m_skeleton.joints.size()];
    }

    ProcessNode(m_pScene->GetRootNode(), Matrix(), FbxNodeAttribute::eMesh);

    fbxImporter->Destroy();
}

void FBXLoader::ReadUV(FbxMesh *inMesh, int inCtrlPointIndex, int inTextureUVIndex, int inUVLayer, Vector2 &outUV)
{
    if (inUVLayer >= 2 || inMesh->GetElementUVCount() <= inUVLayer)
    {
        throw std::exception("Invalid UV Layer Number");
    }
    FbxGeometryElementUV *vertexUV = inMesh->GetElementUV(inUVLayer);

    switch (vertexUV->GetMappingMode())
    {
    case FbxGeometryElement::eByControlPoint:
        switch (vertexUV->GetReferenceMode())
        {
        case FbxGeometryElement::eDirect: {
            outUV.x = static_cast<float>(vertexUV->GetDirectArray().GetAt(inCtrlPointIndex).mData[0]);
            outUV.y = static_cast<float>(vertexUV->GetDirectArray().GetAt(inCtrlPointIndex).mData[1]);
        }
        break;

        case FbxGeometryElement::eIndexToDirect: {
            int index = vertexUV->GetIndexArray().GetAt(inCtrlPointIndex);
            outUV.x = static_cast<float>(vertexUV->GetDirectArray().GetAt(index).mData[0]);
            outUV.y = static_cast<float>(vertexUV->GetDirectArray().GetAt(index).mData[1]);
        }
        break;

        default:
            throw std::exception("Invalid Reference");
        }
        break;

    case FbxGeometryElement::eByPolygonVertex:
        switch (vertexUV->GetReferenceMode())
        {
        case FbxGeometryElement::eDirect:
        case FbxGeometryElement::eIndexToDirect: {
            outUV.x = static_cast<float>(vertexUV->GetDirectArray().GetAt(inTextureUVIndex).mData[0]);
            outUV.y = static_cast<float>(vertexUV->GetDirectArray().GetAt(inTextureUVIndex).mData[1]);
        }
        break;

        default:
            throw std::exception("Invalid Reference");
        }
        break;
    }
}

void FBXLoader::ReadNormal(FbxMesh *inMesh, int inCtrlPointIndex, int inVertexCounter, Vector3 &outNormal)
{
    if (inMesh->GetElementNormalCount() < 1)
    {
        throw std::exception("Invalid Normal Number");
    }

    FbxGeometryElementNormal *vertexNormal = inMesh->GetElementNormal(0);
    switch (vertexNormal->GetMappingMode())
    {
    case FbxGeometryElement::eByControlPoint:
        switch (vertexNormal->GetReferenceMode())
        {
        case FbxGeometryElement::eDirect: {
            outNormal.x = static_cast<float>(vertexNormal->GetDirectArray().GetAt(inCtrlPointIndex).mData[0]);
            outNormal.y = static_cast<float>(vertexNormal->GetDirectArray().GetAt(inCtrlPointIndex).mData[1]);
            outNormal.z = static_cast<float>(vertexNormal->GetDirectArray().GetAt(inCtrlPointIndex).mData[2]);
        }
        break;

        case FbxGeometryElement::eIndexToDirect: {
            int index = vertexNormal->GetIndexArray().GetAt(inCtrlPointIndex);
            outNormal.x = static_cast<float>(vertexNormal->GetDirectArray().GetAt(index).mData[0]);
            outNormal.y = static_cast<float>(vertexNormal->GetDirectArray().GetAt(index).mData[1]);
            outNormal.z = static_cast<float>(vertexNormal->GetDirectArray().GetAt(index).mData[2]);
        }
        break;

        default:
            throw std::exception("Invalid Reference");
        }
        break;

    case FbxGeometryElement::eByPolygonVertex:
        switch (vertexNormal->GetReferenceMode())
        {
        case FbxGeometryElement::eDirect: {
            outNormal.x = static_cast<float>(vertexNormal->GetDirectArray().GetAt(inVertexCounter).mData[0]);
            outNormal.y = static_cast<float>(vertexNormal->GetDirectArray().GetAt(inVertexCounter).mData[1]);
            outNormal.z = static_cast<float>(vertexNormal->GetDirectArray().GetAt(inVertexCounter).mData[2]);
        }
        break;

        case FbxGeometryElement::eIndexToDirect: {
            int index = vertexNormal->GetIndexArray().GetAt(inVertexCounter);
            outNormal.x = static_cast<float>(vertexNormal->GetDirectArray().GetAt(index).mData[0]);
            outNormal.y = static_cast<float>(vertexNormal->GetDirectArray().GetAt(index).mData[1]);
            outNormal.z = static_cast<float>(vertexNormal->GetDirectArray().GetAt(index).mData[2]);
        }
        break;

        default:
            throw std::exception("Invalid Reference");
        }
        break;
    }
}

void FBXLoader::ReadBinormal(FbxMesh *inMesh, int inCtrlPointIndex, int inVertexCounter, Vector3 &outBinormal)
{
}

void FBXLoader::ReadTangent(FbxMesh *inMesh, int inCtrlPointIndex, int inVertexCounter, Vector3 &outTangent)
{
    if (inMesh->GetElementTangentCount() < 1)
    {
        throw std::exception("Invalid Tangent Number");
    }

    FbxGeometryElementTangent *vertexTangent = inMesh->GetElementTangent(0);
    switch (vertexTangent->GetMappingMode())
    {
    case FbxGeometryElement::eByControlPoint:
        switch (vertexTangent->GetReferenceMode())
        {
        case FbxGeometryElement::eDirect: {
            outTangent.x = static_cast<float>(vertexTangent->GetDirectArray().GetAt(inCtrlPointIndex).mData[0]);
            outTangent.y = static_cast<float>(vertexTangent->GetDirectArray().GetAt(inCtrlPointIndex).mData[1]);
            outTangent.z = static_cast<float>(vertexTangent->GetDirectArray().GetAt(inCtrlPointIndex).mData[2]);
        }
        break;

        case FbxGeometryElement::eIndexToDirect: {
            int index = vertexTangent->GetIndexArray().GetAt(inCtrlPointIndex);
            outTangent.x = static_cast<float>(vertexTangent->GetDirectArray().GetAt(index).mData[0]);
            outTangent.y = static_cast<float>(vertexTangent->GetDirectArray().GetAt(index).mData[1]);
            outTangent.z = static_cast<float>(vertexTangent->GetDirectArray().GetAt(index).mData[2]);
        }
        break;

        default:
            throw std::exception("Invalid Reference");
        }
        break;

    case FbxGeometryElement::eByPolygonVertex:
        switch (vertexTangent->GetReferenceMode())
        {
        case FbxGeometryElement::eDirect: {
            outTangent.x = static_cast<float>(vertexTangent->GetDirectArray().GetAt(inVertexCounter).mData[0]);
            outTangent.y = static_cast<float>(vertexTangent->GetDirectArray().GetAt(inVertexCounter).mData[1]);
            outTangent.z = static_cast<float>(vertexTangent->GetDirectArray().GetAt(inVertexCounter).mData[2]);
        }
        break;

        case FbxGeometryElement::eIndexToDirect: {
            int index = vertexTangent->GetIndexArray().GetAt(inVertexCounter);
            outTangent.x = static_cast<float>(vertexTangent->GetDirectArray().GetAt(index).mData[0]);
            outTangent.y = static_cast<float>(vertexTangent->GetDirectArray().GetAt(index).mData[1]);
            outTangent.z = static_cast<float>(vertexTangent->GetDirectArray().GetAt(index).mData[2]);
        }
        break;

        default:
            throw std::exception("Invalid Reference");
        }
        break;
    }
}

UINT FBXLoader::FindJointIndexUsingName(const std::wstring &inJointName)
{
    for (unsigned int i = 0; i < m_skeleton.joints.size(); ++i)
    {
        if (m_skeleton.joints[i].name == inJointName)
        {
            return i;
        }
    }

    throw std::exception("Skeleton information in FBX file is corrupted.");
}

void FBXLoader::ProcessControlPoints(Vector3 *outPositions, FbxNode *inNode)
{
    FbxMesh     *currMesh = inNode->GetMesh();
    unsigned int ctrlPointCount = currMesh->GetControlPointsCount();
    for (unsigned int i = 0; i < ctrlPointCount; ++i)
    {
        Vector3 currPosition;
        currPosition.x = static_cast<float>(currMesh->GetControlPointAt(i).mData[0]);
        currPosition.y = static_cast<float>(currMesh->GetControlPointAt(i).mData[1]);
        currPosition.z = static_cast<float>(currMesh->GetControlPointAt(i).mData[2]);

        outPositions[i] = currPosition;
    }
}

// CtrlPoint와 Vertex는 다름 여기선 Vertex 개수: 삼각형 개수 * 3으로 고정
// Only Static Mesh만 취급
void FBXLoader::ProcessMesh(FbxNode *inNode)
{
    m_meshes.push_back(new MeshData);
    MeshData *outMesh = m_meshes.back();

    FbxMesh *currMesh = inNode->GetMesh();

    UINT triCount = currMesh->GetPolygonCount();
    UINT ctrlPointCount = currMesh->GetControlPointsCount();

    Vector3 *ctrlPoints = new Vector3[ctrlPointCount];
    Vector3 *normals = new Vector3[ctrlPointCount];
    Vector2 *UVs = new Vector2[ctrlPointCount];

    ProcessControlPoints(ctrlPoints, inNode);

    int vertexCounter = 0;
    for (unsigned int i = 0; i < triCount; ++i)
    {
        for (unsigned int j = 0; j < 3; ++j)
        {
            int ctrlPointIndex = currMesh->GetPolygonVertex(i, j);

            ReadNormal(currMesh, ctrlPointIndex, vertexCounter, normals[ctrlPointIndex]);
            // Read Only Diffuse Texture
            ReadUV(currMesh, ctrlPointIndex, currMesh->GetTextureUVIndex(i, j), 0, UVs[ctrlPointIndex]);
            ++vertexCounter;
        }
    }
    outMesh->numTriangles = triCount;
    outMesh->numVertices = ctrlPointCount;

    if (m_isSkinned)
    {
        SkinnedVertex *pVertices = new SkinnedVertex[ctrlPointCount];
        for (UINT i = 0; i < ctrlPointCount; i++)
        {
            pVertices[i].position = ctrlPoints[i];
            pVertices[i].normal = normals[i];
            pVertices[i].texcoord = Vector2(UVs[i].x, 1.0f - UVs[i].y);
        }
        outMesh->pSkinnedVertices = pVertices;
    }
    else
    {
        BasicVertex *pVertices = new BasicVertex[ctrlPointCount];
        for (UINT i = 0; i < ctrlPointCount; i++)
        {
            pVertices[i].position = ctrlPoints[i];
            pVertices[i].normal = normals[i];
            pVertices[i].texcoord = Vector2(UVs[i].x, 1.0f - UVs[i].y);
        }
        outMesh->pBasicVertices = pVertices;
    }

    ProcessMaterials(outMesh, inNode);

    if (m_isSkinned)
    {
        ProcessJointsAndAnimations(outMesh, inNode);
    }

    delete[] ctrlPoints;
    delete[] normals;
    delete[] UVs;
}

void FBXLoader::ProcessFaceGroups(MeshData *outMesh, FbxNode *inNode)
{
    unsigned int materialCount = inNode->GetMaterialCount();
    FACE_GROUP  *pFaces = new FACE_GROUP[materialCount];
    uint32_t    *pIndexCounter = new uint32_t[materialCount];
    int         *pMaterialIndices = new int[m_pScene->GetMaterialCount()];

    memset(pMaterialIndices, -1, sizeof(int) * m_pScene->GetMaterialCount());
    memset(pIndexCounter, 0, sizeof(uint32_t) * materialCount);
    for (UINT i = 0; i < materialCount; i++)
    {
        pFaces[i].numTriangles = 0;
    }
    FbxLayerElementArrayTemplate<int> *materialIndices;
    FbxGeometryElement::EMappingMode   materialMappingMode = FbxGeometryElement::eNone;

    FbxMesh *currMesh = inNode->GetMesh();

    int facegroupCounter = 0;
    if (currMesh->GetElementMaterial())
    {
        materialIndices = &(currMesh->GetElementMaterial()->GetIndexArray());
        materialMappingMode = currMesh->GetElementMaterial()->GetMappingMode();

        if (materialIndices)
        {
            switch (materialMappingMode)
            {
            case FbxGeometryElement::eByPolygon: {
                if (materialIndices->GetCount() == outMesh->numTriangles)
                {
                    for (unsigned int i = 0; i < outMesh->numTriangles; ++i)
                    {
                        unsigned int materialIndex = materialIndices->GetAt(i);
                        if (pMaterialIndices[materialIndex] == -1)
                        {
                            pMaterialIndices[materialIndex] = facegroupCounter;
                            facegroupCounter++;
                        }

                        pFaces[pMaterialIndices[materialIndex]].numTriangles++;
                    }

                    for (UINT i = 0; i < materialCount; i++)
                    {
                        pFaces[i].pIndices = new uint32_t[pFaces[i].numTriangles * 3];
                    }

                    for (UINT i = 0; i < outMesh->numTriangles; i++)
                    {
                        unsigned int materialIndex = materialIndices->GetAt(i);
                        unsigned int faceGroupIndex = pMaterialIndices[materialIndex];
                        for (UINT j = 0; j < 3; j++)
                        {
                            UINT index = currMesh->GetPolygonVertex(i, j);
                            pFaces[faceGroupIndex].pIndices[pIndexCounter[faceGroupIndex]] = index;
                            pIndexCounter[faceGroupIndex]++;
                        }
                    }
                }
            }
            break;

            case FbxGeometryElement::eAllSame: {
                pFaces[0].numTriangles = outMesh->numTriangles;
                pFaces[0].pIndices = new uint32_t[pFaces[0].numTriangles * 3];

                for (UINT i = 0; i < outMesh->numTriangles; i++)
                {
                    for (UINT j = 0; j < 3; j++)
                    {
                        UINT index = currMesh->GetPolygonVertex(i, j);
                        pFaces[0].pIndices[pIndexCounter[0]] = index;
                        pIndexCounter[0]++;
                    }
                }
            }
            break;

            default:
                throw std::exception("Invalid mapping mode for material\n");
            }
        }
    }

    outMesh->numFaceGroups = materialCount;
    outMesh->pFaceGroups = pFaces;

    delete[] pMaterialIndices;
    delete[] pIndexCounter;
}

void FBXLoader::ProcessMaterials(MeshData *outMesh, FbxNode *inNode)
{
    ProcessFaceGroups(outMesh, inNode);
    unsigned int materialCount = inNode->GetMaterialCount();
    for (unsigned int i = 0; i < materialCount; ++i)
    {
        FACE_GROUP         *pFace = outMesh->pFaceGroups + i;
        FbxSurfaceMaterial *surfaceMaterial = inNode->GetMaterial(i);
        ProcessMaterialAttribute(surfaceMaterial, i);
        ProcessMaterialTexture(pFace, surfaceMaterial);

        pFace->material = *m_materialLookUp[i];
    }
}

void FBXLoader::ProcessMaterialAttribute(FbxSurfaceMaterial *inMaterial, unsigned int inMaterialIndex)
{
    FbxDouble3 double3;
    FbxDouble  double1;
    if (inMaterial->GetClassId().Is(FbxSurfacePhong::ClassId))
    {
        Material *currMaterial = new Material;

        // Amibent Color
        double3 = reinterpret_cast<FbxSurfacePhong *>(inMaterial)->Ambient;
        currMaterial->ambient.x = static_cast<float>(double3.mData[0]);
        currMaterial->ambient.y = static_cast<float>(double3.mData[1]);
        currMaterial->ambient.z = static_cast<float>(double3.mData[2]);

        // Diffuse Color
        double3 = reinterpret_cast<FbxSurfacePhong *>(inMaterial)->Diffuse;
        currMaterial->diffuse.x = static_cast<float>(double3.mData[0]);
        currMaterial->diffuse.y = static_cast<float>(double3.mData[1]);
        currMaterial->diffuse.z = static_cast<float>(double3.mData[2]);

        // Specular Color
        double3 = reinterpret_cast<FbxSurfacePhong *>(inMaterial)->Specular;
        currMaterial->specular.x = static_cast<float>(double3.mData[0]);
        currMaterial->specular.y = static_cast<float>(double3.mData[1]);
        currMaterial->specular.z = static_cast<float>(double3.mData[2]);

        // Shininess
        double1 = reinterpret_cast<FbxSurfacePhong *>(inMaterial)->Shininess;
        currMaterial->shininess = double1;

        //// Emissive Color
        // double3 = reinterpret_cast<FbxSurfacePhong *>(inMaterial)->Emissive;
        // currMaterial->mEmissive.x = static_cast<float>(double3.mData[0]);
        // currMaterial->mEmissive.y = static_cast<float>(double3.mData[1]);
        // currMaterial->mEmissive.z = static_cast<float>(double3.mData[2]);

        //// Reflection
        // double3 = reinterpret_cast<FbxSurfacePhong *>(inMaterial)->Reflection;
        // currMaterial->mReflection.x = static_cast<float>(double3.mData[0]);
        // currMaterial->mReflection.y = static_cast<float>(double3.mData[1]);
        // currMaterial->mReflection.z = static_cast<float>(double3.mData[2]);

        // Transparency Factor
        /*  double1 =
              reinterpret_cast<FbxSurfacePhong *>(inMaterial)->TransparencyFactor;
          currMaterial->mTransparencyFactor = double1;*/

        //// Specular Factor
        // double1 = reinterpret_cast<FbxSurfacePhong
        // *>(inMaterial)->SpecularFactor; currMaterial->mSpecularPower = double1;

        //// Reflection Factor
        // double1 = reinterpret_cast<FbxSurfacePhong
        // *>(inMaterial)->ReflectionFactor; currMaterial->mReflectionFactor =
        // double1;

        m_materialLookUp[inMaterialIndex] = currMaterial;
    }
    else if (inMaterial->GetClassId().Is(FbxSurfaceLambert::ClassId))
    {
        // LambertMaterial *currMaterial = new LambertMaterial();
        Material *currMaterial = new Material;
        // Amibent Color
        double3 = reinterpret_cast<FbxSurfaceLambert *>(inMaterial)->Ambient;
        currMaterial->ambient.x = static_cast<float>(double3.mData[0]);
        currMaterial->ambient.y = static_cast<float>(double3.mData[1]);
        currMaterial->ambient.z = static_cast<float>(double3.mData[2]);

        // Diffuse Color
        double3 = reinterpret_cast<FbxSurfaceLambert *>(inMaterial)->Diffuse;
        currMaterial->diffuse.x = static_cast<float>(double3.mData[0]);
        currMaterial->diffuse.y = static_cast<float>(double3.mData[1]);
        currMaterial->diffuse.z = static_cast<float>(double3.mData[2]);

        //// Emissive Color
        // double3 = reinterpret_cast<FbxSurfaceLambert *>(inMaterial)->Emissive;
        // currMaterial->mEmissive.x = static_cast<float>(double3.mData[0]);
        // currMaterial->mEmissive.y = static_cast<float>(double3.mData[1]);
        // currMaterial->mEmissive.z = static_cast<float>(double3.mData[2]);

        //// Transparency Factor
        // double1 =
        //     reinterpret_cast<FbxSurfaceLambert
        //     *>(inMaterial)->TransparencyFactor;
        // currMaterial->mTransparencyFactor = double1;

        m_materialLookUp[inMaterialIndex] = currMaterial;
    }
}

void FBXLoader::ProcessMaterialTexture(FACE_GROUP *outFaceGroup, FbxSurfaceMaterial *inMaterial)
{
    unsigned int textureIndex = 0;
    FbxProperty  property;

    FBXSDK_FOR_EACH_TEXTURE(textureIndex)
    {
        property = inMaterial->FindProperty(FbxLayerElement::sTextureChannelNames[textureIndex]);
        if (property.IsValid())
        {
            unsigned int textureCount = property.GetSrcObjectCount<FbxTexture>();
            for (unsigned int i = 0; i < textureCount; ++i)
            {
                FbxLayeredTexture *layeredTexture = property.GetSrcObject<FbxLayeredTexture>(i);
                if (layeredTexture)
                {
                    throw std::exception("Layered Texture is currently unsupported\n");
                }
                else
                {
                    FbxTexture *texture = property.GetSrcObject<FbxTexture>(i);
                    if (texture)
                    {
                        std::string     textureType = property.GetNameAsCStr();
                        FbxFileTexture *fileTexture = FbxCast<FbxFileTexture>(texture);
                        if (fileTexture)
                        {
                            if (textureType == "DiffuseColor")
                            {
                                outFaceGroup->diffuseTextureName =
                                    m_basePath + std::filesystem::path(fileTexture->GetFileName()).filename().wstring();
                            }
                            else if (textureType == "SpecularColor")
                            {
                                // ioMaterial->mSpecularMapName = fileTexture->GetFileName();
                            }
                            else if (textureType == "Bump")
                            {
                                // ioMaterial->mNormalMapName = fileTexture->GetFileName();
                            }
                        }
                    }
                }
            }
        }
    }
}

FBXLoader::~FBXLoader()
{
    Cleanup();
}
