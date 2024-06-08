#include "FBXLoader.h"
#pragma comment(lib, "DirectXTK12.lib")
#pragma comment(lib, "libfbxsdk.lib")


int main(void) {
	FBXLoader loader;
    loader.Initialize();

	loader.Load(L"C:/workspace/repos/Assets/Models/zeldaPosed001/", L"zeldaPosed001.fbx");

	MeshData *pMesh = loader.m_meshes[0];

	return 0;
}