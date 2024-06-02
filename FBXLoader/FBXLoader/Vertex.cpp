#include "Vertex.h"

// 원형 리스트로 KeyFrame을 구현
Keyframe* CreateKeyFrames(uint32_t numFrame)
{
    Keyframe* pKeys = new Keyframe[numFrame];
    for (uint32_t i = 1; i < numFrame; i++) {
        pKeys[i - 1].pNext = &pKeys[i];
    }
    pKeys[numFrame - 1].pNext = &pKeys[0];

    return pKeys;
}

void DeleteKeyFrames(Keyframe* pkeys)
{
    delete[] pkeys;
}
