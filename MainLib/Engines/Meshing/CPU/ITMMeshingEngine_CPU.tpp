// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMMeshingEngine_CPU.h"
#include "../Shared/ITMMeshingEngine_Shared.h"

using namespace ITMLib;

template<class TVoxel>
void ITMMeshingEngine_CPU<TVoxel, ITMVoxelBlockHash>::MeshScene(ITMMesh *mesh, const ITMScene<TVoxel, ITMVoxelBlockHash> *scene,bool checkVoxelState)
{
	ITMMesh::Triangle *triangles = mesh->triangles->GetData(MEMORYDEVICE_CPU);
    ITMMesh::Normal *normals = nullptr;
    ITMMesh::Color *colors = nullptr;
    if(mesh->hasNormal) normals = mesh->normals->GetData(MEMORYDEVICE_CPU);
    if(mesh->hasColor)  colors  = mesh->colors->GetData(MEMORYDEVICE_CPU);

	const TVoxel *localVBA = scene->localVBA->GetVoxelBlocks();
	const ITMHashEntry *hashTable = scene->index->GetEntries();

	int noTriangles = 0, noMaxTriangles = mesh->noMaxTriangles, noTotalEntries = scene->index->noTotalEntries;
	float factor = scene->sceneParams->voxelSize;

	mesh->triangles->Clear();
	if(mesh->hasNormal) mesh->normals->Clear();
	if(mesh->hasColor) mesh->colors->Clear();

	for (int entryId = 0; entryId < noTotalEntries; entryId++)
	{
		Vector3i globalPos;
		const ITMHashEntry &currentHashEntry = hashTable[entryId];

		if (currentHashEntry.ptr < 0) continue;

		globalPos = currentHashEntry.pos.toInt() * SDF_BLOCK_SIZE;

		for (int z = 0; z < SDF_BLOCK_SIZE; z++) for (int y = 0; y < SDF_BLOCK_SIZE; y++) for (int x = 0; x < SDF_BLOCK_SIZE; x++)
		{
			Vector3f vertList[12];
            int cubeIndex;
            cubeIndex = buildVertList<TVoxel>(vertList, globalPos, Vector3i(x, y, z), localVBA, hashTable, isoValue_, checkVoxelState);
			
			if (cubeIndex < 0) continue;

			for (int i = 0; triangleTable[cubeIndex][i] != -1; i += 3)
			{
				triangles[noTriangles].p0 = Vector4f(vertList[triangleTable[cubeIndex][i]] * factor, 0);
				triangles[noTriangles].p1 = Vector4f(vertList[triangleTable[cubeIndex][i + 1]] * factor, 0);
				triangles[noTriangles].p2 = Vector4f(vertList[triangleTable[cubeIndex][i + 2]] * factor, 0);

                if(normals) {
                    Vector4f n = calcNormal(triangles[noTriangles].p0, triangles[noTriangles].p1,
                                            triangles[noTriangles].p2);
                    n.w = 1;
                    normals[noTriangles].n0 = n;
                    normals[noTriangles].n1 = n;
                    normals[noTriangles].n2 = n;
                }

                if(colors){
                    colors[noTriangles].c0 = Vector4f(1.f,1.f,1.f,1.f);
                    colors[noTriangles].c1 = Vector4f(1.f,1.f,1.f,1.f);
                    colors[noTriangles].c2 = Vector4f(1.f,1.f,1.f,1.f);
                }

				if (noTriangles < noMaxTriangles - 1) noTriangles++;
			}
		}
	}

	mesh->noTotalTriangles = noTriangles;
}