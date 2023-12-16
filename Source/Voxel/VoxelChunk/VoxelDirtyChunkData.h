// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"

#include "RealtimeMeshSimple.h"

#include "VoxelUtilities/Array3D.h"

class AVoxelVolume;
struct FVoxelDirtyChunkData;

struct FVoxelDirtyChunkData
{
	FVoxelDirtyChunkData() {};

	FVoxelDirtyChunkData(FVoxelChunkNode* InChunk, int InChunkResolution, FVoxelChunkNode* InBatchChunkKey)
	{
		Init(InChunk, InChunkResolution, InBatchChunkKey);
	}

	~FVoxelDirtyChunkData()
	{
		if (tGeneration && (tGeneration->IsIdle() || tGeneration->Cancel()))
			delete tGeneration;

		CornerDensityValues.Empty();
		StreamSet.Empty();
	}

	void Init(
		FVoxelChunkNode* InChunk = nullptr,
		int InChunkResolution = 0,
		FVoxelChunkNode* InBatchChunkKey = nullptr
	)
	{
		Chunk = InChunk;
		BatchChunkKey = InBatchChunkKey;
		CornerDensityValues = FArray3D<double>(FIntVector(InChunkResolution + 1), -1.0);
	}

	FVoxelChunkNode* Chunk = nullptr;
	FVoxelChunkNode* BatchChunkKey = nullptr;

	FAsyncTask<AsyncVoxelGenerateChunk>* tGeneration = nullptr;
	FThreadSafeBool bMeshBuilt;
	FThreadSafeBool bCollisionBuilt;

	FArray3D<double> CornerDensityValues;
	FRealtimeMeshStreamSet StreamSet;
	bool bHasAnyVertices = false;
};