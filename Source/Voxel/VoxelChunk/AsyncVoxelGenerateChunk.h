// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"

class AsyncVoxelGenerateChunk : FNonAbandonableTask
{
	friend class FAsyncTask<AsyncVoxelGenerateChunk>;

	AVoxelVolume* VoxelVolume = nullptr;
	FVoxelDirtyChunkData* DirtyChunkData = nullptr;

	AsyncVoxelGenerateChunk(AVoxelVolume* InVoxelVolume, FVoxelDirtyChunkData* InDirtyChunkData) :
		VoxelVolume(InVoxelVolume),
		DirtyChunkData(InDirtyChunkData)
	{
		check(VoxelVolume);
		check(DirtyChunkData);
	}

	void DoWork();

	FORCEINLINE TStatId GetStatId() const { RETURN_QUICK_DECLARE_CYCLE_STAT(AsyncVoxelGenerateChunk, STATGROUP_ThreadPoolAsyncTasks); }
};