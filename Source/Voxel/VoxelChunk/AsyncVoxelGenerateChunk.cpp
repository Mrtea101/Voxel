// Fill out your copyright notice in the Description page of Project Settings.

#include "AsyncVoxelGenerateChunk.h"
#include "VoxelVolume.h"

void AsyncVoxelGenerateChunk::DoWork()
{
	VoxelVolume->RegenerateChunk(DirtyChunkData);
}