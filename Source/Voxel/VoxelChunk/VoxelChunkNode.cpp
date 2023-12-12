// Fill out your copyright notice in the Description page of Project Settings.

#include "VoxelChunkNode.h"

const FVector FVoxelChunkNode::NodeOffsets[8] =
{
	FVector(-0.5f, -0.5f, -0.5f),
	FVector(-0.5f, -0.5f, 0.5f),
	FVector(-0.5f, 0.5f, -0.5f),
	FVector(-0.5f, 0.5f, 0.5f),
	FVector(0.5f, -0.5f, -0.5f),
	FVector(0.5f, -0.5f, 0.5f),
	FVector(0.5f, 0.5f, -0.5f),
	FVector(0.5f, 0.5f, 0.5f)
};