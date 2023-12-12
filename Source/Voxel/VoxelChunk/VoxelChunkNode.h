// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"


struct FVoxelChunkNode
{
	static const FVector NodeOffsets[8];

	// 'n'th subdivision of the octree this node resides in (number of parent nodes)
	uint8 Depth;

	// Location in world space of the center of the chunk
	FVector Location;

	// Subdivided children chunk nodes, nullptrs if this is a leaf node, else full
	FVoxelChunkNode* Children[8] = { nullptr };

	bool bIsLeaf = false;

	short SectionID = 0;

	FVoxelChunkNode() :
		Depth(0),
		Location(FVector::ZeroVector) {};

	FVoxelChunkNode(uint8 InDepth, const FVector& InLocation) :
		Depth(InDepth),
		Location(InLocation) {};

	~FVoxelChunkNode()
	{
		for (int i = 0; i < 8; i++)
		{
			if (Children[i])
			{
				delete Children[i];
				Children[i] = nullptr;
			}
		}
	}

	const bool IsLeaf() const { return bIsLeaf; };
	void SetLeaf(bool value) { bIsLeaf = value; };

	const double GetExtent(double InVolumeExtent) const
	{
		return InVolumeExtent / exp2(Depth);
	}

	const FBox GetBox(double InVolumeExtent) const
	{
		return FBox::BuildAABB(Location, FVector(GetExtent(InVolumeExtent)));
	}

	const FVector GetChildCenter(int InChildIndex, double InVolumeExtent) const
	{
		return Location + NodeOffsets[InChildIndex] * GetExtent(InVolumeExtent);
	}

	const bool IsWithinReach(const FVector& InTargetPosition, double InVolumeExtent, float InLodFactor) const
	{
		const double chunkExtent = GetExtent(InVolumeExtent);
		const FVector distanceToCenter = (InTargetPosition - Location).GetAbs();

		FVector v = (distanceToCenter - chunkExtent).ComponentMax(FVector::ZeroVector);
		v *= v;

		const double distanceToNode = FMath::Sqrt(v.X + v.Y + v.Z);
		return distanceToNode < InLodFactor * chunkExtent * 2;
	}

	const FName GetSectionName()
	{
		return MakeSectionName(SectionID);
	}

	static const FName MakeSectionName(short InSectionID)
	{
		FString name = FString(TEXT("SectionGroup_")) + FString::FromInt(InSectionID);
		return FName(*name);
	}

	TArray<FVoxelChunkNode*> GetChildren(bool bRecurse = true)
	{
		TArray<FVoxelChunkNode*> ret;
		ret.Reserve(8);

		if (!bRecurse)
		{
			for (FVoxelChunkNode* node : Children)
			{
				if (node != nullptr)
				{
					ret.Add(node);
				}
			}
		}
		else
		{
			GetChildrenRecursive(this, ret);
		}

		return ret;
	}

	static void GetChildrenRecursive(FVoxelChunkNode* InNode, TArray<FVoxelChunkNode*>& OutChildren)
	{
		if (!InNode) return;

		for (uint8 i = 0; i < 8; i++)
		{
			if (InNode->Children[i])
			{
				OutChildren.Add(InNode->Children[i]);
				GetChildrenRecursive(InNode->Children[i], OutChildren);
			}
		}
	}
};