// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "HAL/ThreadSafeCounter.h"

#include "RealtimeMeshActor.h"
#include "RealtimeMeshLibrary.h"
#include "RealtimeMeshSimple.h"
#include "Mesh/RealtimeMeshBasicShapeTools.h"
#include "Mesh/RealtimeMeshBuilder.h"
#include "Mesh/RealtimeMeshSimpleData.h"

#include "VoxelUtilities/Array3D.h"
#include "VoxelProceduralGeneration/SignedDistanceField.h"
#include "VoxelProceduralGeneration/Examples/VPG_TestPerlin.h"

#include "VoxelVolume.generated.h"

class AVoxelVolume;
class UBoxComponent;
class UVoxelProceduralGenerator;

struct FVoxelDirtyChunkData;

using FRmcUpdate = TFuture<ERealtimeMeshProxyUpdateStatus>;

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
	FRmcUpdate fMeshUpdate;
	FRmcUpdate fCollisionUpdate;

	FArray3D<double> CornerDensityValues;
	FRealtimeMeshStreamSet StreamSet;
	bool bHasAnyVertices = false;
};

UCLASS()
class VOXEL_API AVoxelVolume : public ARealtimeMeshActor
{
	GENERATED_BODY()

	friend class AsyncVoxelGenerateChunk;

	AVoxelVolume();

protected:

	TMap<FVoxelChunkNode*, TArray<FVoxelChunkNode*>> DirtyChunkBatches;
	TMap<FVoxelChunkNode*, FVoxelDirtyChunkData*> DirtyChunkDataMap;

	FThreadSafeCounter MeshBuildingTracker;
	short NodeSectionIDTracker = 1;

	FVoxelChunkNode* RootNode = nullptr;

	virtual void BeginPlay() override;
	virtual void OnConstruction(const FTransform& Transform) override;
	virtual void TickActor(float DeltaTime, ELevelTick TickType, FActorTickFunction& ThisTickFunction) override;

	virtual void OnGenerateMesh_Implementation() override;

	bool GetLodCenter(FVector& OutLocation);
	void RebatchDirtyChunks(TMap<FVoxelChunkNode*, TArray<FVoxelChunkNode*>>& InDirtyChunkGroups);
	bool RechunkToCenter(TMap<FVoxelChunkNode*, TArray<FVoxelChunkNode*>>& OutGroupedDirtyChunks);
	void RechunkToCenter(
		const FVector& InLodCenter,
		TMap<FVoxelChunkNode*, TArray<FVoxelChunkNode*>>& OutGroupedDirtyChunks,
		FVoxelChunkNode* InMeshNode,
		FVoxelChunkNode* InParentPreviousLeaf = nullptr
	);

	void UpdateVolume(bool bShouldRechunk = true, bool bSynchronous = false);
	void RegenerateChunk(FVoxelDirtyChunkData* OutChunkMeshData);
	bool CancelNodeSection(FVoxelChunkNode* InNode, bool bDeleteIfNotCanceled = false);

public:

	// Simple bounding box visual for the editor 
	UPROPERTY(BlueprintReadOnly)
	TObjectPtr<UBoxComponent> BoundingBox;

	// Calculates voxel corner density values
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Voxel")
	TSubclassOf<UVoxelProceduralGenerator> ProceduralGeneratorClass = UVPG_TestPerlin::StaticClass();

	// Number of meshes that should be allow to async build at any given time
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Voxel")
	int MeshBuildingLimit = 32;
    
	// Total diameter of the volume's bounds
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Voxel", Meta = (ClampMin = "1"))
	double VolumeExtent = 524288;

	// Voxels per chunk
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Voxel", Meta = (ClampMin = "1"))
	int ChunkResolution = 64;
	
	// Number of subdivisions the main chunk will get to provide more detail (should be near log2(ChunkResolution))
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Voxel")
	uint8 MaxDepth = 6;

	// Factor for chunk render distance
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Voxel")
	float LodFactor = 1.f;
    
	// Threshold that determines the boundary between which corners should be considered fully active (where mesh is created)
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Voxel", Meta = (ClampMin = "0", ClampMax = "1"))
	double ActiveDensityThreshold = 1.0;

	// Chunk depth, from most detailed, to create collision for
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Voxel")
	uint8 CollisionInverseDepth = 0;
	
	// Number of materials to use
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Voxel", Meta = (ClampMin = "1"))
	uint8 NumMaterials = 1;
};