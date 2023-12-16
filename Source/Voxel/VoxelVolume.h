// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "HAL/ThreadSafeCounter.h"

#include "RealtimeMeshActor.h"

#include "VoxelProceduralGeneration/Examples/VPG_TestPerlin.h"

#include "VoxelVolume.generated.h"

class AVoxelVolume;
class UBoxComponent;
class UVoxelProceduralGenerator;
struct FVoxelChunkNode;
struct FVoxelDirtyChunkData;

UCLASS()
class VOXEL_API AVoxelVolume : public ARealtimeMeshActor
{
	GENERATED_BODY()

	using FRmcUpdate = TFuture<ERealtimeMeshProxyUpdateStatus>;

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
	
	// Should smooth vertices, fairly expensive currently
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Voxel", Meta = (ClampMin = "1"))
	bool bSmoothVertexNormals = true;

	// Chunk depth, from most detailed, to create collision for
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Voxel")
	uint8 CollisionInverseDepth = 0;
	
	// Number of materials to use
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Voxel", Meta = (ClampMin = "1"))
	uint8 NumMaterials = 1;

	FVector2D GetUvFromPosAndDotVector(const FVector3f& InPos, const FVector3f& InDotVector)
	{
		FVector2D uv;

		if (InDotVector.X > InDotVector.Y && InDotVector.X > InDotVector.Z)
		{
			uv.Set(InPos.Z, InPos.Y);
		}
		else if (InDotVector.Y > InDotVector.X && InDotVector.Y > InDotVector.Z)
		{
			uv.Set(InPos.X, InPos.Z);
		}
		else
		{
			uv.Set(InPos.X, InPos.Y);
		}

		return uv;
	}
};