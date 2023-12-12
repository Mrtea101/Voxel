// Fill out your copyright notice in the Description page of Project Settings.


#include "VoxelVolume.h"

#include "Kismet/KismetMathLibrary.h"
#include "Kismet/GameplayStatics.h"
#include "Components/BillboardComponent.h"
#include "Components/BoxComponent.h"
//#include "Editor.h"
//#include "LevelEditorViewport.h"

#include "RealtimeMeshLibrary.h"
#include "RealtimeMeshSimple.h"
#include "Mesh/RealtimeMeshBasicShapeTools.h"
#include "Mesh/RealtimeMeshBuilder.h"
#include "Mesh/RealtimeMeshSimpleData.h"

#include "VoxelChunk/VoxelChunkNode.h"
#include "VoxelChunk/VoxelDirtyChunkData.h"
#include "VoxelChunk/AsyncVoxelGenerateChunk.h"
#include "VoxelProceduralGeneration/VoxelProceduralGenerator.h"
#include "VoxelUtilities/VoxelStatics.h"
#include "VoxelUtilities/Array3D.h"


AVoxelVolume::AVoxelVolume()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bStartWithTickEnabled = true;

	BoundingBox = CreateDefaultSubobject<UBoxComponent>(TEXT("Bounds"));
	BoundingBox->SetupAttachment(RootComponent);
	BoundingBox->SetCollisionEnabled(ECollisionEnabled::NoCollision);
}

void AVoxelVolume::BeginPlay()
{
	Super::BeginPlay();

	ensure(ProceduralGeneratorClass);

	OnGenerateMesh();
}

void AVoxelVolume::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);

	BoundingBox->SetBoxExtent(FVector(VolumeExtent));
}

void AVoxelVolume::RegenerateChunk(FVoxelDirtyChunkData* OutChunkMeshData)
{
	float time = UGameplayStatics::GetTimeSeconds(this);

	const int edgeCount = ChunkResolution + 1;
	const double chunkExtent = OutChunkMeshData->Chunk->GetExtent(VolumeExtent);
	const double chunkSize = chunkExtent * 2;
	const double voxelExtent = chunkExtent / ChunkResolution;
	const double voxelSize = voxelExtent * 2;

	FArray3D<double>& densityValues = OutChunkMeshData->CornerDensityValues;
	FRealtimeMeshStreamSet& streamSet = OutChunkMeshData->StreamSet;
	TRealtimeMeshBuilderLocal<uint32, FPackedNormal, FVector2DHalf, 1> builder(streamSet);
	builder.EnableTangents();
	builder.EnableTexCoords();
	builder.EnablePolyGroups();
	builder.EnableColors();

	// Try to make allocations outside the loop
	double densityBuffer[8];
	FVector edgeVertexBuffer[12];
	FIntVector cornerLocationIndex;
	FVector cornerLocationWorld;
	int idxFlag = 0;
	int edgeFlags = 0;
	int i = 0;
	int x = 0;
	int y = 0;
	int z = 0;
	double edgeOffset = 0.0;
	double c1 = 0.0;
	double c2 = 0.0;
	FVector3f a;
	FVector3f b;
	FVector3f c;
	FVector3f n;
	uint32 ia = 0;
	uint32 ib = 0;
	uint32 ic = 0;

	// Start marching cubes
	for (x = 0; x < ChunkResolution; x++)
	{
		for (y = 0; y < ChunkResolution; y++)
		{
			for (z = 0; z < ChunkResolution; z++)
			{
				// Find values at the cube's corners
				for (i = 0; i < 8; i++)
				{
					cornerLocationIndex.X = x + (int)VoxelStatics::a2fVertexOffset[i][0];
					cornerLocationIndex.Y = y + (int)VoxelStatics::a2fVertexOffset[i][1];
					cornerLocationIndex.Z = z + (int)VoxelStatics::a2fVertexOffset[i][2];

					if (densityValues[cornerLocationIndex] == -1.0)
					{
						cornerLocationWorld.Set(
							OutChunkMeshData->Chunk->Location.X - chunkExtent + cornerLocationIndex.X * voxelSize,
							OutChunkMeshData->Chunk->Location.Y - chunkExtent + cornerLocationIndex.Y * voxelSize,
							OutChunkMeshData->Chunk->Location.Z - chunkExtent + cornerLocationIndex.Z * voxelSize
						);
						densityValues[cornerLocationIndex] = ProceduralGeneratorClass.GetDefaultObject()->GenerateProceduralValue(cornerLocationWorld, VolumeExtent, GetActorLocation(), time);
					}
					
					densityBuffer[i] = densityValues[cornerLocationIndex];
				}

				// Find which vertices are inside of the surface and which are outside
				idxFlag = 0;
				for (i = 0; i < 8; i++)
				{
					if (densityBuffer[i] <= ActiveDensityThreshold)
						idxFlag |= 1 << i;
				}

				// Find which edges are intersected by the surface
				edgeFlags = VoxelStatics::aiCubeEdgeFlags[idxFlag];

				// If the cube is entirely inside or outside of the surface,
				// then there will be no intersections, continue to next cube
				if (!edgeFlags) continue;

				// Find the point of intersection of the surface with each edge
				for (i = 0; i < 12; i++)
				{
					//if there is an intersection on this edge
					if (edgeFlags & (1 << i))
					{
						c1 = densityBuffer[VoxelStatics::a2iEdgeConnection[i][0]];
						c2 = densityBuffer[VoxelStatics::a2iEdgeConnection[i][1]];
						edgeOffset = c1 == c2 ? 0.5 : FMath::Clamp((ActiveDensityThreshold - c1) / (c2 - c1), 0.0, 1.0);

						edgeVertexBuffer[i].X = (x + VoxelStatics::a2fVertexOffset[VoxelStatics::a2iEdgeConnection[i][0]][0]
							+ edgeOffset * VoxelStatics::a2fEdgeDirection[i][0]) * voxelSize - chunkExtent;

						edgeVertexBuffer[i].Y = (y + VoxelStatics::a2fVertexOffset[VoxelStatics::a2iEdgeConnection[i][0]][1]
							+ edgeOffset * VoxelStatics::a2fEdgeDirection[i][1]) * voxelSize - chunkExtent;

						edgeVertexBuffer[i].Z = (z + VoxelStatics::a2fVertexOffset[VoxelStatics::a2iEdgeConnection[i][0]][2]
							+ edgeOffset * VoxelStatics::a2fEdgeDirection[i][2]) * voxelSize - chunkExtent;
					}
					else
					{
						edgeVertexBuffer[i] = FVector::ZeroVector;
					}
				}

				//Draw the triangles that were found, there can be up to five per cube
				for (i = 0; i < 5; i++)
				{
					if (VoxelStatics::a2iTriangleConnectionTable[idxFlag][3 * i] < 0) break;

					a = FVector3f(edgeVertexBuffer[VoxelStatics::a2iTriangleConnectionTable[idxFlag][3 * i + 0]] + OutChunkMeshData->Chunk->Location);
					b = FVector3f(edgeVertexBuffer[VoxelStatics::a2iTriangleConnectionTable[idxFlag][3 * i + 1]] + OutChunkMeshData->Chunk->Location);
					c = FVector3f(edgeVertexBuffer[VoxelStatics::a2iTriangleConnectionTable[idxFlag][3 * i + 2]] + OutChunkMeshData->Chunk->Location);
					
					n = FVector3f::CrossProduct(b - a, c - a).GetUnsafeNormal();

					ia = builder.AddVertex(a)
						.SetNormalAndTangent(n, FVector3f(0,-1,0))
						.SetTexCoords(FVector2D(0,0))
						.GetIndex();

					ib = builder.AddVertex(b)
						.SetNormalAndTangent(n, FVector3f(0, -1, 0))
						.SetTexCoords(FVector2D(0, 1))
						.GetIndex();

					ic = builder.AddVertex(c)
						.SetNormalAndTangent(n, FVector3f(0, -1, 0))
						.SetTexCoords(FVector2D(1, 0))
						.GetIndex();

					builder.AddTriangle(ia, ib, ic, 0);

					OutChunkMeshData->bHasAnyVertices = true;
				}
			}
		}
	}
}

bool AVoxelVolume::RechunkToCenter(TMap<FVoxelChunkNode*, TArray<FVoxelChunkNode*>>& OutGroupedDirtyChunks)
{
	if (!RootNode)
	{
		UE_LOG(LogTemp, Warning, TEXT("[AVoxelVolume::RechunkToCenter] RootNode null"));
		return false;
	}

	FVector lodCenter(0);
	if (!GetLodCenter(lodCenter))
	{
		UE_LOG(LogTemp, Warning, TEXT("[AVoxelVolume::RechunkToCenter] Could not get lod center"));
		return false;
	}

	RechunkToCenter(lodCenter, OutGroupedDirtyChunks, RootNode);

	return OutGroupedDirtyChunks.Num() != 0;
}

void AVoxelVolume::RechunkToCenter(
	const FVector& InLodCenter,
	TMap<FVoxelChunkNode*, TArray<FVoxelChunkNode*>>& OutGroupedDirtyChunks,
	FVoxelChunkNode* InMeshNode,
	FVoxelChunkNode* InParentPreviousLeaf
)
{
	if (!InMeshNode)
	{
		UE_LOG(LogTemp, Warning, TEXT("InMeshNode null"));
		return;
	}

	if (InMeshNode->Depth == MaxDepth // at max desired node depth, this will be a leaf
		|| !InMeshNode->IsWithinReach(InLodCenter, VolumeExtent, LodFactor) // past range to expand this node, this will be a leaf
		)
	{
		if (!InMeshNode->IsLeaf()) // ensures old leafs aren't rechunked
		{
			InMeshNode->SetLeaf(true);

			// If we have a InParentPreviousLeaf, InMeshNode is the child of the less detailed node mesh which should be deleted
			if (InParentPreviousLeaf)
			{
				TArray<FVoxelChunkNode*>& group = OutGroupedDirtyChunks.FindOrAdd(InParentPreviousLeaf);
				group.Add(InMeshNode);
			}
			else // InMeshNode is the parent of more detailed children (possibly not leafs) which should be deleted
			{
				TArray<FVoxelChunkNode*>& group = OutGroupedDirtyChunks.FindOrAdd(InMeshNode);
			}
		}
	}
	else // this will not be a leaf, it's now a parent to potential leafs
	{
		// If this was a new leaf, we need to keep track of it for deletion
		// We pass it down the recursion until we get to the leafs for creation, then group them
		if (InMeshNode->IsLeaf())
		{
			InMeshNode->SetLeaf(false);
			InParentPreviousLeaf = InMeshNode;
		}

		// expand tree and recurse
		for (int i = 0; i < 8; i++)
		{
			if (!InMeshNode->Children[i])
			{
				InMeshNode->Children[i] = new FVoxelChunkNode(InMeshNode->Depth + 1, InMeshNode->GetChildCenter(i, VolumeExtent));
			}

			RechunkToCenter(InLodCenter, OutGroupedDirtyChunks, InMeshNode->Children[i], InParentPreviousLeaf);
		}
	}
}

bool AVoxelVolume::GetLodCenter(FVector& OutLocation)
{
	if (const UWorld* world = GetWorld())
	{
		// first we check player pawn position
		if (const APlayerController* PC = GetWorld()->GetFirstPlayerController())
		{
			if (APawn* pawn = PC->GetPawn())
			{
				OutLocation = UKismetMathLibrary::InverseTransformLocation(GetActorTransform(), pawn->GetActorLocation());
				return true;
			}
		}

		// then we check editor camera position
		//else if (GCurrentLevelEditingViewportClient)
		//{
		//	OutLocation = GCurrentLevelEditingViewportClient->GetViewLocation();
		//	return true;
		//}

		else
		{
			OutLocation = GetActorLocation() + FVector(0, 0, VolumeExtent);
			return true;
		}
	}

	return false;
}

void AVoxelVolume::OnGenerateMesh_Implementation()
{
	Super::OnGenerateMesh_Implementation();

	// Initialize the simple mesh
	URealtimeMeshSimple* RealtimeMesh = GetRealtimeMeshComponent()->InitializeRealtimeMesh<URealtimeMeshSimple>();

	// Setup the material slots
	for (uint8 i = 0; i < NumMaterials; i++)
	{
		RealtimeMesh->SetupMaterialSlot(i, FName("Material_", i));
	}

	for (TPair<FVoxelChunkNode*, FVoxelDirtyChunkData*>& dirtyChunk : DirtyChunkDataMap)
	{
		delete dirtyChunk.Value;
	}

	DirtyChunkDataMap.Empty();
	DirtyChunkBatches.Empty();

	if (RootNode)
	{
		delete RootNode;
	}

	RootNode = new FVoxelChunkNode();

	NodeSectionIDTracker = 1;

	UpdateVolume(true, true);
}

void AVoxelVolume::RebatchDirtyChunks(TMap<FVoxelChunkNode*, TArray<FVoxelChunkNode*>>& InDirtyChunkGroups)
{
	for (TPair<FVoxelChunkNode*, TArray<FVoxelChunkNode*>>& group : InDirtyChunkGroups)
	{
		// If this chunk was already dirty and being handled, we need to consider that
		if (auto arrayRef = DirtyChunkBatches.Find(group.Key))
		{
			// Case 1:
			// 
			// if the array has no elements, the group.Key parent was to be created and its children deleted
			// this means group.Key is no longer a leaf so we should cancel:
			//		the deletion its children (only if they appear in the group.Value array)
			//		the creation of the new children that were preserved
			if (arrayRef->IsEmpty())
			{
				if (CancelNodeSection(group.Key))
				{
					auto children = group.Key->GetChildren();
					for (auto child : children)
					{
						if (child->SectionID)
						{
							int32 idx;
							if (group.Value.Find(child, idx))
							{
								group.Value.RemoveAt(idx);
							}
						}
					}
				}
			}

			// Case 2:
			// 
			// if the array has any elements, the group.Key parent was to be deleted and the children created
			// this means group.Key is now a leaf so we should cancel:
			//		the creation of each node (if possible, if not, we need to to delete them)
			//		the deletion of the parent

			for (auto node : *arrayRef)
			{
				CancelNodeSection(node, true);
			}

			DirtyChunkBatches[group.Key] = group.Value;
		}
		else
		{
			DirtyChunkBatches.Add(group.Key, group.Value);
		}

		// If the key is a leaf, it's the parent that needs creation, its children need destruction (and node deletion)
		// Note: the value array is empty, use parents direct children and recurse
		if (group.Key->IsLeaf())
		{
			auto data = DirtyChunkDataMap.Add(group.Key, new FVoxelDirtyChunkData(group.Key, ChunkResolution, group.Key));
			data->tGeneration = new FAsyncTask<AsyncVoxelGenerateChunk>(this, data);
			data->tGeneration->StartBackgroundTask();
		}
		// If the key is not a leaf, it's a parent node that was a leaf but needs deletion, the value array children need creation
		// Note: the value array nodes possibly have greater than 1 depth from parent (could be more than 8)
		else
		{
			for (auto leaf : group.Value)
			{
				auto data = DirtyChunkDataMap.Add(leaf, new FVoxelDirtyChunkData(leaf, ChunkResolution, group.Key));
				data->tGeneration = new FAsyncTask<AsyncVoxelGenerateChunk>(this, data);
				data->tGeneration->StartBackgroundTask();
			}
		}
	}
}

bool AVoxelVolume::CancelNodeSection(FVoxelChunkNode* InNode, bool bDeleteIfNotCanceled)
{
	bool bCanceled = false;

	if (auto dirtyChunk = DirtyChunkDataMap.FindRef(InNode))
	{
		if (dirtyChunk->tGeneration)
		{
			if (!dirtyChunk->tGeneration->IsDone() && dirtyChunk->tGeneration->Cancel())
			{
				bCanceled = true;
				delete dirtyChunk;
				DirtyChunkDataMap.Remove(InNode);
			}
		}

		if (bDeleteIfNotCanceled && !bCanceled)
		{
			if (short id = InNode->SectionID)
			{
				if (URealtimeMeshSimple* RealtimeMesh = GetRealtimeMeshComponent()->GetRealtimeMeshAs<URealtimeMeshSimple>())
				{
					FName name = InNode->GetSectionName();
					auto SectionGroupKey = FRealtimeMeshSectionGroupKey::Create(0, name);
					InNode->SectionID = 0;

					RealtimeMesh->RemoveSectionGroup(SectionGroupKey)
						.Next([name](ERealtimeMeshProxyUpdateStatus Status)
							{
								UE_LOG(LogTemp, Warning, TEXT("RemoveSectionGroup Finished (Canceled Node) (%s)"), *name.ToString());
							}
					);
				}
			}
		}
	}

	return bCanceled;
}


void AVoxelVolume::TickActor(float DeltaTime, ELevelTick TickType, FActorTickFunction& ThisTickFunction)
{
	UpdateVolume();

	Super::TickActor(DeltaTime, TickType, ThisTickFunction);
}

void AVoxelVolume::UpdateVolume(bool bShouldRechunk, bool bSynchronous)
{
	URealtimeMeshSimple* RealtimeMesh = GetRealtimeMeshComponent()->GetRealtimeMeshAs<URealtimeMeshSimple>();
	if (!RealtimeMesh) return;

	// Check for dirty chunks
	if (bShouldRechunk)
	{
		TMap<FVoxelChunkNode*, TArray<FVoxelChunkNode*>> DirtyChunkGroups;
		if (RechunkToCenter(DirtyChunkGroups))
		{
			RebatchDirtyChunks(DirtyChunkGroups);
		}
	}

	// If no dirty chunks, no update needed
	if (!DirtyChunkDataMap.Num()) return;

	TArray<FVoxelChunkNode*> DirtyChunkNodes;
	DirtyChunkDataMap.GetKeys(DirtyChunkNodes);

	for (int idxNode = 0; idxNode < DirtyChunkNodes.Num(); idxNode++)
	{
		if (!bSynchronous && MeshBuildingTracker.GetValue() >= MeshBuildingLimit)
			return;

		FVoxelChunkNode* chunkNode = DirtyChunkNodes[idxNode];
		if (!chunkNode) continue;

		FVoxelDirtyChunkData* chunkData = DirtyChunkDataMap.FindRef(chunkNode);
		if (!chunkData) continue;

		if (!chunkData->tGeneration->IsDone())
		{
			if (!bSynchronous) continue; // if async, we wait until next update

			chunkData->tGeneration->EnsureCompletion();
		}

		if (!chunkData->bHasAnyVertices)
		{
			chunkData->CornerDensityValues.Empty();
			chunkData->StreamSet.Empty();
		}

		if (!chunkNode->SectionID && chunkData->bHasAnyVertices)
		{
			if (!NodeSectionIDTracker) NodeSectionIDTracker++;
			chunkNode->SectionID = NodeSectionIDTracker++;

			FName name = chunkNode->GetSectionName();
			const auto SectionGroupKey = FRealtimeMeshSectionGroupKey::Create(0, name);

			MeshBuildingTracker.Increment();

			//UE_LOG(LogTemp, Warning, TEXT("CreateSectionGroup Started (%s)"), *name.ToString());

			chunkData->fMeshUpdate = RealtimeMesh->CreateSectionGroup(SectionGroupKey, chunkData->StreamSet);

			chunkData->fMeshUpdate.Next
			(
				[this, name](ERealtimeMeshProxyUpdateStatus Status)
				{
					UE_LOG(LogTemp, Warning, TEXT("CreateSectionGroup Finished (SectionGroup_%s)"), *name.ToString());
					MeshBuildingTracker.Decrement();
				}
			);

			bool bShouldCreateCollision = MaxDepth - chunkNode->Depth + 1 <= CollisionInverseDepth;
			chunkData->fCollisionUpdate = RealtimeMesh->UpdateSectionConfig
			(
				FRealtimeMeshSectionKey::CreateForPolyGroup(SectionGroupKey, 0),
				FRealtimeMeshSectionConfig(ERealtimeMeshSectionDrawType::Dynamic, 0),
				bShouldCreateCollision
			);

			if (bSynchronous)
			{
				chunkData->fMeshUpdate.Wait();
				chunkData->fCollisionUpdate.Wait();
			}
		}

		if (!chunkData->bHasAnyVertices || (chunkNode->SectionID && chunkData->fCollisionUpdate.IsValid() && chunkData->fCollisionUpdate.IsReady()))
		{
			// Case 1:
			// 
			// Lower detail parent finished section, we can delete all its children now
			if (chunkNode == chunkData->BatchChunkKey)
			{
				auto children = chunkNode->GetChildren();
				for (auto child : children)
				{
					if (child->SectionID)
					{
						FName name = child->GetSectionName();
						const auto SectionGroupKey = FRealtimeMeshSectionGroupKey::Create(0, name);

						//UE_LOG(LogTemp, Warning, TEXT("RemoveSectionGroup Started (Parent Finished) (%s)"), *name.ToString());
						RealtimeMesh->RemoveSectionGroup(SectionGroupKey)
							.Next([name](ERealtimeMeshProxyUpdateStatus Status)
								{
									UE_LOG(LogTemp, Warning, TEXT("RemoveSectionGroup Finished (Parent Finished) (%s)"), *name.ToString());
								}
						);
					}
				}

				DirtyChunkBatches.Remove(chunkNode);
				DirtyChunkDataMap.Remove(chunkNode);

				for (uint8 i = 0; i < 8; i++)
				{
					delete chunkNode->Children[i];
					chunkNode->Children[i] = nullptr;
				}
			}
			// Case 2:
			// 
			// Higher detail child finished section, we can delete its parent now if all its siblings are done
			else
			{
				if (auto arrayRef = DirtyChunkBatches.Find(chunkData->BatchChunkKey))
				{
					arrayRef->Remove(chunkNode);
					DirtyChunkDataMap.Remove(chunkNode);

					if (arrayRef->IsEmpty())
					{
						FName name = chunkData->BatchChunkKey->GetSectionName();
						const auto SectionGroupKey = FRealtimeMeshSectionGroupKey::Create(0, name);
						chunkData->BatchChunkKey->SectionID = 0;

						//UE_LOG(LogTemp, Warning, TEXT("RemoveSectionGroup Started (Children Finished) (%s)"), *name.ToString());
						RealtimeMesh->RemoveSectionGroup(SectionGroupKey)
							.Next([name](ERealtimeMeshProxyUpdateStatus Status)
								{
									UE_LOG(LogTemp, Warning, TEXT("RemoveSectionGroup Finished (Children Finished) (%s)"), *name.ToString());
								}
						);

						DirtyChunkBatches.Remove(chunkData->BatchChunkKey);
					}
				}
			}
		}
	}
}