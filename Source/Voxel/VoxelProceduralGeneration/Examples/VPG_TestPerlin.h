// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "VoxelProceduralGeneration/VoxelProceduralGenerator.h"
#include "VPG_TestPerlin.generated.h"

/**
 * 
 */
UCLASS()
class VOXEL_API UVPG_TestPerlin : public UVoxelProceduralGenerator
{
	GENERATED_BODY()
	
    UVPG_TestPerlin()
	{
		UVoxelProcGen_SdfSphere* sdf =
			CreateDefaultSubobject<UVoxelProcGen_SdfSphere>(MakeUniqueObjectName(this, UVoxelProcGen_SdfSphere::StaticClass()));

		sdf->RadiusNormalized = 0.9;

		UVoxelProcGen_Noise* noise =
			CreateDefaultSubobject<UVoxelProcGen_Noise>(MakeUniqueObjectName(this, UVoxelProcGen_Noise::StaticClass()));

		noise->Type = VN_Perlin;
		noise->Amplitude = 0.1;
		noise->Frequency = 0.00005;
		noise->Octaves = 1;

		ValueGenerators.Add(sdf);
		ValueGenerators.Add(noise);
	};
};
