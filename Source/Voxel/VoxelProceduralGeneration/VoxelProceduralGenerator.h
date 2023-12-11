// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "SignedDistanceField.h"

#include "VoxelProceduralGenerator.generated.h"

UENUM()
enum EVoxelNoiseType : uint8
{
    VN_Perlin
};

namespace VoxelNoise
{

    static double ComputeNoise3D(
        const FVector& InLocation,
        EVoxelNoiseType InNoiseType,
        double InAmplitude = 1.0,
        double InFrequency = 1.0,
        int InOctaves = 1
    )
    {
        double noise = 0.0;
        double persistence = 0.5;
        double lacunarity = 2.0;
        double totalAmpl = 0.0;

        for (int i = 0; i < InOctaves; i++)
        {
            switch (InNoiseType)
            {
            case EVoxelNoiseType::VN_Perlin:
                noise += ((FMath::PerlinNoise3D(InLocation * InFrequency) + 1.0) / 2.0) * InAmplitude;
                break;

            default:
                break;
            };

            totalAmpl += InAmplitude;
            InAmplitude *= persistence;
            InFrequency *= lacunarity;
        }

        return noise;
    }
}

USTRUCT(BlueprintType)
struct FBiomeMaterialData
{
    GENERATED_BODY()
public:
    UPROPERTY(BlueprintReadOnly, EditDefaultsOnly)
    double Isovalue;

    UPROPERTY(BlueprintReadOnly, EditDefaultsOnly)
    TSoftObjectPtr<UMaterialInterface> Material;
};

UCLASS(DefaultToInstanced, EditInlineNew, Abstract)
class VOXEL_API UVoxelProcGen_ValueGenerator : public UObject
{
    GENERATED_BODY()

public:

    virtual float GenerateValue(const FVector& InLocation, const double InVolumeExtent, const FVector& InCenter = FVector::ZeroVector, double Seed = 0.0) const { return 0.f; };
};

UCLASS()
class UVoxelProcGen_SdfSphere : public UVoxelProcGen_ValueGenerator
{
    GENERATED_BODY()

public:
    UPROPERTY(EditDefaultsOnly)
    double RadiusNormalized = 1.0;

    virtual float GenerateValue(const FVector& InLocation, const double InVolumeExtent, const FVector& InCenter = FVector::ZeroVector, double Seed = 0.0) const override
    {
        double desiredRadius = InVolumeExtent * RadiusNormalized;
        return SignedDistanceField::GetDistanceSphere(InCenter.IsZero() ? InLocation : InLocation - InCenter, desiredRadius);
    }
};

UCLASS()
class UVoxelProcGen_Noise : public UVoxelProcGen_ValueGenerator
{
    GENERATED_BODY()

public:
    UPROPERTY(EditDefaultsOnly)
    TEnumAsByte<EVoxelNoiseType> Type = EVoxelNoiseType::VN_Perlin;

    UPROPERTY(EditDefaultsOnly)
    double Amplitude = 1.0;

    UPROPERTY(EditDefaultsOnly)
    double Frequency = 1.0;

    UPROPERTY(EditDefaultsOnly)
    int Octaves = 1;

    // todo seed?
    virtual float GenerateValue(const FVector& InLocation, const double InVolumeExtent, const FVector& InCenter = FVector::ZeroVector, double Seed = 0.0) const override
    {
        FVector locationRelative = InCenter.IsZero() ? InLocation : InLocation - InCenter;
        return VoxelNoise::ComputeNoise3D(locationRelative, Type, Amplitude, Frequency, Octaves);
    }
};

/**
 * 
 */
UCLASS(Blueprintable)
class VOXEL_API UVoxelProceduralGenerator : public UObject
{
    GENERATED_BODY()

protected:

    UVoxelProceduralGenerator() {};

public:

    double GenerateProceduralValue(const FVector& InLocation, const double InVolumeExtent, const FVector& InCenter = FVector::ZeroVector, double Seed = 0.0)
    {
        double value = 0.0;
        for (const TObjectPtr<UVoxelProcGen_ValueGenerator>& gen : ValueGenerators)
        {
            value += gen->GenerateValue(InLocation, InVolumeExtent, InCenter, Seed);
        }

        return value;
    }
    
    // Additive calculations of density per location
    UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Instanced)
    TArray<TObjectPtr<UVoxelProcGen_ValueGenerator>> ValueGenerators;

    // Todo
    UPROPERTY(BlueprintReadOnly, EditDefaultsOnly)
    TArray<FBiomeMaterialData> BiomeMaterialData;
};