

#pragma once

#include "CoreMinimal.h"

#include "Noise.generated.h"

UENUM()
enum ENoiseType : uint8
{
	Perlin,
	Simplex,
	Cellular
};

USTRUCT(BlueprintType)
struct FNoiseSettings
{
	GENERATED_BODY()

public:

	UPROPERTY(BlueprintReadOnly, EditAnywhere)
	TEnumAsByte<ENoiseType> Type;

	UPROPERTY(BlueprintReadOnly, EditAnywhere)
	double Frequency;

	UPROPERTY(BlueprintReadOnly, EditAnywhere)
	double Amplitude;

	UPROPERTY(BlueprintReadOnly, EditAnywhere)
	int OctaveCount;
};

struct FNoise
{
public:
	static double ComputeNoise(const FVector& InLocation, const FNoiseSettings& InNoiseSettings);
	static double ComputeNoise(const FVector& InLocation, ENoiseType InNoiseType);
	static double ComputeNoiseWithOctaves(const FVector& InLocation, ENoiseType InNoiseType, int32 InOctaveCount);
};