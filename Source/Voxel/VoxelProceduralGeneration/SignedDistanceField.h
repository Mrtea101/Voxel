// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"

namespace SignedDistanceField
{
	static double GetDistanceSphere(const FVector& InLocation, double InRadius)
	{
		return InLocation.Length() / InRadius;
	};

	// todo center
	static double GetDistanceBox(const FVector& InLocation, const FVector& InSize)
	{
		FVector b = InSize / 2.0;
		FVector q = FVector::Max(InLocation.GetAbs() - b, FVector::ZeroVector);

		return q.Length() + FMath::Min(FMath::Max(q.X, FMath::Max(q.Y, q.Z)), 0.0);
	};
};