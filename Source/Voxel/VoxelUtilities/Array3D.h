

#pragma once

#include "CoreMinimal.h"


template<typename InElementType>
struct FArray3D
{
	TArray<InElementType> InternalArray;
	FIntVector Size3D;
	int32 SizeTotal;

	FArray3D() {};

	~FArray3D()
	{
		InternalArray.Empty();
	};

	void Init(int32 InSizeX, int32 InSizeY, int32 InSizeZ, const InElementType& InitValue = InElementType())
	{
		Size3D = FIntVector(InSizeX, InSizeY, InSizeZ);
		SizeTotal = Size3D.X * Size3D.Y * Size3D.Z;
		InternalArray.Init(InitValue, SizeTotal);
	}

	void Init(const FIntVector& InSize3D, const InElementType& InitValue = InElementType())
	{
		Size3D = InSize3D;
		SizeTotal = Size3D.X * Size3D.Y * Size3D.Z;
		InternalArray.Init(InitValue, SizeTotal);
	}

	void Reset(const InElementType& InitValue = InElementType())
	{
		InternalArray.Init(InitValue, SizeTotal);
	}

	FArray3D(int32 InSizeX, int32 InSizeY, int32 InSizeZ, const InElementType& InitValue = InElementType())
	{
		Init(InSizeX, InSizeY, InSizeZ, InitValue);
	}

	FArray3D(const FIntVector& InSize3D, const InElementType& InitValue = InElementType())
	{
		Init(InSize3D, InitValue);
	}

	FORCEINLINE InElementType& operator[](int32 Index)
	{
		return InternalArray[Index];
	}

	FORCEINLINE const InElementType& operator[](int32 Index) const
	{
		return InternalArray[Index];
	}

	FORCEINLINE InElementType& operator[](const FIntVector& Index3D)
	{
		return InternalArray[GetIndex1D(Index3D)];
	}

	FORCEINLINE const InElementType& operator[](const FIntVector& Index3D) const
	{
		return InternalArray[GetIndex1D(Index3D)];
	}

	const int32 GetIndex1D(int32 InX, int32 InY, int32 InZ) const
	{
		return InX * Size3D.Z * Size3D.Y + (InY * Size3D.Z + InZ);
	}

	const int32 GetIndex1D(const FIntVector& InIndex3D) const
	{
		return InIndex3D.X * Size3D.Z * Size3D.Y + (InIndex3D.Y * Size3D.Z + InIndex3D.Z);
	}

	FIntVector GetIndex3D(int32 InIndex)
	{
		int32 yz = (Size3D.Y * Size3D.Z);
		int32 x = InIndex / yz;
		int32 w = InIndex % yz;
		int32 y = w / Size3D.Z;
		int32 z = w % Size3D.Z;

		return FIntVector(x, y, z);
	}

	const int32 GetSizeTotal() const { return SizeTotal; };
	const int32 GetSizeX() const { return Size3D.X; };
	const int32 GetSizeY() const { return Size3D.Y; };
	const int32 GetSizeZ() const { return Size3D.Z; };
	const FIntVector GetSize3D() const { return Size3D; };

	void Empty()
	{
		// keep for repopulation
		//Size3D = FIntVector::ZeroValue;
		//SizeTotal = 0;
		InternalArray.Empty();
	}
};