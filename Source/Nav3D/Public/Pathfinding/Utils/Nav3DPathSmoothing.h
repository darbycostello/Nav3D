#pragma once
#include "Pathfinding/Core/Nav3DPath.h"

class NAV3D_API FNav3DPathSmoothing
{
public:
	static void SmoothPath(FNav3DPath& Path, int32 Subdivisions);

private:
	static float GetT(float T, float Alpha, const FVector& P0, const FVector& P1);

	static FVector GetPoint(const FVector& P0, const FVector& P1, const FVector& P2,
	                        const FVector& P3, float T, float Alpha = 0.5f);
};
