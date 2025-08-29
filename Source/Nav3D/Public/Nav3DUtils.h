#pragma once

#include "Nav3DTypes.h"
#include <CoreMinimal.h>
#include <GraphAStar.h>

class NAV3D_API FNav3DUtils
{
public:
	static MortonCode GetMortonCodeFromVector(const FVector& Vector);
	static MortonCode GetMortonCodeFromVector(const FIntVector& Vector);
	static FVector GetVectorFromMortonCode(const MortonCode MortonCode);
	static MortonCode GetParentMortonCode(const MortonCode ChildMortonCode);
	static MortonCode GetFirstChildMortonCode(const MortonCode ParentMortonCode);
	static FVector GetSubNodeOffset(SubNodeIndex SubIdx, float NodeExtent);
	static ENavigationQueryResult::Type GraphAStarResultToNavigationTypeResult(const EGraphAStarResult Result);
	static bool RayBoxIntersection(const FBox& Box, const FVector& RayOrigin, const FVector& RayDir, float RayLength,
	                               float& OutTMin, float& OutTMax);
	static FNavAgentProperties GetNavAgentPropsFromQuerier(const UObject* Querier);
};
