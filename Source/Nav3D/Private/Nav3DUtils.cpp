#include "Nav3DUtils.h"
#include "GameFramework/NavMovementComponent.h"
#include "ThirdParty/libmorton/morton.h"

MortonCode FNav3DUtils::GetMortonCodeFromVector(const FVector& Vector)
{
	return morton3D_64_encode(Vector.X, Vector.Y, Vector.Z);
}

MortonCode FNav3DUtils::GetMortonCodeFromVector(const FIntVector& Vector)
{
	return morton3D_64_encode(Vector.X, Vector.Y, Vector.Z);
}

FVector FNav3DUtils::GetVectorFromMortonCode(const MortonCode MortonCode)
{
	uint_fast32_t X, Y, Z;
	morton3D_64_decode(MortonCode, X, Y, Z);

	return FVector(X, Y, Z);
}

MortonCode FNav3DUtils::GetParentMortonCode(const MortonCode ChildMortonCode)
{
	return ChildMortonCode >> 3;
}

MortonCode FNav3DUtils::GetFirstChildMortonCode(const MortonCode ParentMortonCode)
{
	return ParentMortonCode << 3;
}

FVector FNav3DUtils::GetSubNodeOffset(SubNodeIndex SubIdx, float NodeExtent)
{
	// Convert morton index to 3D coordinates
	uint_fast32_t X, Y, Z;
	morton3D_64_decode(SubIdx, X, Y, Z);

	const float SubNodeSize = NodeExtent * 0.5f;
	return FVector(
		(X * SubNodeSize) - NodeExtent,
		(Y * SubNodeSize) - NodeExtent,
		(Z * SubNodeSize) - NodeExtent
	);
}

ENavigationQueryResult::Type
FNav3DUtils::GraphAStarResultToNavigationTypeResult(
	const EGraphAStarResult Result)
{
	constexpr ENavigationQueryResult::Type ResultConversionTable[] = {
		ENavigationQueryResult::Fail, ENavigationQueryResult::Success,
		ENavigationQueryResult::Fail, ENavigationQueryResult::Fail
	};

	return ResultConversionTable[static_cast<int>(Result)];
}

bool FNav3DUtils::RayBoxIntersection(const FBox& Box, const FVector& RayOrigin, const FVector& RayDir,
                                     const float RayLength, float& OutTMin, float& OutTMax)
{
	// Calculate inverse ray direction for efficient tests
	const FVector InvDir(
		FMath::IsNearlyZero(RayDir.X) ? BIG_NUMBER : 1.0f / RayDir.X,
		FMath::IsNearlyZero(RayDir.Y) ? BIG_NUMBER : 1.0f / RayDir.Y,
		FMath::IsNearlyZero(RayDir.Z) ? BIG_NUMBER : 1.0f / RayDir.Z
	);

	// Calculate intersections with axis-aligned planes
	float TMin = -BIG_NUMBER;
	float TMax = BIG_NUMBER;

	for (int32 i = 0; i < 3; i++)
	{
		const float RayOrig = i == 0 ? RayOrigin.X : (i == 1 ? RayOrigin.Y : RayOrigin.Z);
		const float InvRayDir = i == 0 ? InvDir.X : (i == 1 ? InvDir.Y : InvDir.Z);
		const float BoxMin = i == 0 ? Box.Min.X : (i == 1 ? Box.Min.Y : Box.Min.Z);
		const float BoxMax = i == 0 ? Box.Max.X : (i == 1 ? Box.Max.Y : Box.Max.Z);

		if (FMath::Abs(InvRayDir) < SMALL_NUMBER)
		{
			// Ray parallel to axis - check if ray origin is within box planes
			if (RayOrig < BoxMin || RayOrig > BoxMax)
			{
				return false;
			}
		}
		else
		{
			float T1 = (BoxMin - RayOrig) * InvRayDir;
			float T2 = (BoxMax - RayOrig) * InvRayDir;

			if (T1 > T2)
			{
				const float Temp = T1;
				T1 = T2;
				T2 = Temp;
			}

			TMin = FMath::Max(T1, TMin);
			TMax = FMath::Min(T2, TMax);

			if (TMin > TMax || TMax < 0.0f)
			{
				return false;
			}
		}
	}

	// Check if intersection is within ray length
	if (TMin > RayLength)
	{
		return false;
	}

	TMax = FMath::Min(TMax, RayLength);

	OutTMin = TMin;
	OutTMax = TMax;
	return true;
}

FNavAgentProperties FNav3DUtils::GetNavAgentPropsFromQuerier(const UObject* Querier)
{
	if (const AActor* Actor = Cast<AActor>(Querier))
	{
		if (const UNavMovementComponent* MoveComp = Actor->FindComponentByClass<UNavMovementComponent>())
		{
			return MoveComp->GetNavAgentPropertiesRef();
		}
	}
	return FNavAgentProperties::DefaultProperties;
}