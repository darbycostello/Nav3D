#pragma once

#include <NavigationPath.h>
#include "Nav3DPath.generated.h"

/**
 * Blueprint-compatible navigation path point.
 * Contains position and navigation node reference for a point along a path.
 */
USTRUCT(BlueprintType)
struct NAV3D_API FNav3DPathPoint
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly, Category="Nav3D")
	FVector Location;

	UPROPERTY(BlueprintReadOnly, Category="Nav3D")
	int32 NodeRef;

	FNav3DPathPoint()
		: Location(FVector::ZeroVector)
		  , NodeRef(INVALID_NAVNODEREF)
	{
	}

	FNav3DPathPoint(const FVector& InLocation, const int32 InNodeRef)
		: Location(InLocation)
		  , NodeRef(InNodeRef)
	{
	}

	explicit FNav3DPathPoint(const FNavPathPoint& PathPoint)
		: Location(PathPoint.Location)
		  , NodeRef(PathPoint.NodeRef)
	{
	}
};

// Blueprint-compatible wrapper struct for path data
USTRUCT(BlueprintType)
struct NAV3D_API FNav3DPathData
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly, Category="Nav3D")
	TArray<FNav3DPathPoint> PathPoints;

	UPROPERTY(BlueprintReadOnly, Category="Nav3D")
	TArray<float> PathPointCosts;

	UPROPERTY(BlueprintReadOnly, Category="Nav3D")
	bool bIsValid = false;

	UPROPERTY(BlueprintReadOnly, Category="Nav3D")
	bool bIsPartial = false;
};

class NAV3D_API FNav3DPath final : public FNavigationPath
{
	using Super = FNavigationPath;

public:
	FNav3DPath() = default;

	TArray<float>& GetPathPointCosts() { return PathPointCosts; }
	const TArray<float>& GetPathPointCosts() const { return PathPointCosts; }

	virtual FVector::FReal GetCostFromNode(NavNodeRef PathNode) const override;
	virtual FVector::FReal GetCostFromIndex(int32 PathPointIndex) const override;

	// Create blueprint-compatible data
	FNav3DPathData CreatePathData() const
	{
		FNav3DPathData Data;
		Data.PathPoints.Reserve(GetPathPoints().Num());

		// Convert each path point
		for (const FNavPathPoint& Point : GetPathPoints())
		{
			Data.PathPoints.Add(FNav3DPathPoint(Point));
		}

		Data.PathPointCosts = PathPointCosts;
		Data.bIsValid = IsValid();
		Data.bIsPartial = IsPartial();
		return Data;
	}

private:
	TArray<float> PathPointCosts;
};
