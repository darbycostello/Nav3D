#pragma once

#include "Nav3DPath.h"
#include "Nav3DTypes.h"

#include "Nav3DPathFindingTypes.generated.h"

struct FNav3DQueryFilterSettings;
class FNav3DQueryFilter;

struct FNav3DNodeAddressWithLocation
{
	FNav3DNodeAddressWithLocation() = default;
	FNav3DNodeAddressWithLocation(const FNav3DNodeAddress& NodeAddress,
	                              const FVector& Location);
	FNav3DNodeAddressWithLocation(
		const FNav3DNodeAddress& NodeAddress,
		const FNav3DVolumeNavigationData& BoundsNavigationData);

	bool operator==(const FNav3DNodeAddressWithLocation& Other) const
	{
		return NodeAddress == Other.NodeAddress;
	}

	bool operator!=(const FNav3DNodeAddressWithLocation& Other) const
	{
		return !operator==(Other);
	}

	FNav3DNodeAddress NodeAddress;
	FVector Location;
};

FORCEINLINE uint32
GetTypeHash(const FNav3DNodeAddressWithLocation& NodeAddressWithLocation)
{
	return GetTypeHash(NodeAddressWithLocation.NodeAddress);
}

struct FNav3DPathFinderDebugNodeCost
{
	FNav3DPathFinderDebugNodeCost() = default;
	FNav3DPathFinderDebugNodeCost(const FNav3DNodeAddressWithLocation& From,
	                              const FNav3DNodeAddressWithLocation& To,
	                              const float Cost, const bool IsClosed);

	void Reset();

	FNav3DNodeAddressWithLocation From;
	FNav3DNodeAddressWithLocation To;
	double Cost;
	uint8 bIsClosed : 1;
};

struct FNav3DPathFinderNodeAddress
{
	FNav3DPathFinderNodeAddress() = default;
	FNav3DPathFinderNodeAddress(const FNav3DNodeAddress& NodeAddress,
	                            double TraversalCost);

	FNav3DNodeAddress NodeAddress;
	double Cost;
};

USTRUCT()
struct NAV3D_API FNav3DPathFinderDebugData
{
	GENERATED_BODY()

	FNav3DPathFinderDebugData();

	void Reset();

	FNav3DPathFinderDebugNodeCost LastProcessedSingleNode;
	TArray<FNav3DPathFinderDebugNodeCost> ProcessedNeighbours;

	FNav3DPath CurrentBestPath;

	UPROPERTY(VisibleAnywhere)
	int Iterations;

	UPROPERTY(VisibleAnywhere)
	int VisitedNodes;

	UPROPERTY(VisibleAnywhere)
	int PathSegmentCount;

	UPROPERTY(VisibleAnywhere)
	float PathLength;

	UPROPERTY(VisibleAnywhere)
	FString StartNodeAddress;

	UPROPERTY(VisibleAnywhere)
	FString EndNodeAddress;
};

struct FNav3DPathFindingParameters
{
	static TOptional<FNav3DPathFindingParameters>
	Initialize(const FNav3DVolumeNavigationData& VolumeNavigationData,
	           const FVector& StartLocation, const FVector& EndLocation,
	           const FNavigationQueryFilter& NavQueryFilter, const LayerIndex& MinLayerIndex);

	FVector StartLocation;
	FVector EndLocation;
	const FNavigationQueryFilter& NavigationQueryFilter;
	const FNav3DQueryFilter* QueryFilterImplementation;
	const FNav3DQueryFilterSettings& QueryFilterSettings;
	const UNav3DPathHeuristicCalculator* HeuristicCalculator;
	const UNav3DPathTraversalCostCalculator* CostCalculator;
	const FNav3DVolumeNavigationData& VolumeNavigationData;
	FNav3DNodeAddress StartNodeAddress;
	FNav3DNodeAddress EndNodeAddress;

private:
	FNav3DPathFindingParameters(
		const FNav3DVolumeNavigationData& VolumeNavigationData,
		const FVector& StartLocation, const FVector& EndLocation,
		const FNavigationQueryFilter& NavQueryFilter);
};
