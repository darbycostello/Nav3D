#pragma once

#include "CoreMinimal.h"
#include "Pathfinding/Core/INav3DPathfinder.h"
#include "Nav3DVolumeNavigationData.h"

// ReSharper disable once CppUE4CodingStandardNamingViolationWarning
class NAV3D_API FNav3DAStar : public INav3DPathfinder
{
public:
	FNav3DAStar();
	virtual ~FNav3DAStar() override;

	virtual ENavigationQueryResult::Type FindPath(
		FNav3DPath& OutPath,
		const FNav3DPathingRequest& Request,
		const FNav3DVolumeNavigationData* VolumeNavData) override;

protected:
	struct FSearchNode
	{
		FNav3DNodeAddress Address;
		float GScore = 0.0f;
		float FScore = 0.0f;
		FNav3DNodeAddress Parent;
		bool bInOpenSet = false;
		bool bInClosedSet = false;
	};

	void InitializeSearch(const FNav3DPathingRequest& Request, const FNav3DVolumeNavigationData* VolumeNavData);
	void ProcessCurrentNode(const FSearchNode& CurrentNode);
	void ProcessNeighbor(const FNav3DNodeAddress& NeighborAddress, const FSearchNode& CurrentNode);
	ENavigationQueryResult::Type ReconstructPath(FNav3DPath& OutPath, const FSearchNode& GoalNode);

	float CalculateHeuristic(const FNav3DNodeAddress& From, const FNav3DNodeAddress& To) const;
	float CalculateDistance(const FNav3DNodeAddress& From, const FNav3DNodeAddress& To) const;
	float AdjustTotalCostWithNodeSizeCompensation(float TotalCost, const FNav3DNodeAddress& NodeAddress) const;
	static void LogSearchProgress(int32 Iteration, int32 OpenSetSize, float BestFScore);

	TMap<FNav3DNodeAddress, FSearchNode> AllNodes;
	TArray<FNav3DNodeAddress> OpenSet;
	FNav3DNodeAddress StartAddress;
	FNav3DNodeAddress GoalAddress;
	const FNav3DVolumeNavigationData* VolumeData = nullptr;
	ENav3DPathingLogVerbosity LogVerbosity = ENav3DPathingLogVerbosity::Standard;

	// Store original request for endpoint handling and calculators
	FNav3DPathingRequest CurrentRequest;

};


