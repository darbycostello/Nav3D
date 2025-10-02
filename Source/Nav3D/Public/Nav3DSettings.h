#pragma once

#include "CoreMinimal.h"
#include "Engine/DeveloperSettings.h"
#include "Pathfinding/Core/Nav3DPathingTypes.h"
#include "Nav3DSettings.generated.h"

UCLASS(config=Engine, defaultconfig, meta=(DisplayName="Nav3D Settings"))
class NAV3D_API UNav3DSettings : public UDeveloperSettings
{
	GENERATED_BODY()

public:
	UNav3DSettings();

    // Default algorithm selection (enum-based)
    UPROPERTY(EditAnywhere, config, Category="Pathfinding")
    ENav3DPathingAlgorithm DefaultAlgorithm = ENav3DPathingAlgorithm::LazyThetaStar;

	// Default traversal cost calculator class
	UPROPERTY(EditAnywhere, config, Category="Pathfinding")
	TSubclassOf<UNav3DPathTraversalCostCalculator> DefaultCostCalculator;

	// Default heuristic calculator class
	UPROPERTY(EditAnywhere, config, Category="Pathfinding")
	TSubclassOf<UNav3DPathHeuristicCalculator> DefaultHeuristic;

	// Heuristic scale
	UPROPERTY(EditAnywhere, config, Category="Pathfinding")
	float HeuristicScale;

	// Use node size compensation
	UPROPERTY(EditAnywhere, config, Category="Pathfinding")
	bool bUseNodeSizeCompensation;

	// Smooth paths
	UPROPERTY(EditAnywhere, config, Category="Pathfinding")
	bool bSmoothPaths;

	// Prune paths using direct traversal between waypoints before smoothing
	UPROPERTY(EditAnywhere, config, Category="Pathfinding")
	bool bPrunePaths = false;

	// Smoothing subdivisions
	UPROPERTY(EditAnywhere, config, Category="Pathfinding")
	int32 SmoothingSubdivisions;

	// Used to prevent regioning from crashing the editor during region building.
	UPROPERTY(EditAnywhere, config, Category="Tactical Reasoning")
	int32 MaxRegions;

	UPROPERTY(EditAnywhere, config, Category="Adjacency")
	bool bEnableAdjacencyOptimization = true;

	// 0 = use voxel size as adjacency threshold
	UPROPERTY(EditAnywhere, config, Category="Adjacency", meta=(ClampMin="0"))
	float Nav3DChunkAdjacencyThreshold = 0.0f;

	// Universal Volume Partitioning
	UPROPERTY(EditAnywhere, config, Category="Volume Partitioning")
	bool bEnableAutomaticVolumePartitioning = true;
	
	UPROPERTY(EditAnywhere, config, Category="Volume Partitioning",
	          meta=(EditCondition="bEnableAutomaticVolumePartitioning", ClampMin="50000", ClampMax="1000000"))
	float MaxVolumePartitionSize = 250000.0f; // 2.5km
	
	UPROPERTY(EditAnywhere, config, Category="Volume Partitioning",
	          meta=(EditCondition="bEnableAutomaticVolumePartitioning", ClampMin="1", ClampMax="16"))
	int32 MaxSubVolumesPerAxis = 8;
	
	UPROPERTY(EditAnywhere, config, Category="Volume Partitioning")
	bool bPreferCubePartitions = true; // Prefer cubic sub-volumes over elongated ones
	
	// Performance
	UPROPERTY(EditAnywhere, config, Category="Performance")
	int32 MaxParallelVolumeBuilds = 4; // How many volumes to build simultaneously

	// Get the singleton instance
	static const UNav3DSettings* Get();

	// Get default query filter settings with class defaults
	FNav3DQueryFilterSettings GetDefaultQueryFilterSettings() const;
};


