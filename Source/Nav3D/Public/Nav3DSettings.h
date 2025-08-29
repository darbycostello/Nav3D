// Nav3DSettings.h
#pragma once

#include "CoreMinimal.h"
#include "Engine/DeveloperSettings.h"
#include "Pathfinding/Nav3DQueryFilterSettings.h"
#include "Nav3DSettings.generated.h"

UCLASS(config=Engine, defaultconfig, meta=(DisplayName="Nav3D Settings"))
class NAV3D_API UNav3DSettings : public UDeveloperSettings
{
	GENERATED_BODY()
    
public:
	UNav3DSettings();
    
	UPROPERTY(EditAnywhere, config, Category="Pathfinding")
	TSubclassOf<UNav3DPathFindingSearch> DefaultPathFinder;
    
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
    
	// Smoothing subdivisions
	UPROPERTY(EditAnywhere, config, Category="Pathfinding")
	int32 SmoothingSubdivisions;

	// Used to prevent regioning from crashing the editor during region building.
	UPROPERTY(EditAnywhere, config, Category="Tactical Reasoning")
	int32 MaxRegions;
    
	// Get the singleton instance
	static const UNav3DSettings* Get();
    
	// Get default query filter settings with class defaults
	FNav3DQueryFilterSettings GetDefaultQueryFilterSettings() const;
};