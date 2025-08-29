#pragma once

#include "Nav3DQueryFilterSettings.generated.h"

class UNav3DPathHeuristicCalculator;
class UNav3DPathTraversalCostCalculator;
class UNav3DPathFindingSearch;

USTRUCT()
struct NAV3D_API FNav3DQueryFilterSettings
{
	GENERATED_BODY()

	FNav3DQueryFilterSettings();

	UPROPERTY(EditAnywhere, Instanced)
	UNav3DPathFindingSearch* PathFinder;

	UPROPERTY(EditAnywhere, Instanced)
	UNav3DPathTraversalCostCalculator* TraversalCostCalculator;

	UPROPERTY(EditAnywhere, Instanced)
	UNav3DPathHeuristicCalculator* HeuristicCalculator;

	UPROPERTY(EditDefaultsOnly)
	float HeuristicScale;

	// If set to true, this will lower the cost of traversing bigger nodes, and
	// make the PathFinding more favorable traversing them
	UPROPERTY(EditDefaultsOnly)
	uint8 bUseNodeSizeCompensation : 1;

	UPROPERTY(EditDefaultsOnly)
	uint8 bSmoothPaths : 1;

	// How many intermediate points we will generate between the points returned
	// by the PathFinding in order to smooth the curve (the bigger, the smoother)
	UPROPERTY(EditDefaultsOnly, meta = (EditCondition = "bSmoothPaths == true"))
	int SmoothingSubdivisions;
};
