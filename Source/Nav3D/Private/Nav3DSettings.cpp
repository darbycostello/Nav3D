#include "Nav3DSettings.h"
#include "Pathfinding/Search/Nav3DPathTraversalCostCalculator.h"
#include "Pathfinding/Search/Nav3DPathHeuristicCalculator.h"

UNav3DSettings::UNav3DSettings()
{
    // Set default selections
    DefaultAlgorithm = ENav3DPathingAlgorithm::LazyThetaStar;
	DefaultCostCalculator = UNav3DPathCostCalculator_Distance::StaticClass();
	DefaultHeuristic = UNav3DPathHeuristicCalculator_Euclidean::StaticClass();
    
	// Default values
	HeuristicScale = 1.0f;
	bUseNodeSizeCompensation = true;
	bSmoothPaths = true;
	bPrunePaths = false;
	SmoothingSubdivisions = 10;
	MaxRegions = 10000;
}

const UNav3DSettings* UNav3DSettings::Get()
{
	return GetDefault<UNav3DSettings>();
}

FNav3DQueryFilterSettings UNav3DSettings::GetDefaultQueryFilterSettings() const
{
	FNav3DQueryFilterSettings Settings;
    // PathFinder instance removed; algorithm now chosen via enum elsewhere
    
	if (DefaultCostCalculator)
	{
		Settings.TraversalCostCalculator = DefaultCostCalculator->GetDefaultObject<UNav3DPathTraversalCostCalculator>();
	}
    
	if (DefaultHeuristic)
	{
		Settings.HeuristicCalculator = DefaultHeuristic->GetDefaultObject<UNav3DPathHeuristicCalculator>();
	}
    
	// Copy other settings
	Settings.HeuristicScale = HeuristicScale;
	Settings.bUseNodeSizeCompensation = bUseNodeSizeCompensation;
	Settings.bSmoothPaths = bSmoothPaths;
	Settings.SmoothingSubdivisions = SmoothingSubdivisions;

	return Settings;
}