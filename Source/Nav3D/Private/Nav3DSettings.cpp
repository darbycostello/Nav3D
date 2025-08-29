#include "Nav3DSettings.h"
#include "Pathfinding/Search/Nav3DPathFindingSearch.h"
#include "Pathfinding/Search/Nav3DLazyThetaStar.h"
#include "Pathfinding/Search/Nav3DPathTraversalCostCalculator.h"
#include "Pathfinding/Search/Nav3DPathHeuristicCalculator.h"

UNav3DSettings::UNav3DSettings()
{
	// Set default class selections
	DefaultPathFinder = UNav3DLazyThetaStar::StaticClass();
	DefaultCostCalculator = UNav3DPathCostCalculator_Distance::StaticClass();
	DefaultHeuristic = UNav3DPathHeuristicCalculator_Euclidean::StaticClass();
    
	// Default values
	HeuristicScale = 1.0f;
	bUseNodeSizeCompensation = true;
	bSmoothPaths = true;
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
    
	// Create default instances of the configured classes
	if (DefaultPathFinder)
	{
		Settings.PathFinder = DefaultPathFinder->GetDefaultObject<UNav3DPathFindingSearch>();
	}
    
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