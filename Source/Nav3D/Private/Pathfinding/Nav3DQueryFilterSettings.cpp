#include "Pathfinding/Nav3DQueryFilterSettings.h"

FNav3DQueryFilterSettings::FNav3DQueryFilterSettings()
	: PathFinder(nullptr), TraversalCostCalculator(nullptr),
	  HeuristicCalculator(nullptr), HeuristicScale(1.0f),
	  bUseNodeSizeCompensation(true), bSmoothPaths(true),
	  SmoothingSubdivisions(10)
{
}
