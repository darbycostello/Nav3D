#include "Pathfinding/Nav3DPathBuilder.h"
#include "Pathfinding/Stepper/Nav3DPathStepper.h"
#include "Pathfinding/Nav3DPathFinder.h"
#include "Pathfinding/Search/Nav3DPathFindingSearch.h"
#include "Pathfinding/Nav3DPathSmoothing.h"
#include "Pathfinding/Nav3DQueryFilterSettings.h"

FNav3DPathBuilder::FNav3DPathBuilder(
	FNav3DPath& NavigationPath,
	const FNav3DPathStepper& Stepper)
	: FNav3DPathFindingProcessor(Stepper)
	  , NavigationPath(NavigationPath)
{
}

void FNav3DPathBuilder::ProcessFinalPath(
	const TArray<FNav3DPathFinderNodeAddress>& NodeAddresses)
{
	const auto& Params = Stepper.GetParameters();
	FNav3DPathFinder::BuildPath(NavigationPath, Params, NodeAddresses, true);

	if (Params.QueryFilterSettings.bSmoothPaths)
	{
		FNav3DPathSmoothing::SmoothPath(
			NavigationPath,
			Params.QueryFilterSettings.SmoothingSubdivisions);
	}

	NavigationPath.MarkReady();
}
