#include "Pathfinding/Core/Nav3DPathCoordinator.h"
#include "Nav3DSettings.h"
#include "Pathfinding/Core/Nav3DPath.h"
#include "Pathfinding/Search/Nav3DPathHeuristicCalculator.h"
#include "Pathfinding/Search/Nav3DPathTraversalCostCalculator.h"
#include "Pathfinding/Core/Nav3DVolumePathfinder.h"
#include "Pathfinding/Search/Nav3DAStar.h"
#include "Pathfinding/Search/Nav3DThetaStar.h"
#include "Pathfinding/Search/Nav3DLazyThetaStar.h"

TUniquePtr<FNav3DPathCoordinator> FNav3DPathCoordinator::Instance;

FNav3DPathCoordinator::FNav3DPathCoordinator()
{
	VolumeManager = MakeUnique<FNav3DVolumePathfinder>();
	AStarSolver = MakeUnique<FNav3DAStar>();
	ThetaStarSolver = MakeUnique<FNav3DThetaStar>();
	LazyThetaStarSolver = MakeUnique<FNav3DLazyThetaStar>();
}

FNav3DPathCoordinator::~FNav3DPathCoordinator() = default;

FNav3DPathCoordinator& FNav3DPathCoordinator::Get()
{
	if (!Instance)
	{
		Instance = MakeUnique<FNav3DPathCoordinator>();
	}
	return *Instance.Get();
}

INav3DPathfinder* FNav3DPathCoordinator::GetAlgorithm(ENav3DPathingAlgorithm AlgorithmType) const
{
	switch (AlgorithmType)
	{
	case ENav3DPathingAlgorithm::AStar:
		return AStarSolver.Get();
	case ENav3DPathingAlgorithm::ThetaStar:
		return ThetaStarSolver.Get();
	case ENav3DPathingAlgorithm::LazyThetaStar:
	default:
		return LazyThetaStarSolver.Get();
	}
}

ENavigationQueryResult::Type FNav3DPathCoordinator::FindPath(
	FNav3DPath& OutPath,
	const FNav3DPathingRequest& Request)
{
	FNav3DPathingRequest EnhancedRequest = Request;

	// Populate calculators and defaults from settings if missing
	if (!EnhancedRequest.CostCalculator || !EnhancedRequest.HeuristicCalculator || EnhancedRequest.HeuristicScale <= 0.0f)
	{
		if (const UNav3DSettings* Settings = UNav3DSettings::Get())
		{
			if (!EnhancedRequest.CostCalculator && Settings->DefaultCostCalculator)
			{
				EnhancedRequest.CostCalculator = NewObject<UNav3DPathTraversalCostCalculator>(GetTransientPackage(), Settings->DefaultCostCalculator);
			}
			if (!EnhancedRequest.HeuristicCalculator && Settings->DefaultHeuristic)
			{
				EnhancedRequest.HeuristicCalculator = NewObject<UNav3DPathHeuristicCalculator>(GetTransientPackage(), Settings->DefaultHeuristic);
			}
			if (EnhancedRequest.HeuristicScale <= 0.0f)
			{
				EnhancedRequest.HeuristicScale = Settings->HeuristicScale;
			}
			if (!EnhancedRequest.bUseNodeSizeCompensation)
			{
				EnhancedRequest.bUseNodeSizeCompensation = Settings->bUseNodeSizeCompensation;
			}
		}
	}

	INav3DPathfinder* Algorithm = Get().GetAlgorithm(EnhancedRequest.Algorithm);
	return Get().VolumeManager->FindPath(OutPath, EnhancedRequest, Algorithm);
}


