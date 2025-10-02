#include "Pathfinding/Core/Nav3DPathCoordinator.h"
#include "Nav3D.h"
#include "Nav3DSettings.h"
#include "Pathfinding/Core/Nav3DPath.h"
#include "Pathfinding/Search/Nav3DPathHeuristicCalculator.h"
#include "Pathfinding/Search/Nav3DPathTraversalCostCalculator.h"
#include "Pathfinding/Core/Nav3DVolumePathfinder.h"
#include "Pathfinding/Search/Nav3DAStar.h"
#include "Pathfinding/Search/Nav3DThetaStar.h"
#include "Pathfinding/Search/Nav3DLazyThetaStar.h"
#include "Raycasting/Nav3DMultiChunkRaycaster.h"

TUniquePtr<FNav3DPathCoordinator> FNav3DPathCoordinator::Instance;

FNav3DPathCoordinator::FNav3DPathCoordinator()
{
	VolumeManager = MakeUnique<FNav3DVolumePathfinder>();
	AStarSolver = MakeUnique<FNav3DAStar>();
	ThetaStarSolver = MakeUnique<FNav3DThetaStar>();
	LazyThetaStarSolver = MakeUnique<FNav3DLazyThetaStar>();
	MultiChunkRaycaster = NewObject<UNav3DMultiChunkRaycaster>();
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
	
	if (Get().TryDirectTraversal(EnhancedRequest, OutPath))
	{
		return ENavigationQueryResult::Success;
	}

	INav3DPathfinder* Algorithm = Get().GetAlgorithm(EnhancedRequest.Algorithm);
	return Get().VolumeManager->FindPath(OutPath, EnhancedRequest, Algorithm);
}

bool FNav3DPathCoordinator::TryDirectTraversal(
	const FNav3DPathingRequest& Request,
	FNav3DPath& OutPath) const
{
	UE_LOG(LogNav3D, Verbose, TEXT("TryDirectTraversal: Starting direct traversal check from %s to %s"), 
		*Request.StartLocation.ToString(), *Request.EndLocation.ToString());

	if (!Request.NavData)
	{
		UE_LOG(LogNav3D, Verbose, TEXT("TryDirectTraversal: Failed - No NavData provided"));
		return false;
	}

	if (!MultiChunkRaycaster)
	{
		UE_LOG(LogNav3D, Verbose, TEXT("TryDirectTraversal: Failed - MultiChunkRaycaster not initialized"));
		return false;
	}

	const float Distance = FVector::Dist(Request.StartLocation, Request.EndLocation);
	UE_LOG(LogNav3D, Verbose, TEXT("TryDirectTraversal: Distance = %.2f, AgentRadius = %.2f"), 
		Distance, Request.AgentProperties.AgentRadius);

	FNav3DRaycastHit Hit;
	const bool bHasLineOfTraversal = MultiChunkRaycaster->HasLineOfTraversal(
		Request.NavData,
		Request.StartLocation,
		Request.EndLocation,
		Request.AgentProperties.AgentRadius,
		Hit);

	if (!bHasLineOfTraversal)
	{
		UE_LOG(LogNav3D, Verbose, TEXT("TryDirectTraversal: Failed - Line of traversal blocked at distance %.2f"), 
			Hit.Distance);
		return false;
	}

	UE_LOG(LogNav3D, Log, TEXT("TryDirectTraversal: SUCCESS - Direct path found! Creating 2-point path"));

	// Create 2-point path
	OutPath.ResetForRepath();
	OutPath.GetPathPoints().Add(FNavPathPoint(Request.StartLocation));
	OutPath.GetPathPoints().Add(FNavPathPoint(Request.EndLocation));
	OutPath.MarkReady();

	UE_LOG(LogNav3D, Log, TEXT("TryDirectTraversal: Created direct path with %d points"), 
		OutPath.GetPathPoints().Num());

	return true;
}


