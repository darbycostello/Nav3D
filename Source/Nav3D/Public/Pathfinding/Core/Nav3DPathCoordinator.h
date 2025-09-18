#pragma once

#include "CoreMinimal.h"
#include "Pathfinding/Core/Nav3DPathingTypes.h"
#include "Pathfinding/Core/Nav3DPath.h"

class INav3DPathfinder;
class FNav3DVolumePathfinder;

class NAV3D_API FNav3DPathCoordinator
{
public:
	FNav3DPathCoordinator();
	~FNav3DPathCoordinator();

	static ENavigationQueryResult::Type FindPath(
		FNav3DPath& OutPath,
		const FNav3DPathingRequest& Request);

	static FNav3DPathCoordinator& Get();

private:
	TUniquePtr<class FNav3DAStar> AStarSolver;
	TUniquePtr<class FNav3DThetaStar> ThetaStarSolver;
	TUniquePtr<class FNav3DLazyThetaStar> LazyThetaStarSolver;

	TUniquePtr<FNav3DVolumePathfinder> VolumeManager;

	INav3DPathfinder* GetAlgorithm(ENav3DPathingAlgorithm AlgorithmType) const;

	static TUniquePtr<FNav3DPathCoordinator> Instance;
};


