#pragma once

#include "CoreMinimal.h"
#include "Nav3DVolumeNavigationData.h"
#include "Pathfinding/Core/Nav3DPath.h"
#include "Pathfinding/Core/Nav3DPathingTypes.h"

class NAV3D_API INav3DPathfinder
{
public:
	virtual ~INav3DPathfinder() = default;

	virtual ENavigationQueryResult::Type FindPath(
		FNav3DPath& OutPath,
		const FNav3DPathingRequest& Request,
		const FNav3DVolumeNavigationData* VolumeNavData) = 0;

protected:
	static void LogPathfindingStart(const FNav3DPathingRequest& Request, const FString& AlgorithmName);
	static void LogPathfindingResult(ENavigationQueryResult::Type Result, int32 PathPointCount, const FString& AlgorithmName);
	static void LogAlgorithmProgress(const FNav3DPathingRequest& Request, const FString& Message);
};


