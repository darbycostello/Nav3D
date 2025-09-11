#pragma once
#include <AI/Navigation/NavQueryFilter.h>
#include <AI/Navigation/NavigationTypes.h>

#include "Nav3DCrossVolumePathfinder.h"
#include "Nav3DData.h"
#include "Nav3DPathFindingTypes.h"

class FNav3DPath;
class FNav3DPathStepper;
struct FPathFindingQuery;
class ANav3DData;

struct FPathSegment
{
	FVector StartPoint;
	FVector EndPoint;
	const FNav3DVolumeNavigationData* NavVolume;

	FPathSegment(const FVector& Start, const FVector& End, const FNav3DVolumeNavigationData* Volume)
		: StartPoint(Start)
		  , EndPoint(End)
		  , NavVolume(Volume)
	{
	}
};

struct FSanitizedPath
{
	TArray<FPathSegment> Segments;
	bool bAdjustedEndPoint = false;
};

class NAV3D_API FNav3DPathFinder
{
public:
	static ENavigationQueryResult::Type GetPath(
		FNav3DPath& NavPath,
		const ANav3DData& NavData,
		const FVector& StartLocation,
		const FVector& EndLocation,
		const FNavAgentProperties& NavAgentProperties,
		const FSharedConstNavQueryFilter& NavQueryFilter);

	static void BuildPath(
		FNav3DPath& Path,
		const FNav3DPathFindingParameters& Params,
		const TArray<FNav3DPathFinderNodeAddress>& NodeAddresses,
		const bool AddEndLocation);

private:
	static ENavigationQueryResult::Type GetPathInternal(
		FNav3DPath& NavigationPath,
		const FNav3DVolumeNavigationData& VolumeNavData,
		const ANav3DData& NavData,
		const FVector& StartLocation,
		const FVector& EndLocation,
		const FNavAgentProperties& NavAgentProperties,
		const FSharedConstNavQueryFilter& NavQueryFilter);

	static ENavigationQueryResult::Type GetPathInternal(
		FNav3DPath& NavigationPath,
		const ANav3DData& NavData,
		const FVector& StartLocation,
		const FVector& EndLocation,
		const FNavAgentProperties& NavAgentProperties,
		const FSharedConstNavQueryFilter& NavQueryFilter);

	static FSanitizedPath SanitizePath(
		const ANav3DData& NavData,
		const FVector& OriginalStart,
		const FVector& OriginalEnd);

	static UNav3DPathFindingSearch* GetPathFindingSearch(
		const FSharedConstNavQueryFilter& NavQueryFilter);

	static TSharedPtr<FNav3DPathStepper> CreateCrossVolumeDebugStepper(
		FNav3DPathFinderDebugData& DebugData,
		const ANav3DData& NavData,
		const TArray<ANav3DDataChunkActor*>& ActorPath,
		const TArray<FNav3DActorPortal>& Portals,
		const FVector& StartLocation,
		const FVector& EndLocation,
		const FNavAgentProperties& NavAgentProperties,
		const FSharedConstNavQueryFilter& NavQueryFilter);

	static void ValidatePathPoints(
		FNav3DPath& Path,
		const ANav3DData& NavData,
		const FNavAgentProperties& AgentProperties);
};
