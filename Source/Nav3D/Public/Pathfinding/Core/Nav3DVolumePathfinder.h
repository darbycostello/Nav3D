#pragma once

#include "CoreMinimal.h"
#include "Pathfinding/Core/Nav3DPathingTypes.h"
#include "Pathfinding/Core/Nav3DPath.h"

class INav3DPathfinder;
class ANav3DData;
class ANav3DDataChunkActor;
class ANav3DBoundsVolume;
class FNav3DVolumeNavigationData;

class NAV3D_API FNav3DVolumePathfinder
{
public:
	FNav3DVolumePathfinder();
	~FNav3DVolumePathfinder();

	ENavigationQueryResult::Type FindPath(
		FNav3DPath& OutPath,
		const FNav3DPathingRequest& Request,
		INav3DPathfinder* Algorithm);

private:
	struct FPathSegment
	{
		FVector StartPoint = FVector::ZeroVector;
		FVector EndPoint = FVector::ZeroVector;
		const FNav3DVolumeNavigationData* VolumeData = nullptr;
		bool bRequiresWaypoint = false;
		const ANav3DDataChunkActor* SourceChunk = nullptr;
	};

	// Spatial analysis
	const ANav3DDataChunkActor* FindChunkContaining(const FVector& Location) const;
	const ANav3DBoundsVolume* FindVolumeContaining(const FVector& Location) const;
	static bool IsChunkEmpty(const ANav3DDataChunkActor* Chunk);

	// Cross-volume helpers
	ENavigationQueryResult::Type FindPathWithinVolume(
		FNav3DPath& OutPath,
		const FVector& StartLocation,
		const FVector& EndLocation,
		const ANav3DBoundsVolume* Volume,
		INav3DPathfinder* Algorithm) const;

	// Overload that reuses resolved chunks to avoid re-query and ensure consistency
	ENavigationQueryResult::Type FindPathWithinVolume(
		FNav3DPath& OutPath,
		const FVector& StartLocation,
		const FVector& EndLocation,
		const ANav3DBoundsVolume* Volume,
		INav3DPathfinder* Algorithm,
		const ANav3DDataChunkActor* ResolvedStartChunk,
		const ANav3DDataChunkActor* ResolvedEndChunk) const;

	ENavigationQueryResult::Type FindPathCrossVolume(
		FNav3DPath& OutPath,
		const FVector& StartLocation,
		const FVector& EndLocation,
		const ANav3DBoundsVolume* StartVolume,
		const ANav3DBoundsVolume* EndVolume,
		INav3DPathfinder* Algorithm) const;

	ENavigationQueryResult::Type FindPathInChunk(
		FNav3DPath& OutPath,
		const FVector& StartLocation,
		const FVector& EndLocation,
		const ANav3DDataChunkActor* Chunk,
		INav3DPathfinder* Algorithm) const;

	// Intra-volume helpers
	static TArray<const ANav3DDataChunkActor*> FindChunkPathWithinVolume(
		const ANav3DDataChunkActor* StartChunk,
		const ANav3DDataChunkActor* EndChunk);

	static FVector FindPortalBetweenChunks(
		const ANav3DDataChunkActor* FromChunk,
		const ANav3DDataChunkActor* ToChunk);

	static bool GetPortalPositions(
		const ANav3DDataChunkActor* FromChunk,
		const ANav3DDataChunkActor* ToChunk,
		FVector& OutLocalInFrom,
		FVector& OutRemoteInTo);

	ENavigationQueryResult::Type ProcessPathSegments(
		FNav3DPath& OutPath,
		const TArray<FPathSegment>& Segments,
		INav3DPathfinder* Algorithm) const;

	// Utilities
	static FVector FindVolumeExitPoint(const ANav3DBoundsVolume* Volume, const FVector& StartLocation, const FVector& TowardLocation);
	static FVector FindVolumeEntryPoint(const ANav3DBoundsVolume* Volume, const FVector& FromLocation, const FVector& EndLocation);

	static ENavigationQueryResult::Type CreateDirectPath(FNav3DPath& OutPath, const FVector& Start, const FVector& End);
	static ENavigationQueryResult::Type CombinePathSegments(FNav3DPath& OutPath, const TArray<FPathSegment>& Segments);

	void LogSpatialAnalysis(const FVector& Start, const FVector& End,
		const ANav3DDataChunkActor* StartChunk, const ANav3DDataChunkActor* EndChunk,
		const ANav3DBoundsVolume* StartVolume, const ANav3DBoundsVolume* EndVolume) const;

	// Current context
	FNav3DPathingRequest CurrentRequest;
	const ANav3DData* CurrentNavData = nullptr;
};


