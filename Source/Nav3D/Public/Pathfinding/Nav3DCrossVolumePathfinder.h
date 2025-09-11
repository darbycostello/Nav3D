#pragma once

#include "CoreMinimal.h"
#include "Nav3DTypes.h"

class UNav3DWorldSubsystem;
class ANav3DDataChunkActor;

struct FNav3DActorPortal
{
	TSoftObjectPtr<ANav3DDataChunkActor> From;
	TSoftObjectPtr<ANav3DDataChunkActor> To;
	FNav3DVoxelConnection Connection;
};

class NAV3D_API FNav3DCrossVolumePathfinder
{
public:
	// Compute a sequence of actors and portal connections from Start->End.
	// Returns false if no path at actor level.
	static bool FindActorPath(const UWorld* World,
		const FVector& Start,
		const FVector& End,
		TArray<ANav3DDataChunkActor*>& OutActorPath,
		TArray<FNav3DActorPortal>& OutPortals);
};


