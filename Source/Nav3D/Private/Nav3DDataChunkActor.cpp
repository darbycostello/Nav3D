#include "Nav3DDataChunkActor.h"
#include "Nav3DUtils.h"
#include "Nav3DWorldSubsystem.h"
#include "Nav3DData.h"
#include "Nav3DDataChunk.h"
#include "AI/NavigationSystemBase.h"
#include "GameFramework/WorldSettings.h"
#include "Nav3D.h"

ANav3DDataChunkActor::ANav3DDataChunkActor(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	SetCanBeDamaged(false);
	SetActorEnableCollision(false);
}

uint32 ANav3DDataChunkActor::GetDefaultGridSize(UWorld* InWorld) const
{
	return 25600;
}

void ANav3DDataChunkActor::GetActorBounds(bool bOnlyCollidingComponents, FVector& OutOrigin, FVector& OutBoxExtent, bool bIncludeFromChildActors) const
{
	DataChunkActorBounds.GetCenterAndExtents(OutOrigin, OutBoxExtent);
}

#if WITH_EDITOR
void ANav3DDataChunkActor::GetStreamingBounds(FBox& OutRuntimeBounds, FBox& OutEditorBounds) const
{
	OutRuntimeBounds = OutEditorBounds = DataChunkActorBounds;
}

void ANav3DDataChunkActor::SetDataChunkActorBounds(const FBox& InBounds)
{
	DataChunkActorBounds = InBounds;
}
#endif // WITH_EDITOR

void ANav3DDataChunkActor::BeginPlay()
{
	Super::BeginPlay();
	AddNav3DChunkToWorld();
}

void ANav3DDataChunkActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	RemoveNav3DChunkFromWorld();
	Super::EndPlay(EndPlayReason);
}

void ANav3DDataChunkActor::AddNav3DChunkToWorld()
{
	if (UWorld* World = GetWorld())
	{
		if (World->GetNavigationSystem())
		{
			// Bake adjacency across our local chunks; store baked data for runtime
			float VoxelSize = 0.0f;
			if (Nav3DChunks.Num() > 0)
			{
				VoxelSize = FNav3DUtils::GetChunkLeafNodeSize(Nav3DChunks[0]);
			}
			if (VoxelSize > 0.0f)
			{
				for (UNav3DDataChunk* Chunk : Nav3DChunks)
				{
					if (Chunk && Chunk->BoundaryVoxels.Num() == 0)
					{
						FNav3DUtils::IdentifyBoundaryVoxels(Chunk);
					}
				}

				for (int32 i = 0; i < Nav3DChunks.Num(); ++i)
				{
					for (int32 j = i + 1; j < Nav3DChunks.Num(); ++j)
					{
						UNav3DDataChunk* A = Nav3DChunks[i];
						UNav3DDataChunk* B = Nav3DChunks[j];
						if (A && B && FNav3DUtils::AreChunksAdjacent(A, B, VoxelSize))
						{
							FNav3DUtils::BuildAdjacencyBetweenChunks(A, B, VoxelSize);
						}
					}
				}

				// Build transient portal lookup from baked ChunkAdjacency
				PortalLookup.Reset();
				for (const FNav3DChunkAdjacency& Adj : ChunkAdjacency)
				{
					for (const FNav3DVoxelConnection& Conn : Adj.Connections)
					{
						PortalLookup.FindOrAdd(Conn.LocalVolumeIndex).Add(Conn.Local, Conn);
					}
				}

				// Register actor in world spatial index
				if (UNav3DWorldSubsystem* Subsys = GetWorld()->GetSubsystem<UNav3DWorldSubsystem>())
				{
					Subsys->RegisterChunkActor(this);
				}
			}
		}
	}
}

void ANav3DDataChunkActor::RemoveNav3DChunkFromWorld()
{
	if (UWorld* World = GetWorld())
	{
		if (World->GetNavigationSystem())
		{
			for (UNav3DDataChunk* Chunk : Nav3DChunks)
			{
				if (Chunk)
				{
					Chunk->BoundaryVoxels.Reset();
					Chunk->MortonToBoundaryIndex.Reset();
				}
			}
			// Clear transient portal lookup
			PortalLookup.Reset();
			// Unregister from world spatial index
			if (UNav3DWorldSubsystem* Subsystem = GetWorld()->GetSubsystem<UNav3DWorldSubsystem>())
			{
				Subsystem->UnregisterChunkActor(this);
			}
		}
	}
}

// ============================================================================
// NEW UNIVERSAL CHUNK ACTOR METHODS
// ============================================================================

void ANav3DDataChunkActor::InitializeForStandardLevel()
{
	UE_LOG(LogNav3D, Log, TEXT("Initializing chunk actor for standard level: %s"), *GetName());
	
	// For standard levels, we don't need world partition specific setup
	// Just ensure we're registered with the navigation system
	RegisterWithNavigationSystem();
}

void ANav3DDataChunkActor::InitializeForWorldPartition()
{
	UE_LOG(LogNav3D, Log, TEXT("Initializing chunk actor for world partition: %s"), *GetName());
	
	// For world partition, we use the existing partition actor functionality
	// The parent class already handles world partition registration
	RegisterWithNavigationSystem();
}

bool ANav3DDataChunkActor::ContainsPoint(const FVector& Point) const
{
	return DataChunkActorBounds.IsInsideXY(Point);
}

const UNav3DDataChunk* ANav3DDataChunkActor::GetChunkContainingPoint(const FVector& Point) const
{
	if (!ContainsPoint(Point))
	{
		return nullptr;
	}
	
	// For now, return the first chunk if we contain the point
	// In the future, we might have multiple chunks per actor
	if (Nav3DChunks.Num() > 0)
	{
		return Nav3DChunks[0];
	}
	
	return nullptr;
}

void ANav3DDataChunkActor::RegisterWithNavigationSystem()
{
	// Register with spatial subsystem for fast queries
	if (UWorld* World = GetWorld())
	{
		if (UNav3DWorldSubsystem* Subsystem = World->GetSubsystem<UNav3DWorldSubsystem>())
		{
			Subsystem->RegisterChunkActor(this);
		}
		
		// Register with the first available Nav3DData
		if (ANav3DData* Nav3DData = FNav3DUtils::GetNav3DData(World))
		{
			Nav3DData->RegisterChunkActor(this);
		}
	}
	
	// Build transient portal lookup for fast runtime queries
	PortalLookup.Reset();
	for (const FNav3DChunkAdjacency& Adj : ChunkAdjacency)
	{
		for (const FNav3DVoxelConnection& Conn : Adj.Connections)
		{
			PortalLookup.FindOrAdd(Conn.LocalVolumeIndex).Add(Conn.Local, Conn);
		}
	}
}

void ANav3DDataChunkActor::UnregisterFromNavigationSystem()
{
	if (UWorld* World = GetWorld())
	{
		if (UNav3DWorldSubsystem* Subsystem = World->GetSubsystem<UNav3DWorldSubsystem>())
		{
			Subsystem->UnregisterChunkActor(this);
		}
		
		// Unregister from the first available Nav3DData
		if (ANav3DData* Nav3DData = FNav3DUtils::GetNav3DData(World))
		{
			Nav3DData->UnregisterChunkActor(this);
		}
	}
	
	// Clear transient data
	PortalLookup.Reset();
}

#if WITH_EDITOR
void ANav3DDataChunkActor::RebuildNavigationData() const
{
	UE_LOG(LogNav3D, Log, TEXT("Rebuilding navigation data for chunk actor: %s"), *GetName());
	
	// Get the Nav3DData owner to trigger a single volume build
	if (ANav3DData* Nav3DData = FNav3DUtils::GetNav3DData(GetWorld()))
	{
		// Store the bounds before destroying this actor
		const FBox BoundsToRebuild = DataChunkActorBounds;
        
		// Destroy this actor first (it will auto-unregister)
		GetWorld()->DestroyActor(const_cast<ANav3DDataChunkActor*>(this));
        
		// Then rebuild only this chunk
		Nav3DData->RebuildSingleChunk(BoundsToRebuild);
		
		UE_LOG(LogNav3D, Log, TEXT("Rebuild initiated for chunk actor: %s"), *GetName());
	}
	else
	{
		UE_LOG(LogNav3D, Error, TEXT("No Nav3DData found for chunk actor rebuild: %s"), *GetName());
	}
}
#endif // WITH_EDITOR


