#include "Nav3DDataChunkActor.h"

#include "EngineUtils.h"
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

void ANav3DDataChunkActor::Serialize(FArchive& Ar)
{
	Super::Serialize(Ar);
    
	if (Ar.IsSaving())
	{
		UE_LOG(LogNav3D, Verbose, TEXT("SAVE Chunk %s: %d compact regions, %d adjacency, %d visibility refs, VolumeID=%d"),
			*GetName(),
			CompactTacticalData.Regions.Num(),
			CompactTacticalData.RegionAdjacency.Num(),
			CompactTacticalData.VisibilityMatrix.SparseReferences.Num(),
			CompactTacticalData.VolumeID);

		for (int32 i = 0; i < ChunkAdjacency.Num(); ++i)
		{
			const FNav3DChunkAdjacency& Adj = ChunkAdjacency[i];
			UE_LOG(LogNav3D, VeryVerbose, TEXT("  Adj[%d]: OtherChunk=%s, Valid=%s, Portals=%d"), 
			       i, 
			       Adj.OtherChunkActor.IsValid() ? *Adj.OtherChunkActor->GetName() : TEXT("INVALID"),
			       Adj.OtherChunkActor.IsValid() ? TEXT("Yes") : TEXT("No"),
			       Adj.CompactPortals.Num());
		}
	}
	else if (Ar.IsLoading())
	{
		UE_LOG(LogNav3D, Warning, TEXT("LOAD Chunk %s: %d compact regions, %d adjacency, %d visibility refs, VolumeID=%d"),
			*GetName(),
			CompactTacticalData.Regions.Num(),
			CompactTacticalData.RegionAdjacency.Num(),
			CompactTacticalData.VisibilityMatrix.SparseReferences.Num(),
			CompactTacticalData.VolumeID);
		
		// Debug adjacency data being loaded
		UE_LOG(LogNav3D, Verbose, TEXT("LOAD Chunk %s: ChunkAdjacency has %d entries"), *GetName(), ChunkAdjacency.Num());
		for (int32 i = 0; i < ChunkAdjacency.Num(); ++i)
		{
			const FNav3DChunkAdjacency& Adj = ChunkAdjacency[i];
			UE_LOG(LogNav3D, VeryVerbose, TEXT("  Adj[%d]: OtherChunk=%s, Valid=%s, Portals=%d"), 
			       i, 
			       Adj.OtherChunkActor.IsValid() ? *Adj.OtherChunkActor->GetName() : TEXT("INVALID"),
			       Adj.OtherChunkActor.IsValid() ? TEXT("Yes") : TEXT("No"),
			       Adj.CompactPortals.Num());
		}
	}
}

void ANav3DDataChunkActor::PostLoad()
{
	Super::PostLoad();

	if (!CompactTacticalData.IsEmpty())
	{
		UE_LOG(LogNav3D, Verbose, TEXT("Chunk %s loaded with %d compact regions, %d adjacency entries"),
			*GetName(),
			CompactTacticalData.Regions.Num(),
			CompactTacticalData.RegionAdjacency.Num());
	}
	else
	{
		UE_LOG(LogNav3D, VeryVerbose, TEXT("Chunk %s loaded without compact tactical data"), *GetName());
	}
	
	// Fix invalid weak object pointers in ChunkAdjacency
	int32 FixedReferences = 0;
	for (FNav3DChunkAdjacency& Adj : ChunkAdjacency)
	{
		if (!Adj.OtherChunkActor.IsValid())
		{
			// Try to find the chunk by name or spatial proximity
			if (const UWorld* World = GetWorld())
			{
				// Get all chunk actors in the world
				TArray<ANav3DDataChunkActor*> AllChunks;
				for (TActorIterator<ANav3DDataChunkActor> ActorItr(World); ActorItr; ++ActorItr)
				{
					AllChunks.Add(*ActorItr);
				}
				
				// Find the closest chunk that this one should be adjacent to
				ANav3DDataChunkActor* BestMatch = nullptr;
				float BestDistance = FLT_MAX;
				
				for (ANav3DDataChunkActor* OtherChunk : AllChunks)
				{
					if (OtherChunk && OtherChunk != this)
					{
						// Check if this chunk is spatially adjacent
						if (IsAdjacentToChunk(OtherChunk))
						{
							float Distance = FVector::Dist(DataChunkActorBounds.GetCenter(), OtherChunk->DataChunkActorBounds.GetCenter());
							if (Distance < BestDistance)
							{
								BestDistance = Distance;
								BestMatch = OtherChunk;
							}
						}
					}
				}
				
				if (BestMatch)
				{
					Adj.OtherChunkActor = BestMatch;
					FixedReferences++;
					UE_LOG(LogNav3D, Verbose, TEXT("PostLoad: Fixed invalid adjacency reference in %s -> %s"), 
					       *GetName(), *BestMatch->GetName());
				}
			}
		}
	}
	
	if (FixedReferences > 0)
	{
		UE_LOG(LogNav3D, Verbose, TEXT("PostLoad: Fixed %d invalid adjacency references in chunk %s"), 
		       FixedReferences, *GetName());
	}
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

				// Register actor in world spatial index
				if (UNav3DWorldSubsystem* Subsystem = GetWorld()->GetSubsystem<UNav3DWorldSubsystem>())
				{
					Subsystem->RegisterChunkActor(this);
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
			// No transient portal lookup to clear
			// Unregister from world spatial index
			if (UNav3DWorldSubsystem* Subsystem = GetWorld()->GetSubsystem<UNav3DWorldSubsystem>())
			{
				Subsystem->UnregisterChunkActor(this);
			}
		}
	}
}

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

void ANav3DDataChunkActor::RegisterWithNavigationSystem()
{
	// Register with spatial subsystem for fast queries
	if (const UWorld* World = GetWorld())
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
	
	// No transient portal lookup; CompactPortals are used directly
}

void ANav3DDataChunkActor::UnregisterFromNavigationSystem()
{
	if (const UWorld* World = GetWorld())
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
	
	// No transient data to clear
}

bool ANav3DDataChunkActor::IsAdjacentToChunk(const ANav3DDataChunkActor* OtherChunk, const float Tolerance) const
{
	if (!OtherChunk)
	{
		return false;
	}
	
	const FBox ExpandedBounds = DataChunkActorBounds.ExpandBy(Tolerance);
	return ExpandedBounds.Intersect(OtherChunk->DataChunkActorBounds);
}

void ANav3DDataChunkActor::ClearTacticalData()
{
	// Clear compact tactical data structures
	CompactTacticalData.Reset();
	CompactRegions.Reset();
	ConnectionInterfaces.Reset();

	UE_LOG(LogNav3D, Verbose, TEXT("Cleared tactical data from chunk: %s"), *GetName());

	// Mark dirty in editor
#if WITH_EDITOR
	auto _ = MarkPackageDirty();
#endif
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