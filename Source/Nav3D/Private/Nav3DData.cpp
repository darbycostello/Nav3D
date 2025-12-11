#include "Nav3DData.h"
#include "Nav3DDataChunk.h"
#include "Nav3DDataGenerator.h"
#include "Nav3DNavDataRenderingComponent.h"
#include "Nav3DDataChunkActor.h"
#include "Nav3DWorldSubsystem.h"
#include <AI/NavDataGenerator.h>
#include <DrawDebugHelpers.h>
#include <NavigationSystem.h>
#include <HAL/CriticalSection.h>
#include "EngineUtils.h"
#include "Nav3D.h"
#include "Nav3DUtils.h"
#include "Raycasting/Nav3DRaycaster.h"
#include "Tactical/Nav3DTacticalReasoning.h"
#include "Tactical/Nav3DTacticalDataConverter.h"
#include "Nav3DVolumeIDSystem.h"
#include "Components/PrimitiveComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Components/SphereComponent.h"
#include "Components/BoxComponent.h"
#include "Components/CapsuleComponent.h"
#include "LandscapeMeshCollisionComponent.h"
#include "LandscapeHeightfieldCollisionComponent.h"
#include "Nav3DBoundsVolume.h"
#include "PhysicsEngine/BodySetup.h"
#include "Engine/StaticMesh.h"
#include "Engine/OverlapResult.h"
#include "Internationalization/Text.h"
#include "Internationalization/Internationalization.h"
#include "Pathfinding/Core/Nav3DPath.h"

#if WITH_EDITOR
#include <ObjectEditorUtils.h>
#endif

FNav3DGenerationFinishedDelegate ANav3DData::GenerationFinishedDelegate;

// ============================================================================
// CONSOLE COMMANDS
// ============================================================================

static FAutoConsoleCommand CCmd_ConsolidateTactical(
	TEXT("Nav3D.ConsolidateTactical"),
	TEXT("Manually consolidate tactical data and force draw update"),
	FConsoleCommandDelegate::CreateLambda([]()
	{
		// Find all Nav3DData instances in the world
		if (const UWorld* World = GEngine->GetWorldFromContextObject(nullptr, EGetWorldErrorMode::LogAndReturnNull))
		{
			for (TActorIterator<ANav3DData> It(World); It; ++It)
			{
				ANav3DData* Nav3DData = *It;
				if (Nav3DData && Nav3DData->TacticalSettings.bEnableTacticalReasoning)
				{
					UE_LOG(LogNav3D, Log, TEXT("Manually consolidating tactical data for %s"), *Nav3DData->GetName());
					Nav3DData->RefreshConsolidatedTacticalData();
					Nav3DData->RequestDrawingUpdate();
				}
			}
		}
	})
);

static FAutoConsoleCommand CCmd_LogPerformanceStats(
	TEXT("Nav3D.LogPerformanceStats"),
	TEXT("Log performance statistics for all Nav3D instances"),
	FConsoleCommandDelegate::CreateLambda([]()
	{
		// Find all Nav3DData instances in the world
		if (const UWorld* World = GEngine->GetWorldFromContextObject(nullptr, EGetWorldErrorMode::LogAndReturnNull))
		{
			for (TActorIterator<ANav3DData> It(World); It; ++It)
			{
				const ANav3DData* Nav3DData = *It;
				if (Nav3DData && Nav3DData->TacticalSettings.bEnableTacticalReasoning)
				{
					UE_LOG(LogNav3D, Log, TEXT("Performance stats for %s:"), *Nav3DData->GetName());
					Nav3DData->LogPerformanceStats();
				}
			}
		}
	})
);

static FAutoConsoleCommand CCmd_LogLoadedRegions(
	TEXT("Nav3D.LogLoadedRegions"),
	TEXT("Log all loaded region IDs for tactical filtering"),
	FConsoleCommandDelegate::CreateLambda([]()
	{
		// Find all Nav3DData instances in the world
		if (const UWorld* World = GEngine->GetWorldFromContextObject(nullptr, EGetWorldErrorMode::LogAndReturnNull))
		{
			for (TActorIterator<ANav3DData> It(World); It; ++It)
			{
				ANav3DData* Nav3DData = *It;
				if (Nav3DData && Nav3DData->TacticalSettings.bEnableTacticalReasoning)
				{
					UE_LOG(LogNav3D, Log, TEXT("Loaded regions for %s:"), *Nav3DData->GetName());
					Nav3DData->UpdateLoadedRegionIds();
					const TSet<int32>& LoadedRegions = Nav3DData->GetLoadedRegionIds();
					
					FString RegionList;
					for (const int32 RegionId : LoadedRegions)
					{
						RegionList += FString::Printf(TEXT("%d "), RegionId);
					}
					UE_LOG(LogNav3D, Log, TEXT("Loaded region IDs: %s"), *RegionList);
					UE_LOG(LogNav3D, Log, TEXT("Total loaded regions: %d"), LoadedRegions.Num());
				}
			}
		}
	})
);

static FAutoConsoleCommand CCmd_RebuildCompactTactical(
	TEXT("Nav3D.RebuildCompactTactical"),
	TEXT("Manually rebuild consolidated compact tactical data"),
	FConsoleCommandDelegate::CreateLambda([]()
	{
		// Find all Nav3DData instances in the world
		if (const UWorld* World = GEngine->GetWorldFromContextObject(nullptr, EGetWorldErrorMode::LogAndReturnNull))
		{
			for (TActorIterator<ANav3DData> It(World); It; ++It)
			{
				ANav3DData* Nav3DData = *It;
				if (Nav3DData && Nav3DData->TacticalSettings.bEnableTacticalReasoning)
				{
					UE_LOG(LogNav3D, Log, TEXT("Manually rebuilding consolidated compact tactical data for %s"), *Nav3DData->GetName());
					Nav3DData->RebuildConsolidatedCompactTacticalData();
					Nav3DData->RequestDrawingUpdate();
				}
			}
		}
	})
);

static FAutoConsoleCommand CCmd_ListVolumeIDs(
	TEXT("Nav3D.ListVolumeIDs"),
	TEXT("List all currently loaded Nav3DBoundsVolume IDs and GUIDs"),
	FConsoleCommandDelegate::CreateLambda([]()
	{
		if (UWorld* World = GWorld)
		{
			TMap<uint16, FGuid> VolumeMap = FNav3DVolumeIDSystem::GetLoadedVolumeIDs(World);
			
			UE_LOG(LogNav3D, Display, TEXT("=== Currently Loaded Nav3DBoundsVolumes ==="));
			for (const auto& Pair : VolumeMap)
			{
				const uint16 VolumeID = Pair.Key;
				const FGuid& VolumeGUID = Pair.Value;
				
				if (const ANav3DBoundsVolume* Volume = FNav3DVolumeIDSystem::FindVolumeByID(World, VolumeID))
				{
					UE_LOG(LogNav3D, Display, TEXT("Volume ID: %d | GUID: %s | Name: %s"), 
					   VolumeID, *VolumeGUID.ToString(), *Volume->GetName());
				}
			}
			
			// Check for collisions
			if (!FNav3DVolumeIDSystem::ValidateNoCollisions(World))
			{
				UE_LOG(LogNav3D, Error, TEXT("Volume ID collisions detected! See log above."));
			}
			else
			{
				UE_LOG(LogNav3D, Display, TEXT("No volume ID collisions detected."));
			}
		}
	})
);

static FAutoConsoleCommand CCmd_TestTacticalConversion(
	TEXT("Nav3D.TestTacticalConversion"),
	TEXT("Test conversion from compact to consolidated tactical data"),
	FConsoleCommandDelegate::CreateLambda([]()
	{
		if (const UWorld* World = GWorld)
		{
			for (TActorIterator<ANav3DData> It(World); It; ++It)
			{
				ANav3DData* NavData = *It;
				if (NavData && IsValid(NavData))
				{
					const int32 CompactRegions = NavData->ConsolidatedCompactTacticalData.GetRegionCount();
					const int32 ConsolidatedRegions = NavData->ConsolidatedTacticalData.GetRegionCount();
					
					UE_LOG(LogNav3D, Display, TEXT("Nav3DData '%s': Compact regions: %d, Consolidated regions: %d"), 
					   *NavData->GetName(), CompactRegions, ConsolidatedRegions);
					
					if (CompactRegions > 0 && ConsolidatedRegions == 0)
					{
						UE_LOG(LogNav3D, Display, TEXT("Testing conversion..."));
						NavData->RebuildConsolidatedTacticalDataFromCompact();
						
						const int32 NewConsolidatedRegions = NavData->ConsolidatedTacticalData.GetRegionCount();
						UE_LOG(LogNav3D, Display, TEXT("Conversion result: %d consolidated regions"), NewConsolidatedRegions);
					}
				}
			}
		}
	})
);

// ============================================================================
// INITIALIZATION METHODS
// ============================================================================

void ANav3DData::BeginPlay()
{
	Super::BeginPlay();
	
}

void ANav3DData::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	// Clean up chunk actors
	for (ANav3DDataChunkActor* ChunkActor : ChunkActors)
	{
		if (ChunkActor)
		{
			ChunkActor->UnregisterFromNavigationSystem();
		}
	}
	
	Super::EndPlay(EndPlayReason);
}

void ANav3DData::DiscoverExistingChunkActors()
{
	if (const UWorld* World = GetWorld())
	{
		// Clean up any invalid actors before discovering new ones
		const int32 InvalidCount = GetInvalidChunkActorCount();
		if (InvalidCount > 0)
		{
			UE_LOG(LogNav3D, Log, TEXT("DiscoverExistingChunkActors: Found %d invalid actors, cleaning up"), InvalidCount);
			CleanupInvalidChunkActors();
		}
		
		ChunkActors.Reset();
		
		for (TActorIterator<ANav3DDataChunkActor> It(World); It; ++It)
		{
			ANav3DDataChunkActor* ChunkActor = *It;
			if (ChunkActor && ChunkActor->Nav3DChunks.Num() > 0)
			{
				RegisterChunkActor(ChunkActor);
			}
		}
		
		UE_LOG(LogNav3D, Log, TEXT("Discovered %d existing chunk actors"), ChunkActors.Num());
	}
}

UNav3DWorldSubsystem* ANav3DData::GetSubsystem() const
{
	if (!CachedSubsystem.IsValid())
	{
		if (const UWorld* World = GetWorld())
		{
			CachedSubsystem = World->GetSubsystem<UNav3DWorldSubsystem>();
		}
	}
	
	return CachedSubsystem.Get();
}

// ============================================================================
// SYSTEM VALIDATION AND HEALTH CHECKS
// ============================================================================

void ANav3DData::ValidateNavigationSystem()
{
	UE_LOG(LogNav3D, Display, TEXT("=== Nav3D System Validation ==="));
	
	// Check chunk actor integrity and clean up invalid actors
	const int32 InvalidCount = GetInvalidChunkActorCount();
	if (InvalidCount > 0)
	{
		UE_LOG(LogNav3D, Warning, TEXT("Found %d invalid chunk actors, cleaning up"), InvalidCount);
		CleanupInvalidChunkActors();
	}
	
	// Count valid actors after cleanup
	int32 ValidChunkActors = 0;
	int32 ActorsWithNoNavData = 0;
	
	for (const ANav3DDataChunkActor* ChunkActor : ChunkActors)
	{
		if (ChunkActor && IsValid(ChunkActor))
		{
			if (ChunkActor->Nav3DChunks.Num() == 0)
			{
				UE_LOG(LogNav3D, Warning, TEXT("Chunk actor %s has no navigation data"), 
				       *ChunkActor->GetName());
				ActorsWithNoNavData++;
			}
			else
			{
				ValidChunkActors++;
			}
		}
	}
	
	UE_LOG(LogNav3D, Display, TEXT("Chunk Actors - Valid: %d, No Nav Data: %d, Total: %d"), 
	       ValidChunkActors, ActorsWithNoNavData, ChunkActors.Num());
	
	
	// Check volume coverage
	const TArray<FBox> PartitionedVolumes = GetPartitionedVolumes();
	UE_LOG(LogNav3D, Display, TEXT("Partitioned Volumes: %d"), PartitionedVolumes.Num());
	
	// Check adjacency
	int32 TotalAdjacencies = 0;
	for (const ANav3DDataChunkActor* ChunkActor : ChunkActors)
	{
		if (ChunkActor)
		{
			TotalAdjacencies += ChunkActor->ChunkAdjacency.Num();
		}
	}
	UE_LOG(LogNav3D, Display, TEXT("Total Adjacency Connections: %d"), TotalAdjacencies);
	
	// Check spatial subsystem
	if (GetSubsystem())
	{
		UE_LOG(LogNav3D, Display, TEXT("Spatial subsystem operational"));
	}
	else
	{
		UE_LOG(LogNav3D, Error, TEXT("Spatial subsystem not available"));
	}
	
	UE_LOG(LogNav3D, Display, TEXT("============================"));
}

void ANav3DData::ShowBuildStatus()
{
	UE_LOG(LogNav3D, Display, TEXT("=== Nav3D Build Status ==="));
	UE_LOG(LogNav3D, Display, TEXT("Chunk Actors: %d"), ChunkActors.Num());
	UE_LOG(LogNav3D, Display, TEXT("Total Bounds: %s"), *GetBoundingBox().ToString());
	
	TArray<FBox> PartitionedVolumes = GetPartitionedVolumes();
	UE_LOG(LogNav3D, Display, TEXT("Partitioned Volumes: %d"), PartitionedVolumes.Num());
	
	for (int32 i = 0; i < PartitionedVolumes.Num(); i++)
	{
		UE_LOG(LogNav3D, Display, TEXT("  Volume %d: %s"), i, *PartitionedVolumes[i].ToString());
	}
	
	// Show chunk actor details
	for (int32 i = 0; i < ChunkActors.Num(); i++)
	{
		if (const ANav3DDataChunkActor* ChunkActor = ChunkActors[i])
		{
			UE_LOG(LogNav3D, Display, TEXT("  Chunk Actor %d: %s (Built: %s, Building: %s, Needs Rebuild: %s)"), 
			       i, *ChunkActor->GetName(),
			       ChunkActor->bIsBuilt ? TEXT("Yes") : TEXT("No"),
			       ChunkActor->bIsBuilding ? TEXT("Yes") : TEXT("No"),
			       ChunkActor->bNeedsRebuild ? TEXT("Yes") : TEXT("No"));
		}
	}
	
	UE_LOG(LogNav3D, Display, TEXT("========================="));
}

// Helper functions for clean analysis output
static FString FormatNumber(const int32 Number)
{
    return FText::AsNumber(Number, &FNumberFormattingOptions::DefaultWithGrouping()).ToString();
}

static FString GetSimplifiedComponentName(const UPrimitiveComponent* Component)
{
    if (!Component) return TEXT("<Invalid>");
    
    FString Name = Component->GetName();
    // Remove common prefixes/suffixes for cleaner output
    Name = Name.Replace(TEXT("DefaultSceneRoot_"), TEXT(""));
    Name = Name.Replace(TEXT("_C"), TEXT(""));
    return Name;
}

static void LogSectionHeader(const FString& Title)
{
    UE_LOG(LogNav3D, Log, TEXT(""));
    UE_LOG(LogNav3D, Log, TEXT("=========================================="));
    UE_LOG(LogNav3D, Log, TEXT("=== %s ==="), *Title);
    UE_LOG(LogNav3D, Log, TEXT("=========================================="));
}

static void LogSectionFooter()
{
    UE_LOG(LogNav3D, Log, TEXT("=========================================="));
    UE_LOG(LogNav3D, Log, TEXT(""));
}

ANav3DData::ANav3DData()
{
	if (!HasAnyFlags(RF_ClassDefaultObject))
	{
		FindPathImplementation = FindPath;
	}

	InitializeTacticalReasoning();
}

ANav3DData::~ANav3DData()
{
	// Clean up tactical reasoning
	TacticalReasoning.Reset();
}

void ANav3DData::PostInitProperties()
{
	Super::PostInitProperties();

	UE_LOG(LogNav3D, Verbose, TEXT("Nav3DData PostInitProperties: %s"), *GetName());
	if (HasAnyFlags(RF_ClassDefaultObject | RF_NeedLoad) == false)
	{
		RecreateDefaultFilter();
	}
}

void ANav3DData::OnRegistered()
{
	Super::OnRegistered();

	UE_LOG(LogNav3D, Verbose, TEXT("Nav3DData OnRegistered: %s"), *GetName());
}

void ANav3DData::PostLoad()
{
	Super::PostLoad();
    
	if (TacticalSettings.bEnableTacticalReasoning)
	{
		UE_LOG(LogNav3D, Verbose, TEXT("PostLoad: Converting compact tactical data for debug rendering"));
        
		// Find chunks with compact data
		TArray<ANav3DDataChunkActor*> ChunksWithCompactData;
		for (ANav3DDataChunkActor* Chunk : GetChunkActors())
		{
			if (Chunk && !Chunk->CompactTacticalData.IsEmpty())
			{
				ChunksWithCompactData.Add(Chunk);
				UE_LOG(LogNav3D, VeryVerbose, TEXT("Found chunk %s with %d compact regions"),
					*Chunk->GetName(), Chunk->CompactTacticalData.Regions.Num());
			}
		}
        
		if (ChunksWithCompactData.Num() == 0)
		{
			UE_LOG(LogNav3D, VeryVerbose, TEXT("No compact tactical data to convert"));
			return;
		}
        
		// Build consolidated compact data from chunks
		BuildConsolidatedCompactFromChunks(ChunksWithCompactData);
        
		if (!ConsolidatedCompactTacticalData.IsEmpty())
		{
			// Convert to Build format for debug rendering only
			ConsolidatedTacticalData = FNav3DTacticalDataConverter::CompactToBuild(
				ConsolidatedCompactTacticalData, ChunksWithCompactData);
                
			UE_LOG(LogNav3D, Verbose, TEXT("Tactical data conversion successful - %d regions, %d adjacency entries, %d visibility entries available for original debug rendering"),
				ConsolidatedTacticalData.AllLoadedRegions.Num(),
				ConsolidatedTacticalData.RegionAdjacency.Num(),
				ConsolidatedTacticalData.RegionVisibility.Num());
		}
		else
		{
			UE_LOG(LogNav3D, Warning, TEXT("Failed to convert compact tactical data for debug rendering"));
		}
	}
}

void ANav3DData::BuildConsolidatedCompactFromChunks(const TArray<ANav3DDataChunkActor*>& ChunksWithData)
{
    ConsolidatedCompactTacticalData.Reset();
    
    TMap<int32, uint16> LocalToGlobalIdMap;  // Map chunk-local region index to global ID
    uint16 NextGlobalId = 0;
    
    UE_LOG(LogNav3D, Verbose, TEXT("Building consolidated compact data from %d chunks"), ChunksWithData.Num());
    
    for (const ANav3DDataChunkActor* ChunkActor : ChunksWithData)
    {
        const FCompactTacticalData& ChunkData = ChunkActor->CompactTacticalData;
        
        UE_LOG(LogNav3D, VeryVerbose, TEXT("Processing chunk %s: %d regions, VolumeID=%d"),
            *ChunkActor->GetName(), ChunkData.Regions.Num(), ChunkData.VolumeID);
        
        // Add regions with global ID remapping
        for (int32 LocalIdx = 0; LocalIdx < ChunkData.Regions.Num(); ++LocalIdx)
        {
            LocalToGlobalIdMap.Add(LocalIdx, NextGlobalId);
            ConsolidatedCompactTacticalData.AllLoadedRegions.Add(NextGlobalId, ChunkData.Regions[LocalIdx]);
            
            NextGlobalId++;
        }
        
        // Convert intra-volume adjacency to global IDs
        for (const auto& AdjPair : ChunkData.RegionAdjacency)
        {
            const uint8 LocalFromId = AdjPair.Key;
            const uint64 AdjMask = AdjPair.Value;
            
            if (const uint16* GlobalFromId = LocalToGlobalIdMap.Find(LocalFromId))
            {
                // Convert bitmask from local to global IDs
                uint64 GlobalAdjMask = 0;
                for (int32 Bit = 0; Bit < 64; ++Bit)
                {
                    if (AdjMask & (1ULL << Bit))
                    {
                        if (const uint16* GlobalToId = LocalToGlobalIdMap.Find(Bit))
                        {
                            if (*GlobalToId < 64)  // Ensure it fits in the 64-bit mask
                            {
                                GlobalAdjMask |= (1ULL << *GlobalToId);
                            }
                        }
                    }
                }
                
                if (GlobalAdjMask != 0)
                {
                    ConsolidatedCompactTacticalData.GlobalRegionAdjacency.Add(*GlobalFromId, GlobalAdjMask);
                }
            }
        }
        
        // Collect existing serialized visibility data
        if (!ChunkData.VisibilityMatrix.SparseReferences.IsEmpty())
        {
            UE_LOG(LogNav3D, VeryVerbose, TEXT("  Found %d visibility references in chunk"),
                ChunkData.VisibilityMatrix.SparseReferences.Num());
                
            const uint16 VolumeId = ChunkData.VolumeID;
            FVolumeRegionMatrix& ConsolidatedMatrix = 
                ConsolidatedCompactTacticalData.VolumeVisibilityData.FindOrAdd(VolumeId);
            
            // Copy serialized visibility data with ID remapping
            int32 VisibilityEntriesLoaded = 0;
            for (const auto& VisPair : ChunkData.VisibilityMatrix.SparseReferences)
            {
                const uint16 Key = VisPair.Key;
                const uint64 VisMask = VisPair.Value;
                
                // Decode the key to get local region ID
                // Using the decoding from FVolumeRegionMatrix
                const uint8 LocalRegionId = Key & 0x3F;
                const uint16 TargetVolumeId = Key >> 6;
                UE_LOG(LogNav3D, VeryVerbose, TEXT("LOAD: Original Key=0x%04X -> LocalRegionId=%d, TargetVolumeId=%d"), Key, LocalRegionId, TargetVolumeId);
                
                // Remap local region ID to global
                if (const uint16* GlobalRegionId = LocalToGlobalIdMap.Find(LocalRegionId))
                {
                    // Re-encode with global region ID
                    uint16 GlobalKey = (TargetVolumeId << 6) | (*GlobalRegionId & 0x3F);
                    ConsolidatedMatrix.SparseReferences.Add(GlobalKey, VisMask);
                    VisibilityEntriesLoaded++;
                }
            }
        }
        else
        {
            UE_LOG(LogNav3D, VeryVerbose, TEXT("  No visibility data in chunk %s"), *ChunkActor->GetName());
        }
        
        // Clear the local-to-global mapping for next chunk
        LocalToGlobalIdMap.Empty();
    }
    
    // Update source chunks
    ConsolidatedCompactTacticalData.SourceChunks.Empty();
    for (ANav3DDataChunkActor* ChunkActor : ChunksWithData)
    {
        ConsolidatedCompactTacticalData.SourceChunks.Add(ChunkActor);
    }
    
    UE_LOG(LogNav3D, Verbose, TEXT("Consolidated %d compact regions from %d chunks"),
        ConsolidatedCompactTacticalData.AllLoadedRegions.Num(), ChunksWithData.Num());
}

void ANav3DData::CleanUp()
{
	Super::CleanUp();
	ResetGenerator();
}

bool ANav3DData::NeedsRebuild() const
{
	// Check if any chunk actors need rebuilding
	for (const ANav3DDataChunkActor* ChunkActor : ChunkActors)
	{
		if (ChunkActor && ChunkActor->bNeedsRebuild)
		{
			return true;
		}
	}

	// Check if data generator has remaining tasks
	if (NavDataGenerator.IsValid())
	{
		return NavDataGenerator->GetNumRemaningBuildTasks() > 0;
	}
	
	return false;
}

void ANav3DData::EnsureBuildCompletion()
{
	Super::EnsureBuildCompletion();
	RecreateDefaultFilter();
}

bool ANav3DData::SupportsRuntimeGeneration() const
{
	return false;
}

bool ANav3DData::SupportsStreaming() const
{
	return (RuntimeGeneration != ERuntimeGenerationType::Dynamic);
}

FNavLocation ANav3DData::GetRandomPoint(FSharedConstNavQueryFilter, const UObject*) const
{
	FNavLocation Result;

	if (ChunkActors.Num() == 0)
	{
		return Result;
	}

	// Try to get a random point from any chunk actor
	for (const ANav3DDataChunkActor* ChunkActor : ChunkActors)
	{
		if (!ChunkActor) continue;
		
		for (const UNav3DDataChunk* Chunk : ChunkActor->Nav3DChunks)
		{
			if (!Chunk) continue;

			if (const FNav3DVolumeNavigationData* VolumeData = Chunk->GetVolumeNavigationData())
			{
				const TOptional<FNavLocation> RandomPoint = VolumeData->GetRandomPoint();
				if (RandomPoint.IsSet())
				{
					Result = RandomPoint.GetValue();
					return Result;
				}
			}
		}
	}

	return Result;
}

bool ANav3DData::GetRandomReachablePointInRadius(
	const FVector& Origin, const float Radius, FNavLocation& OutResult,
	FSharedConstNavQueryFilter Filter, const UObject* Querier) const
{
	if (Radius < 0.f)
	{
		return false;
	}

	// Find volume containing the origin
	if (const auto* NavData = GetVolumeNavigationDataContainingPoints({Origin}))
	{
		const auto NavAgentProps = FNav3DUtils::GetNavAgentPropsFromQuerier(Querier);
		const auto MinLayerIndex = NavData->GetMinLayerIndexForAgentSize(NavAgentProps.AgentRadius);

		// Get starting node
		FNav3DNodeAddress StartNodeAddress;
		if (!NavData->GetNodeAddressFromPosition(StartNodeAddress, Origin, MinLayerIndex))
		{
			return false;
		}

		// Keep track of valid nodes found
		TArray<FNav3DNodeAddress> ValidNodes;
		const float RadiusSq = FMath::Square(Radius);

		// Collect valid nodes within radius
		for (LayerIndex LayerIdx = 0; LayerIdx < NavData->GetData().GetLayerCount(); LayerIdx++)
		{
			const auto& Layer = NavData->GetData().GetLayer(LayerIdx);
			const float NodeExtent = Layer.GetNodeExtent();

			// Only check layers where nodes are smaller than our search radius
			if (NodeExtent > Radius)
			{
				continue;
			}

			for (NodeIndex NodeIdx = 0; NodeIdx < static_cast<uint32>(Layer.GetNodes().Num()); NodeIdx++)
			{
				const auto& Node = Layer.GetNode(NodeIdx);
				if (Node.HasChildren())
				{
					continue;
				}

				const FVector NodePos = NavData->GetNodePositionFromLayerAndMortonCode(LayerIdx, Node.MortonCode);

				// Check if node is within radius
				if (FVector::DistSquared(NodePos, Origin) <= RadiusSq)
				{
					// If leaf node, check sub-nodes
					if (LayerIdx == 0 && Node.FirstChild.IsValid())
					{
						const auto& LeafNode = NavData->GetData().GetLeafNodes().GetLeafNode(Node.FirstChild.NodeIndex);
						for (SubNodeIndex SubIdx = 0; SubIdx < 64; SubIdx++)
						{
							if (!LeafNode.IsSubNodeOccluded(SubIdx))
							{
								FNav3DNodeAddress SubAddress(0, NodeIdx, SubIdx);
								const FVector SubPos = NavData->GetNodePositionFromAddress(SubAddress, true);
								if (FVector::DistSquared(SubPos, Origin) <= RadiusSq)
								{
									ValidNodes.Add(SubAddress);
								}
							}
						}
					}
					else
					{
						ValidNodes.Add(FNav3DNodeAddress(LayerIdx, NodeIdx));
					}
				}
			}
		}

		// If we found valid nodes, pick one randomly
		if (ValidNodes.Num() > 0)
		{
			const int32 RandomIndex = FMath::RandHelper(ValidNodes.Num());
			const FNav3DNodeAddress& ChosenNode = ValidNodes[RandomIndex];
			const FVector RandomPoint = NavData->GetNodePositionFromAddress(ChosenNode, true);

			OutResult = FNavLocation(RandomPoint, ChosenNode.GetNavNodeRef());
			return true;
		}
	}

	return false;
}

bool ANav3DData::GetRandomPointInNavigableRadius(
	const FVector& Origin, const float Radius, FNavLocation& OutResult,
	const FSharedConstNavQueryFilter Filter, const UObject* Querier) const
{
	if (Radius < 0.f)
	{
		return false;
	}

	// Generate random point in radius
	const float RandomAngle = 2.f * PI * FMath::FRand();
	const float U = FMath::FRand() + FMath::FRand();
	const float RandomRadius = Radius * (U > 1 ? 2.f - U : U);
	const FVector RandomOffset(
		FMath::Cos(RandomAngle) * RandomRadius,
		FMath::Sin(RandomAngle) * RandomRadius,
		0
	);
	const FVector RandomPoint = Origin + RandomOffset;

	// Try to find volume containing both origin and random point
	if (const auto* NavData = GetVolumeNavigationDataContainingPoints({Origin, RandomPoint}))
	{
		const auto NavAgentProps = FNav3DUtils::GetNavAgentPropsFromQuerier(Querier);
		const auto MinLayerIndex = NavData->GetMinLayerIndexForAgentSize(NavAgentProps.AgentRadius);

		// Try to get node at random point
		FNav3DNodeAddress NodeAddress;
		if (NavData->GetNodeAddressFromPosition(NodeAddress, RandomPoint, MinLayerIndex))
		{
			OutResult = FNavLocation(RandomPoint, NodeAddress.GetNavNodeRef());
			return true;
		}

		// If direct point fails, try to find nearest navigable point
		const FVector ProjectionExtent(GetDefaultQueryExtent().X, GetDefaultQueryExtent().Y, BIG_NUMBER);
		return ProjectPoint(RandomPoint, OutResult, ProjectionExtent, Filter, Querier);
	}

	return false;
}

void ANav3DData::BatchRaycast(TArray<FNavigationRaycastWork>& Workload,
                              FSharedConstNavQueryFilter Filter,
                              const UObject* Querier) const
{
	if (Workload.Num() == 0)
	{
		return;
	}

	const auto* Raycaster = NewObject<UNav3DRaycaster>();
	if (!Raycaster)
	{
		return;
	}

	// Process each raycast request
	for (FNavigationRaycastWork& Work : Workload)
	{
		// Find the volume containing both points
		if (const auto* NavData = GetVolumeNavigationDataContainingPoints({Work.RayStart, Work.RayEnd}))
		{
			// Perform the raycast
			if (FNav3DRaycastHit Hit; Raycaster->Trace(*NavData, Work.RayStart, Work.RayEnd, Hit))
			{
				Work.bDidHit = true;
				Work.HitLocation = FNavLocation(Hit.ImpactPoint, Hit.NodeAddress.GetNavNodeRef());
			}
		}
	}
}

bool ANav3DData::FindMoveAlongSurface(const FNavLocation& StartLocation,
                                      const FVector& TargetPosition,
                                      FNavLocation& OutLocation,
                                      FSharedConstNavQueryFilter Filter,
                                      const UObject* Querier) const
{
	// Get the volume containing both points
	if (const auto* NavData = GetVolumeNavigationDataContainingPoints(
		{StartLocation.Location, TargetPosition}))
	{
		// Direction to target
		const FVector MoveDirection = (TargetPosition - StartLocation.Location).GetSafeNormal();
		const float DistanceToTarget = FVector::Dist(StartLocation.Location, TargetPosition);

		// Start from current node
		FNav3DNodeAddress CurrentNode(StartLocation.NodeRef);
		FVector CurrentPos = StartLocation.Location;

		const float StepSize = NavData->GetData().GetLeafNodes().GetLeafNodeSize();
		float DistanceMoved = 0.0f;

		const auto NavAgentProps = FNav3DUtils::GetNavAgentPropsFromQuerier(Querier);
		const auto MinLayerIndex = NavData->GetMinLayerIndexForAgentSize(NavAgentProps.AgentRadius);

		while (DistanceMoved < DistanceToTarget)
		{
			// Try to move in target direction
			const FVector NextPos = CurrentPos + MoveDirection * StepSize;

			// Check if next position is navigable
			FNav3DNodeAddress NextNode;
			if (!NavData->GetNodeAddressFromPosition(NextNode, NextPos, MinLayerIndex))
			{
				// Hit non-navigable area - return last valid position
				OutLocation = FNavLocation(CurrentPos, CurrentNode.GetNavNodeRef());
				return true;
			}

			// Move to next position
			CurrentPos = NextPos;
			CurrentNode = NextNode;
			DistanceMoved += StepSize;
		}

		// Reached target
		OutLocation = FNavLocation(CurrentPos, CurrentNode.GetNavNodeRef());
		return true;
	}

	return false;
}

bool ANav3DData::ProjectPoint(const FVector& Point,
                              FNavLocation& OutLocation,
                              const FVector& Extent,
                              FSharedConstNavQueryFilter Filter,
                              const UObject* Querier) const
{
	// Try to find a volume containing the point
	if (const auto* NavData = GetVolumeNavigationDataContainingPoints({Point}))
	{
		const auto NavAgentProps = FNav3DUtils::GetNavAgentPropsFromQuerier(Querier);
		const auto MinLayerIndex = NavData->GetMinLayerIndexForAgentSize(NavAgentProps.AgentRadius);

		// Try to get node at point location first
		FNav3DNodeAddress NodeAddress;
		if (NavData->GetNodeAddressFromPosition(NodeAddress, Point, MinLayerIndex))
		{
			OutLocation = FNavLocation(Point, NodeAddress.GetNavNodeRef());
			return true;
		}

		// If not found, search within extent
		const float ExtentSize = Extent.GetAbsMax();
		const auto& Data = NavData->GetData();

		// Start from current layer and work up
		for (LayerIndex LayerIdx = 0; LayerIdx < Data.GetLayerCount(); LayerIdx++)
		{
			const auto& Layer = Data.GetLayer(LayerIdx);
			const float NodeExtent = Layer.GetNodeExtent();

			// Only check layers where nodes are smaller than our search extent
			if (NodeExtent > ExtentSize)
			{
				continue;
			}

			// Check nodes near point
			for (NodeIndex NodeIdx = 0; NodeIdx < static_cast<uint32>(Layer.GetNodes().Num()); NodeIdx++)
			{
				const auto& Node = Layer.GetNode(NodeIdx);
				const FVector NodePos = NavData->GetNodePositionFromLayerAndMortonCode(LayerIdx, Node.MortonCode);

				// If node is within extent of point
				if (FVector::DistSquared(NodePos, Point) <= ExtentSize * ExtentSize)
				{
					if (!Node.HasChildren())
					{
						// Found valid node
						OutLocation = FNavLocation(NodePos, FNav3DNodeAddress(LayerIdx, NodeIdx).GetNavNodeRef());
						return true;
					}
				}
			}
		}
	}

	return false;
}

void ANav3DData::BatchProjectPoints(TArray<FNavigationProjectionWork>& Workload,
                                    const FVector& Extent,
                                    const FSharedConstNavQueryFilter Filter,
                                    const UObject* Querier) const
{
	if (Workload.Num() == 0)
	{
		return;
	}

	// Process each projection request
	for (auto& Work : Workload)
	{
		Work.bResult = ProjectPoint(Work.Point, Work.OutLocation, Extent, Filter, Querier);
	}
}

void ANav3DData::BatchProjectPoints(TArray<FNavigationProjectionWork>& Workload,
                                    const FSharedConstNavQueryFilter Filter,
                                    const UObject* Querier) const
{
	if (Workload.Num() == 0)
	{
		return;
	}

	// Process each projection request using their individual limits
	for (auto& Work : Workload)
	{
		if (Work.ProjectionLimit.IsValid)
		{
			Work.bResult = ProjectPoint(Work.Point, Work.OutLocation,
			                            Work.ProjectionLimit.GetExtent(), Filter, Querier);
		}
	}
}

ENavigationQueryResult::Type
ANav3DData::CalcPathCost(const FVector& PathStart, const FVector& PathEnd,
                         FVector::FReal& OutPathCost,
                         const FSharedConstNavQueryFilter Filter,
                         const UObject* Querier) const
{
	FVector::FReal PathLength = 0.f;
	return CalcPathLengthAndCost(PathStart, PathEnd, PathLength, OutPathCost, Filter, Querier);
}

ENavigationQueryResult::Type ANav3DData::CalcPathLength(const FVector& PathStart, const FVector& PathEnd,
                                                        FVector::FReal& OutPathLength,
                                                        const FSharedConstNavQueryFilter Filter,
                                                        const UObject* Querier) const
{
	FVector::FReal PathCost = 0.f;
	return CalcPathLengthAndCost(PathStart, PathEnd, OutPathLength, PathCost, Filter, Querier);
}

bool ANav3DData::DoesNodeContainLocation(const NavNodeRef NodeRef, const FVector& WorldSpaceLocation) const
{
	const FNav3DNodeAddress NodeAddress(NodeRef);
	if (!NodeAddress.IsValid())
	{
		return false;
	}

	if (const auto* NavData = GetVolumeNavigationDataContainingPoints({WorldSpaceLocation}))
	{
		const FVector NodePosition = NavData->GetNodePositionFromAddress(NodeAddress, true);
		const float NodeExtent = NavData->GetNodeExtentFromNodeAddress(NodeAddress);
		const FBox NodeBox = FBox::BuildAABB(NodePosition, FVector(NodeExtent));
		return NodeBox.IsInside(WorldSpaceLocation);
	}

	return false;
}

UPrimitiveComponent* ANav3DData::ConstructRenderingComponent()
{
	return NewObject<UNav3DNavDataRenderingComponent>(
		this, TEXT("Nav3DNavRenderingComp"), RF_Transient);
}

void ANav3DData::OnStreamingLevelAdded(ULevel* Level, UWorld*)
{
	QUICK_SCOPE_CYCLE_COUNTER(STAT_RecastNavMesh_OnStreamingLevelAdded);

	if (SupportsStreaming())
	{
		// In the new chunk-based system, streaming levels are handled by chunk actors
		// This method is kept for compatibility but doesn't need to do anything
		// as chunk actors are managed separately
		UE_LOG(LogNav3D, Verbose, TEXT("Nav3D: Streaming level added - handled by chunk actors"));
	}
}

void ANav3DData::OnStreamingLevelRemoved(ULevel* Level, UWorld*)
{
	QUICK_SCOPE_CYCLE_COUNTER(STAT_RecastNavMesh_OnStreamingLevelRemoved);

	if (SupportsStreaming())
	{
		// In the new chunk-based system, streaming levels are handled by chunk actors
		// This method is kept for compatibility but doesn't need to do anything
		// as chunk actors are managed separately
		UE_LOG(LogNav3D, Verbose, TEXT("Nav3D: Streaming level removed - handled by chunk actors"));
	}
}

void ANav3DData::OnNavAreaChanged() { Super::OnNavAreaChanged(); }

void ANav3DData::OnNavAreaAdded(const UClass* NavAreaClass, const int32 AgentIndex)
{
	Super::OnNavAreaAdded(NavAreaClass, AgentIndex);
}

int32 ANav3DData::GetNewAreaID(const UClass* NavAreaClass) const
{
	return Super::GetNewAreaID(NavAreaClass);
}

int32 ANav3DData::GetMaxSupportedAreas() const { return 32; }

bool ANav3DData::IsNodeRefValid(const NavNodeRef NodeRef) const
{
	return FNav3DNodeAddress(NodeRef).IsValid();
}

void ANav3DData::TickActor(const float DeltaTime, const ELevelTick Tick, FActorTickFunction& ThisTickFunction)
{
	Super::TickActor(DeltaTime, Tick, ThisTickFunction);
}

#if WITH_EDITOR

/*
 * Return true if any of the named properties report to have been updated.
 */
bool ANav3DData::NeedsTacticalRebuild(const FPropertyChangedEvent& PropertyChangedEvent)
{
	// List of property names that require rebuild
	static const TArray RebuildPropertyNames = {
		GET_MEMBER_NAME_CHECKED(FNav3DTacticalSettings, bEnableTacticalReasoning),
		GET_MEMBER_NAME_CHECKED(FNav3DTacticalSettings, MinRegioningLayer),
		GET_MEMBER_NAME_CHECKED(FNav3DTacticalSettings, MaxRegioningLayer),
		GET_MEMBER_NAME_CHECKED(FNav3DTacticalSettings, MinSamplesPerRegion),
		GET_MEMBER_NAME_CHECKED(FNav3DTacticalSettings, MaxSamplesPerRegion),
		GET_MEMBER_NAME_CHECKED(FNav3DTacticalSettings, RegionSampleDensityFactor),
		GET_MEMBER_NAME_CHECKED(FNav3DTacticalSettings, VisibilityScoreThreshold),
		GET_MEMBER_NAME_CHECKED(FNav3DTacticalSettings, MinOcclusions),
		GET_MEMBER_NAME_CHECKED(FNav3DTacticalSettings, MaxCoverSearchDistance),
		GET_MEMBER_NAME_CHECKED(FNav3DTacticalSettings, MaxCoverRaycasts)
	};

	const FName PropertyName = PropertyChangedEvent.Property->GetFName();
    
	// Check if the property name is in our list
	return RebuildPropertyNames.Contains(PropertyName);
}

void ANav3DData::PostEditChangeProperty(
	FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);

	if (PropertyChangedEvent.Property == nullptr)
	{
		return;
	}

	if (PropertyChangedEvent.Property != nullptr)
	{
		const FName CategoryName = FObjectEditorUtils::GetCategoryFName(PropertyChangedEvent.Property);
		static const FName NameGeneration = FName(TEXT("Generation"));
		static const FName NameQuery = FName(TEXT("Query"));

		if (CategoryName == NameGeneration)
		{
			if (!HasAnyFlags(RF_ClassDefaultObject) )
			{
				RebuildAll();
			}
		}
		else if (CategoryName == NameQuery)
		{
			RecreateDefaultFilter();
		}

		// Check if tactical settings changed
		if (PropertyChangedEvent.Property) 
		{
			if (NeedsTacticalRebuild(PropertyChangedEvent) && TacticalSettings.bEnableTacticalReasoning)
			{
				UE_LOG(LogNav3D, Display, TEXT("Tactical rebuild requested (deferred)"));
				bNeedsTacticalRebuild = true;
				if (const UWorld* World = GetWorld())
				{
					FTimerManager& TimerManager = World->GetTimerManager();
					TimerManager.SetTimer(DeferredTacticalRebuildHandle,
						FTimerDelegate::CreateUObject(this, &ANav3DData::PerformDeferredTacticalRefresh),
						0.1f, false);
				}
			}
		}
	}
}

bool ANav3DData::ShouldExport() { return false; }
#endif

#if !UE_BUILD_SHIPPING
uint32 ANav3DData::LogMemUsed() const
{
	const auto SuperMemUsed = Super::LogMemUsed();

	auto NavigationMemSize = 0;
	for (ANav3DDataChunkActor* ChunkActor : ChunkActors)
	{
		if (!ChunkActor) continue;
		
		for (const UNav3DDataChunk* Chunk : ChunkActor->Nav3DChunks)
		{
			if (!Chunk) continue;

			if (const FNav3DVolumeNavigationData* NavBoundsData = Chunk->GetVolumeNavigationData())
			{
				const auto OctreeDataMemSize = NavBoundsData->GetData().GetAllocatedSize();
				NavigationMemSize += OctreeDataMemSize;
			}
		}
	}
	const auto MemUsed = SuperMemUsed + NavigationMemSize;

	UE_LOG(LogNav3D, Warning, TEXT("%s: ANav3DData: %u, self: %llu"),
	       *GetName(), MemUsed, sizeof(ANav3DData));

	return MemUsed;
}
#endif

void ANav3DData::ConditionalConstructGenerator()
{
	ResetGenerator();

	const UWorld* World = GetWorld();
	check(World);
	const bool RequiresGenerator =
		SupportsRuntimeGeneration() || !World->IsGameWorld();

	if (!RequiresGenerator)
	{
		return;
	}

	if (FNav3DDataGenerator* Generator = new FNav3DDataGenerator(*this))
	{
		NavDataGenerator =
			MakeShareable(static_cast<FNavDataGenerator*>(Generator));
		Generator->Init();
	}
}

void ANav3DData::RequestDrawingUpdate(const bool Force)
{
#if !UE_BUILD_SHIPPING
	if (Force ||
		UNav3DNavDataRenderingComponent::IsNavigationShowFlagSet(GetWorld()))
	{
		if (Force)
		{
			if (UNav3DNavDataRenderingComponent* RenderingComponent =
				Cast<UNav3DNavDataRenderingComponent>(RenderingComp))
			{
				RenderingComponent->ForceUpdate();
			}
		}

		DECLARE_CYCLE_STAT(
			TEXT("FSimpleDelegateGraphTask.Requesting Nav3D navmesh redraw"),
			STAT_FSimpleDelegateGraphTask_RequestingNavmeshRedraw,
			STATGROUP_TaskGraphTasks);

		FSimpleDelegateGraphTask::CreateAndDispatchWhenReady(
			FSimpleDelegateGraphTask::FDelegate::CreateUObject(
				this, &ANav3DData::UpdateDrawing),
			GET_STATID(STAT_FSimpleDelegateGraphTask_RequestingNavmeshRedraw),
			nullptr, ENamedThreads::GameThread);
	}
#endif // !UE_BUILD_SHIPPING
}

FBox ANav3DData::GetBoundingBox() const
{
	FBox BoundingBox(ForceInit);

	for (const ANav3DDataChunkActor* ChunkActor : ChunkActors)
	{
		if (ChunkActor)
		{
			BoundingBox += ChunkActor->DataChunkActorBounds;
		}
	}

	return BoundingBox;
}




void ANav3DData::CheckToDiscardSubLevelNavData(
	const UNavigationSystemBase& NavigationSystem)
{
	if (const auto* World = GetWorld())
	{
		if (Cast<UNavigationSystemV1>(&NavigationSystem))
		{
			if (GEngine->IsSettingUpPlayWorld() == false
				&& (World->PersistentLevel != GetLevel())
				&& (IsRunningCommandlet() == false))
			{
				UE_LOG(LogNav3D, Verbose,
				       TEXT("%s Discarding %s due to it not being part of PersistentLevel."),
				       // ReSharper disable once CppPrintfBadFormat
				       ANSI_TO_TCHAR(__FUNCTION__), *GetFullNameSafe(this));

				// Marking self for deletion
				CleanUpAndMarkPendingKill();
			}
		}
	}
}

void ANav3DData::RecreateDefaultFilter() const
{
	DefaultQueryFilter->SetFilterType<FNav3DQueryFilter>();
}

void ANav3DData::UpdateDrawing() const
{
#if !UE_BUILD_SHIPPING
	if (UNav3DNavDataRenderingComponent* RenderingComponent =
		Cast<UNav3DNavDataRenderingComponent>(RenderingComp))
	{
		if (RenderingComponent->GetVisibleFlag() &&
			(RenderingComponent->UpdateIsForced() ||
				UNav3DNavDataRenderingComponent::IsNavigationShowFlagSet(
					GetWorld())))
		{
			RenderingComponent->MarkRenderStateDirty();
		}
	}
#endif
}

void ANav3DData::ResetGenerator(const bool CancelBuild)
{
	if (NavDataGenerator.IsValid())
	{
		if (CancelBuild)
		{
			NavDataGenerator->CancelBuild();
		}

		NavDataGenerator.Reset();
	}
}

void ANav3DData::OnNavigationDataUpdatedInBounds(
	const TArray<FBox>& UpdatedBounds)
{
	InvalidateAffectedPaths(UpdatedBounds);
}

void ANav3DData::ClearNavigationData()
{
	ChunkActors.Reset();
	
	RequestDrawingUpdate();
}

void ANav3DData::Analyse() const
{
    LogSectionHeader(TEXT("NAV3D ANALYSIS"));
    
    UWorld* World = GetWorld();
    if (!World)
    {
        UE_LOG(LogNav3D, Log, TEXT("No valid world"));
        LogSectionFooter();
        return;
    }

    // Discover volumes from the world
    TArray<FBox> AnalysisBounds;
    
    if (ChunkActors.Num() == 0)
    {
        LogSectionHeader(TEXT("VOLUME DISCOVERY"));
        
        // Find all Nav3DBoundsVolume actors in the world
        for (TActorIterator<ANav3DBoundsVolume> ActorIterator(World); ActorIterator; ++ActorIterator)
        {
            ANav3DBoundsVolume* BoundsVolume = *ActorIterator;
            if (BoundsVolume && IsValid(BoundsVolume))
            {
	            if (const FBox VolumeBounds = BoundsVolume->GetComponentsBoundingBox(true);
	            	VolumeBounds.IsValid)
                {
                    AnalysisBounds.Add(VolumeBounds);
                    UE_LOG(LogNav3D, Log, TEXT("Discovered bounds volume: %s"), *VolumeBounds.ToString());
                }
            }
        }
        
        // If still no bounds found, use navigation system bounds
        if (AnalysisBounds.Num() == 0)
        {
            if (UNavigationSystemV1* NavSys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(World))
            {
                TArray<FBox> SupportedNavigationBounds;
                NavSys->GetNavigationBoundsForNavData(*this, SupportedNavigationBounds);
                AnalysisBounds = SupportedNavigationBounds;
                UE_LOG(LogNav3D, Log, TEXT("Using navigation system bounds: %d volumes"), AnalysisBounds.Num());
            }
        }
        
        if (AnalysisBounds.Num() == 0)
        {
            UE_LOG(LogNav3D, Log, TEXT("Analyse: No volumes found in world"));
            return;
        }
    }
    else
    {
        // Use existing navigation data bounds
        for (ANav3DDataChunkActor* ChunkActor : ChunkActors)
        {
            if (!ChunkActor) continue;
            
            for (UNav3DDataChunk* Chunk : ChunkActor->Nav3DChunks)
            {
                if (!Chunk) continue;

                if (const FNav3DVolumeNavigationData* Volume = Chunk->GetVolumeNavigationData())
                {
                    AnalysisBounds.Add(Volume->GetVolumeBounds());
                }
            }
        }
    }

    LogSectionHeader(TEXT("OBJECT FILTERING"));
    UE_LOG(LogNav3D, Log, TEXT("Volumes: %s, Collision Channel: %d"), *FormatNumber(AnalysisBounds.Num()), static_cast<int32>(GenerationSettings.CollisionChannel));

    TArray<int32> CandidateCounts;
    CandidateCounts.SetNum(AnalysisBounds.Num());

    for (int32 VolumeIdx = 0; VolumeIdx < AnalysisBounds.Num(); ++VolumeIdx)
    {
        const FBox& Bounds = AnalysisBounds[VolumeIdx];

        TArray<FOverlapResult> Overlaps;
        const bool _ = World->OverlapMultiByChannel(
            Overlaps,
            Bounds.GetCenter(),
            FQuat::Identity,
            GenerationSettings.CollisionChannel,
            FCollisionShape::MakeBox(Bounds.GetExtent()),
            GenerationSettings.CollisionQueryParameters
        );

		// Counters
		int32 Total = Overlaps.Num();
    	int32 Kept = 0;
    	int32 RemovedInvalid = 0;
    	int32 RemovedNoAffectNav = 0;
    	int32 RemovedCollisionOnly = 0;
    	int32 RemovedStaticNoGeom = 0;
    	int32 RemovedIsmNoGeom = 0;
    	int32 KeptLandscape = 0;
    	int32 KeptStaticWithGeom = 0;
    	int32 KeptIsmWithGeom = 0;
    	int32 KeptOther = 0;

    	// ISM breakdown stats
    	int32 Ism_Total = 0;
    	int32 Ism_NoCollision = 0;
    	int32 Ism_QueryOnly = 0;
    	int32 Ism_QueryAndPhysics = 0;
    	int32 Ism_PhysicsOnly = 0;
    	int32 Ism_Response_Ignore = 0;
    	int32 Ism_Response_Overlap = 0;
    	int32 Ism_Response_Block = 0;
    	int32 Ism_AggGeom_Any = 0;
    	int32 Ism_AggGeom_None = 0;
    	int32 Ism_Trace_Default = 0;
    	int32 Ism_Trace_SimpleAsComplex = 0;
    	int32 Ism_Trace_ComplexAsSimple = 0;

		// Optional detailed listing (throttled)
		int32 DetailedPrinted = 0;

        for (const FOverlapResult& Result : Overlaps)
		{
			if (!Result.Component.IsValid())
			{
				RemovedInvalid++;
				continue;
			}

			UPrimitiveComponent* Prim = Result.Component.Get();
			if (!Prim || !IsValid(Prim))
			{
				RemovedInvalid++;
				continue;
			}

			if (!Prim->CanEverAffectNavigation())
			{
				RemovedNoAffectNav++;
				continue;
			}

			// Local helper: collision-only shapes (similar to GatherOverlappingObjects)
			auto IsCollisionOnly = [](const UPrimitiveComponent* Component) -> bool
			{
				return Component && (
					Component->IsA<USphereComponent>() ||
					Component->IsA<UBoxComponent>() ||
					Component->IsA<UCapsuleComponent>()
				);
			};

			if (IsCollisionOnly(Prim))
			{
				RemovedCollisionOnly++;
				continue;
			}

			// Landscapes are always kept
			if (Prim->IsA<ULandscapeHeightfieldCollisionComponent>() || Prim->IsA<ULandscapeMeshCollisionComponent>())
			{
				KeptLandscape++;
				Kept++;
				continue;
			}

			// ISM handling: require collision enabled, instances present, and body setup geometry
			if (const UInstancedStaticMeshComponent* ISM = Cast<UInstancedStaticMeshComponent>(Prim))
			{
				Ism_Total++;
				const ECollisionEnabled::Type CE = ISM->GetCollisionEnabled();
				if (CE == ECollisionEnabled::NoCollision) { Ism_NoCollision++; }
				else if (CE == ECollisionEnabled::QueryOnly) { Ism_QueryOnly++; }
				else if (CE == ECollisionEnabled::QueryAndPhysics) { Ism_QueryAndPhysics++; }
				else if (CE == ECollisionEnabled::PhysicsOnly) { Ism_PhysicsOnly++; }

				const ECollisionResponse Resp = ISM->GetCollisionResponseToChannel(GenerationSettings.CollisionChannel);
				if (Resp == ECR_Ignore) { Ism_Response_Ignore++; }
				else if (Resp == ECR_Overlap) { Ism_Response_Overlap++; }
				else if (Resp == ECR_Block) { Ism_Response_Block++; }

				bool bHasGeom = false;
				ECollisionTraceFlag TraceFlag = CTF_UseDefault;
				if (ISM->GetStaticMesh())
				{
					if (UBodySetup* BodySetup = ISM->GetStaticMesh()->GetBodySetup())
					{
						const FKAggregateGeom& Agg = BodySetup->AggGeom;
						bHasGeom = (Agg.ConvexElems.Num() > 0 || Agg.BoxElems.Num() > 0 || Agg.SphereElems.Num() > 0 || Agg.SphylElems.Num() > 0 || Agg.TaperedCapsuleElems.Num() > 0);
						TraceFlag = BodySetup->CollisionTraceFlag;
					}
				}
				if (bHasGeom) { Ism_AggGeom_Any++; } else { Ism_AggGeom_None++; }
				if (TraceFlag == CTF_UseDefault) { Ism_Trace_Default++; }
				else if (TraceFlag == CTF_UseSimpleAsComplex) { Ism_Trace_SimpleAsComplex++; }
				else if (TraceFlag == CTF_UseComplexAsSimple) { Ism_Trace_ComplexAsSimple++; }
				if (bHasGeom)
				{
					KeptIsmWithGeom++;
					Kept++;
				}
				else
				{
					RemovedIsmNoGeom++;
				}

				if (constexpr int32 DetailedMax = 25; DetailedPrinted < DetailedMax)
				{
					const FString MeshName = ISM->GetStaticMesh() ? ISM->GetStaticMesh()->GetName() : TEXT("<None>");
                    UE_LOG(LogNav3D, Log, TEXT("   ISM: %s | Mesh=%s | Instances=%s"), 
                        *GetSimplifiedComponentName(Prim), *MeshName, *FormatNumber(ISM->GetInstanceCount()));
					DetailedPrinted++;
				}
				continue;
			}

			// Static mesh component handling
			if (const UStaticMeshComponent* SMC = Cast<UStaticMeshComponent>(Prim))
			{
				bool bHasGeom = false;
				if (SMC->GetCollisionEnabled() != ECollisionEnabled::NoCollision && SMC->GetStaticMesh())
				{
					if (UBodySetup* BodySetup = SMC->GetStaticMesh()->GetBodySetup())
					{
						const FKAggregateGeom& Agg = BodySetup->AggGeom;
						bHasGeom = (Agg.ConvexElems.Num() > 0 || Agg.BoxElems.Num() > 0 || Agg.SphereElems.Num() > 0 || Agg.SphylElems.Num() > 0 || Agg.TaperedCapsuleElems.Num() > 0);
					}
				}
				if (bHasGeom)
				{
					KeptStaticWithGeom++;
					Kept++;
				}
				else
				{
					RemovedStaticNoGeom++;
				}
				continue;
			}

			// Default: keep other nav-affecting components
			KeptOther++;
			Kept++;
		}

		const int32 Removed = Total - Kept;
		const float ReductionPct = Total > 0 ? (100.0f * Removed / static_cast<float>(Total)) : 0.0f;
		const float KeptPct = Total > 0 ? (100.0f * Kept / static_cast<float>(Total)) : 0.0f;

        UE_LOG(LogNav3D, Log, TEXT("Volume %d: %s"), VolumeIdx, *Bounds.ToString());
        UE_LOG(LogNav3D, Log, TEXT("  Objects: %s total, %s kept (%.1f%%), %s removed (%.1f%%)"), 
            *FormatNumber(Total), *FormatNumber(Kept), KeptPct, *FormatNumber(Removed), ReductionPct);
        UE_LOG(LogNav3D, Log, TEXT("  Kept: Landscape=%s, StaticMesh=%s, ISM=%s, Other=%s"),
            *FormatNumber(KeptLandscape), *FormatNumber(KeptStaticWithGeom), *FormatNumber(KeptIsmWithGeom), *FormatNumber(KeptOther));

        // ISM breakdown (only if significant)
		if (Ism_Total > 0)
		{
            UE_LOG(LogNav3D, Log, TEXT("  ISM: %s total, %s with geometry, %s without geometry"), 
                *FormatNumber(Ism_Total), *FormatNumber(Ism_AggGeom_Any), *FormatNumber(Ism_AggGeom_None));
        }

        CandidateCounts[VolumeIdx] = Kept;
        
        // Perform spatial analysis for this volume if we have enough objects
        if (Kept > 10 && Overlaps.Num() > 0)
        {
            LogSectionHeader(TEXT("SPATIAL ANALYSIS"));
            AnalyzeActualSpatialDistribution(Bounds, Overlaps);
        }
    }

    LogSectionFooter();
}

void ANav3DData::AnalyzeActualSpatialDistribution(const FBox& VolumeBounds, const TArray<FOverlapResult>& OverlappingObjects)
{
    QUICK_SCOPE_CYCLE_COUNTER(STAT_Nav3D_AnalyzeSpatialDistribution);
    
    if (OverlappingObjects.Num() == 0) return;
    
    // Performance optimization: Skip detailed analysis for very large datasets
    constexpr int32 MaxObjectsForDetailedAnalysis = 100000;
    if (OverlappingObjects.Num() > MaxObjectsForDetailedAnalysis)
    {
        UE_LOG(LogNav3D, Warning, TEXT("Skipping detailed spatial analysis for %d objects (too large). Use smaller volumes or reduce object count for detailed analysis."), OverlappingObjects.Num());
        return;
    }
    
    UE_LOG(LogNav3D, Log, TEXT("Analyzing spatial distribution of %s objects..."), *FormatNumber(OverlappingObjects.Num()));
    
    // Collect all object positions efficiently
    TArray<FVector> ObjectPositions;
    TArray<FBox> ObjectBounds;
    ObjectPositions.Reserve(OverlappingObjects.Num() * 10); // Reserve for potential ISM instances
    ObjectBounds.Reserve(OverlappingObjects.Num());
    
    for (const auto& Overlap : OverlappingObjects)
    {
        const UPrimitiveComponent* Component = Overlap.GetComponent();
        if (!Component) continue;
        
        // Store component bounds for coverage analysis
        ObjectBounds.Add(Component->Bounds.GetBox());
        
        if (const UInstancedStaticMeshComponent* ISM = Cast<UInstancedStaticMeshComponent>(Component))
        {
            // For ISMs, sample instance positions for performance (max 1000 per ISM)
            const int32 InstanceCount = ISM->GetInstanceCount();
            const int32 SampleCount = FMath::Min(InstanceCount, 1000);
            const int32 SampleStep = FMath::Max(1, InstanceCount / SampleCount);
            
            for (int32 i = 0; i < InstanceCount; i += SampleStep)
            {
                FTransform InstanceTransform;
                if (ISM->GetInstanceTransform(i, InstanceTransform, true))
                {
                    ObjectPositions.Add(InstanceTransform.GetLocation());
                }
            }
        }
        else if (const UStaticMeshComponent* SMC = Cast<UStaticMeshComponent>(Component))
        {
            // Regular static mesh - just one position
            ObjectPositions.Add(SMC->GetComponentLocation());
        }
        else
        {
            // Fallback for other component types
            ObjectPositions.Add(Component->GetComponentLocation());
        }
    }
    
    UE_LOG(LogNav3D, Log, TEXT("Collected %s sampled object positions from %s components"), 
           *FormatNumber(ObjectPositions.Num()), *FormatNumber(OverlappingObjects.Num()));
    UE_LOG(LogNav3D, Log, TEXT("Note: ISM instances are sampled (max 1000 per component) for performance"));
    
    // Now analyze the distribution
    AnalyzeSpatialClustering(ObjectPositions, ObjectBounds, VolumeBounds, OverlappingObjects.Num());
}

void ANav3DData::AnalyzeSpatialClustering(const TArray<FVector>& ObjectPositions, const TArray<FBox>& ObjectBounds, const FBox& VolumeBounds, const int32 NumCandidateObjects)
{
    if (ObjectPositions.Num() < 10) return;
    
    // 1. Calculate spatial statistics
    FVector CenterOfMass = FVector::ZeroVector;
    for (const FVector& Pos : ObjectPositions)
    {
        CenterOfMass += Pos;
    }
    CenterOfMass /= ObjectPositions.Num();
    
    // 2. Calculate average distance from center (clustering measure)
    float TotalDistanceFromCenter = 0.0f;
    float MaxDistanceFromCenter = 0.0f;
    for (const FVector& Pos : ObjectPositions)
    {
        const float Distance = FVector::Dist(Pos, CenterOfMass);
        TotalDistanceFromCenter += Distance;
        MaxDistanceFromCenter = FMath::Max(MaxDistanceFromCenter, Distance);
    }

    // 3. Calculate volume utilization from component bounds
    float TotalComponentVolume = 0.0f;
    for (const FBox& Bounds : ObjectBounds)
    {
        TotalComponentVolume += Bounds.GetVolume();
    }

    // 4. Grid-based density analysis
    constexpr int32 AnalysisGridSize = 20; // 20x20x20 = 8000 cells for analysis
    TArray<int32> GridCounts;
    GridCounts.SetNumZeroed(AnalysisGridSize * AnalysisGridSize * AnalysisGridSize);
    
    const FVector GridCellSize = VolumeBounds.GetSize() / AnalysisGridSize;
    const FVector VolumeMin = VolumeBounds.Min;
    
    for (const FVector& Pos : ObjectPositions)
    {
        // Convert position to grid coordinates
        const FVector RelativePos = Pos - VolumeMin;
        const int32 X = FMath::Clamp(FMath::FloorToInt(RelativePos.X / GridCellSize.X), 0, AnalysisGridSize - 1);
        const int32 Y = FMath::Clamp(FMath::FloorToInt(RelativePos.Y / GridCellSize.Y), 0, AnalysisGridSize - 1);
        const int32 Z = FMath::Clamp(FMath::FloorToInt(RelativePos.Z / GridCellSize.Z), 0, AnalysisGridSize - 1);
        
        const int32 GridIndex = X + Y * AnalysisGridSize + Z * AnalysisGridSize * AnalysisGridSize;
        GridCounts[GridIndex]++;
    }
    
    // 5. Calculate grid statistics
    int32 NonEmptyGridCells = 0;
    int32 MaxObjectsInCell = 0;
    float TotalObjectsInNonEmptyCells = 0.0f;
    
    for (const int32 Count : GridCounts)
    {
        if (Count > 0)
        {
            NonEmptyGridCells++;
            TotalObjectsInNonEmptyCells += Count;
            MaxObjectsInCell = FMath::Max(MaxObjectsInCell, Count);
        }
    }
    
    const float EmptyGridRatio = 1.0f - (static_cast<float>(NonEmptyGridCells) / GridCounts.Num());
    const float AvgObjectsPerNonEmptyCell = NonEmptyGridCells > 0 ? TotalObjectsInNonEmptyCells / NonEmptyGridCells : 0.0f;
    const float DensityVariance = MaxObjectsInCell / FMath::Max(1.0f, AvgObjectsPerNonEmptyCell);
    
    // Log actionable spatial summary
    UE_LOG(LogNav3D, Log, TEXT(""));
    UE_LOG(LogNav3D, Log, TEXT("=== SPATIAL SUMMARY ==="));
    UE_LOG(LogNav3D, Log, TEXT("Objects: %s | Empty Space: %d%% | Clustering: %s"), 
        *FormatNumber(NumCandidateObjects), 
        FMath::RoundToInt(EmptyGridRatio * 100),
        DensityVariance > 5.0f ? TEXT("Heavy") : DensityVariance > 2.0f ? TEXT("Moderate") : TEXT("Light"));
    UE_LOG(LogNav3D, Log, TEXT(""));
}


void ANav3DData::EstimateOctreeSize(const FBox& VolumeBounds, const float EmptyGridRatio, const int32 MaxLayers, const float LeafNodeSize)
{
    UE_LOG(LogNav3D, Log, TEXT("=== OCTREE SIZE ESTIMATION ==="));
    
    const FVector Size = VolumeBounds.GetSize();
    const float VolumeDensity = 1.0f - EmptyGridRatio; // Convert empty ratio to density
    
    UE_LOG(LogNav3D, Log, TEXT("Volume Density: %.1f%% (%.1f%% empty space)"), 
           VolumeDensity * 100.0f, EmptyGridRatio * 100.0f);
    
    // Estimate total voxels that will be generated (non-empty voxels only)
    int64 TotalEstimatedVoxels = 0;
    int64 TotalEstimatedNodes = 0;
    int64 TotalEstimatedBytes = 0;
    
    for (int32 Layer = 0; Layer < MaxLayers; Layer++)
    {
        const float NodeSize = LeafNodeSize * FMath::Pow(2.0f, Layer);
        const int32 Nx = FMath::Max(1, FMath::CeilToInt(Size.X / NodeSize));
        const int32 Ny = FMath::Max(1, FMath::CeilToInt(Size.Y / NodeSize));
        const int32 Nz = FMath::Max(1, FMath::CeilToInt(Size.Z / NodeSize));
        const int32 TotalNodesAtLayer = Nx * Ny * Nz;
        
        // Estimate non-empty nodes based on density
        // Higher layers (coarser) have higher density due to aggregation
        const float LayerDensity = FMath::Min(1.0f, VolumeDensity * FMath::Pow(1.2f, Layer));
        const int32 NonEmptyNodesAtLayer = FMath::RoundToInt(TotalNodesAtLayer * LayerDensity);
        
        // Estimate memory per node (simplified - includes node data, children pointers, etc.)
        const int32 BytesPerNode = Layer == 0 ? 16 : 24; // Leaf nodes vs internal nodes
        const int64 LayerBytes = static_cast<int64>(NonEmptyNodesAtLayer) * BytesPerNode;
        
        TotalEstimatedNodes += NonEmptyNodesAtLayer;
        TotalEstimatedBytes += LayerBytes;
        
        if (Layer == 0)
        {
            TotalEstimatedVoxels = NonEmptyNodesAtLayer;
        }
        
        UE_LOG(LogNav3D, Log, TEXT("Layer %d: %s nodes (%.1f%% density) | %s bytes"), 
               Layer,
               *FormatNumber(NonEmptyNodesAtLayer),
               LayerDensity * 100.0f,
               *FormatNumber(LayerBytes));
    }
    
    // Estimate additional overhead (serialization headers, metadata, etc.)
    const int64 OverheadBytes = FMath::Max(1024LL, TotalEstimatedBytes / 20); // ~5% overhead
    const int64 TotalEstimatedSize = TotalEstimatedBytes + OverheadBytes;
    
    UE_LOG(LogNav3D, Log, TEXT(""));
    UE_LOG(LogNav3D, Log, TEXT("=== OCTREE SIZE SUMMARY ==="));
    UE_LOG(LogNav3D, Log, TEXT("Total Voxels: %s"), *FormatNumber(TotalEstimatedVoxels));
    UE_LOG(LogNav3D, Log, TEXT("Total Nodes: %s"), *FormatNumber(TotalEstimatedNodes));
    UE_LOG(LogNav3D, Log, TEXT("Estimated Size: %s bytes (%.2f MB)"), 
           *FormatNumber(TotalEstimatedSize), TotalEstimatedSize / (1024.0 * 1024.0));
    
    // Provide size context
    if (TotalEstimatedSize < 1024 * 1024) // < 1MB
    {
        UE_LOG(LogNav3D, Log, TEXT("Size Category: Small (< 1MB)"));
    }
    else if (TotalEstimatedSize < 10 * 1024 * 1024) // < 10MB
    {
        UE_LOG(LogNav3D, Log, TEXT("Size Category: Medium (1-10MB)"));
    }
    else if (TotalEstimatedSize < 100 * 1024 * 1024) // < 100MB
    {
        UE_LOG(LogNav3D, Log, TEXT("Size Category: Large (10-100MB)"));
    }
    else
    {
        UE_LOG(LogNav3D, Log, TEXT("Size Category: Very Large (>100MB)"));
    }
    
    UE_LOG(LogNav3D, Log, TEXT(""));
}

void ANav3DData::BuildNavigationData() const
{
	// Drive the navigation system directly to avoid duplicate editor build notifications
	if (UWorld* World = GetWorld())
	{
		
		// Clean up invalid chunk actors before destroying valid ones
		const int32 InvalidCount = const_cast<ANav3DData*>(this)->GetInvalidChunkActorCount();
		if (InvalidCount > 0)
		{
			UE_LOG(LogNav3D, Log, TEXT("BuildNavigationData: Cleaning up %d invalid chunk actors before rebuild"), InvalidCount);
			const_cast<ANav3DData*>(this)->CleanupInvalidChunkActors();
		}
		
		// Destroy all existing chunk actors before rebuilding everything
		TArray<ANav3DDataChunkActor*> ActorsToDestroy;
		ActorsToDestroy.Reserve(ChunkActors.Num());
		for (ANav3DDataChunkActor* ChunkActor : ChunkActors)
		{
			if (ChunkActor)
			{
				ActorsToDestroy.Add(ChunkActor);
			}
		}
		for (ANav3DDataChunkActor* ActorToDestroy : ActorsToDestroy)
		{
			UE_LOG(LogNav3D, Log, TEXT("Destroying chunk actor before full rebuild: %s"), *ActorToDestroy->GetName());
			World->DestroyActor(ActorToDestroy);
		}

		if (UNavigationSystemV1* NavSys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(World))
		{
			NavSys->CancelBuild();
			NavSys->Build();
		}
	}
}

void ANav3DData::BuildSingleVolume(const FBox& VolumeBounds)
{
	// Record build start time
	const double BuildStartTime = FPlatformTime::Seconds();
	
	// Clean up invalid chunk actors before building
	const int32 InvalidCount = GetInvalidChunkActorCount();
	if (InvalidCount > 0)
	{
		UE_LOG(LogNav3D, Log, TEXT("BuildSingleVolume: Cleaning up %d invalid chunk actors before rebuild"), InvalidCount);
		CleanupInvalidChunkActors();
	}
	
	// First, find and destroy any existing chunk actors in these bounds
	TArray<ANav3DDataChunkActor*> ActorsToDestroy;
	for (ANav3DDataChunkActor* ChunkActor : ChunkActors)
	{
		if (ChunkActor && ChunkActor->DataChunkActorBounds.Intersect(VolumeBounds))
		{
			ActorsToDestroy.Add(ChunkActor);
		}
	}

	// Destroy existing actors (they will auto-unregister)
	for (ANav3DDataChunkActor* ActorToDestroy : ActorsToDestroy)
	{
		UE_LOG(LogNav3D, Log, TEXT("Destroying chunk actor: %s"), *ActorToDestroy->GetName());
		GetWorld()->DestroyActor(ActorToDestroy);
	}
	
	UE_LOG(LogNav3D, Log, TEXT("Building single volume: %s"), *VolumeBounds.ToString());
	
	// Store build start time for timing completion
	SingleVolumeBuildStartTime = BuildStartTime;
	
	// Use the navigation system's async build process for proper UI feedback
	// This will trigger the same build notifications, toasts, and progress updates as Build All
	if (UWorld* World = GetWorld())
	{
		if (UNavigationSystemV1* NavSys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(World))
		{
			// Cancel any existing build
			NavSys->CancelBuild();
			
			// Set the generator to build only this volume
			if (FNavDataGenerator* BaseGenerator = GetGenerator())
			{
				if (FNav3DDataGenerator* Generator = static_cast<FNav3DDataGenerator*>(BaseGenerator))
				{
					Generator->SetBuildTargetVolume(VolumeBounds);
				}
			}
			
			// Start the async build process - this will show progress, toasts, etc.
			NavSys->Build();
		}
	}
}

void ANav3DData::RebuildSingleChunk(const FBox& ChunkBounds)
{
	// Chunk-only rebuild: do NOT destroy other chunk actors.
	UE_LOG(LogNav3D, Log, TEXT("Building single chunk: %s"), *ChunkBounds.ToString());
	
	
	// Clean up invalid chunk actors before rebuilding
	const int32 InvalidCount = GetInvalidChunkActorCount();
	if (InvalidCount > 0)
	{
		UE_LOG(LogNav3D, Log, TEXT("RebuildSingleChunk: Cleaning up %d invalid chunk actors before rebuild"), InvalidCount);
		CleanupInvalidChunkActors();
	}
	
	// Prefer driving the generator directly to avoid losing single-target state
	if (FNavDataGenerator* BaseGenerator = GetGenerator())
	{
		if (FNav3DDataGenerator* Generator = static_cast<FNav3DDataGenerator*>(BaseGenerator))
		{
			// Ensure generator is initialized so it can submit async tasks
			Generator->Init();
			Generator->SetBuildTargetVolume(ChunkBounds);
			Generator->RebuildAll();
			Generator->EnsureBuildCompletion();
			return;
		}
	}

	// Fallback: construct generator and retry
	ConditionalConstructGenerator();
	if (FNavDataGenerator* BaseGenerator2 = GetGenerator())
	{
		if (FNav3DDataGenerator* Generator = static_cast<FNav3DDataGenerator*>(BaseGenerator2))
		{
			Generator->Init();
			Generator->SetBuildTargetVolume(ChunkBounds);
			Generator->RebuildAll();
			Generator->EnsureBuildCompletion();
		}
	}
}

void ANav3DData::RebuildSingleChunk(const ANav3DDataChunkActor* ChunkActor)
{
	if (!ChunkActor) return;
	RebuildSingleChunk(ChunkActor->DataChunkActorBounds);
}

void ANav3DData::RebuildTacticalData()
{
	if (!TacticalSettings.bEnableTacticalReasoning)
	{
		UE_LOG(LogNav3D, Warning, TEXT("Tactical reasoning is disabled"));
		return;
	}
	
	if (ChunkActors.Num() == 0)
	{
		UE_LOG(LogNav3D, Warning, TEXT("Cannot rebuild tactical data, no navigation data available"));
		return;
	}
	
	UE_LOG(LogNav3D, Log, TEXT("Rebuilding consolidated tactical data for %d chunk actors"), ChunkActors.Num());
	
	// Simply rebuild the consolidated tactical data
	RefreshConsolidatedTacticalData();
}

void ANav3DData::InvalidateAffectedPaths(const TArray<FBox>& UpdatedBounds)
{
	const int32 PathsCount = ActivePaths.Num();
	const int32 UpdatedBoundsCount = UpdatedBounds.Num();

	if (UpdatedBoundsCount == 0 || PathsCount == 0)
	{
		return;
	}

	// Paths can be registered from async PathFinding thread.
	// Theoretically paths are invalidated synchronously by the navigation system
	// before starting async queries task but protecting ActivePaths will make
	// the system safer in case of future timing changes.
	{
		FScopeLock PathLock(&ActivePathsLock);

		FNavPathWeakPtr* WeakPathPtr = (ActivePaths.GetData() + PathsCount - 1);

		for (int32 PathIndex = PathsCount - 1; PathIndex >= 0;
		     --PathIndex, --WeakPathPtr)
		{
			FNavPathSharedPtr SharedPath = WeakPathPtr->Pin();
			if (!WeakPathPtr->IsValid())
			{
				ActivePaths.RemoveAtSwap(PathIndex, 1, EAllowShrinking::No);
			}
			else
			{
				const FNavigationPath* Path = SharedPath.Get();
				if (!Path->IsReady() || Path->GetIgnoreInvalidation())
				{
					continue;
				}

				for (const auto& PathPoint : Path->GetPathPoints())
				{
					if (UpdatedBounds.FindByPredicate([&PathPoint](const FBox& Bounds)
					{
						return Bounds.IsInside(PathPoint.Location);
					}) != nullptr)
					{
						SharedPath->Invalidate();
						ActivePaths.RemoveAtSwap(PathIndex, 1, EAllowShrinking::No);
						break;
					}
				}

				if (!SharedPath->IsValid())
				{
					break;
				}
			}
		}
	}
}

void ANav3DData::OnNavigationDataGenerationFinished() const
{
	if (const UWorld* World = GetWorld())
	{
		if (IsValid(World))
		{
#if WITH_EDITOR
			// Create navigation data holders in each streaming level.
			if (!World->IsGameWorld())
			{
				for (const auto& Levels = World->GetLevels(); auto* Level : Levels)
				{
					if (Level->IsPersistentLevel())
					{
						continue;
					}

					UNav3DDataChunk* NavigationDataChunk = GetNavigationDataChunk(Level);

					if (SupportsStreaming())
					{
						TArray<int32> OverlappingVolumeIndices;
						const FBox LevelBounds = CalculateLevelBounds(Level);
						
						UE_LOG(LogNav3D, Verbose, TEXT("Nav3D: Processing level %s with bounds %s"), 
							*Level->GetName(), *LevelBounds.ToString());

						// In the new chunk-based system, streaming levels are handled by chunk actors
						// This section is kept for compatibility but doesn't need to do anything
						UE_LOG(LogNav3D, Verbose, TEXT("Nav3D: Level %s - handled by chunk actors"), *Level->GetName());

						UE_LOG(LogNav3D, Verbose, TEXT("Nav3D: Level %s has %d overlapping volumes"), 
							*Level->GetName(), OverlappingVolumeIndices.Num());

						if (OverlappingVolumeIndices.Num() > 0)
						{
							if (NavigationDataChunk == nullptr)
							{
								NavigationDataChunk = NewObject<UNav3DDataChunk>(Level);
								NavigationDataChunk->NavigationDataName = GetFName();
								Level->NavDataChunks.Add(NavigationDataChunk);
								UE_LOG(LogNav3D, Verbose, TEXT("Nav3D: Created new navigation data chunk for level %s"), 
									*Level->GetName());
							}

							// In the new chunk-based system, this is handled by chunk actors
							// No need to add volumes to chunks manually

							UE_LOG(LogNav3D, Verbose, TEXT("Nav3D: Added %d volumes to level %s chunk"), 
								OverlappingVolumeIndices.Num(), *Level->GetName());

							// Build boundary voxels for this chunk after population
							FNav3DUtils::IdentifyBoundaryVoxels(NavigationDataChunk);

							continue;
						}
					}

					// Remove stale data.
					if (!IsRunningCommandlet())
					{
						if (NavigationDataChunk != nullptr)
						{
							NavigationDataChunk->ReleaseNavigationData();
							Level->NavDataChunks.Remove(NavigationDataChunk);
						}
					}
				}
			}
#endif
		}
	}
}

UNav3DDataChunk* ANav3DData::GetNavigationDataChunk(ULevel* Level) const
{
	const auto ThisName = GetFName();

	if (const auto* Result = Level->NavDataChunks.FindByPredicate(
		[&](const UNavigationDataChunk* Chunk)
		{
			return Chunk->NavigationDataName == ThisName;
		}))
	{
		return Cast<UNav3DDataChunk>(*Result);
	}

	return nullptr;
}

FBox ANav3DData::CalculateLevelBounds(ULevel* Level)
{
	if (!Level || Level->Actors.Num() == 0)
	{
		return FBox(ForceInit);
	}
	
	FBox LevelBounds(ForceInit);
	
	for (const AActor* Actor : Level->Actors)
	{
		if (Actor && Actor->GetRootComponent())
		{
			const FBox ActorBounds = Actor->GetComponentsBoundingBox(true);
			if (ActorBounds.IsValid)
			{
				LevelBounds += ActorBounds;
			}
		}
	}
	
	return LevelBounds;
}

FPathFindingResult ANav3DData::FindPath(
	const FNavAgentProperties& NavAgentProperties,
	const FPathFindingQuery& PathFindingQuery)
{
	const auto* Self = Cast<ANav3DData>(PathFindingQuery.NavData.Get());
	if (Self == nullptr)
	{
		return ENavigationQueryResult::Error;
	}

	FPathFindingResult Result(ENavigationQueryResult::Error);
	FNavigationPath* NavigationPath = PathFindingQuery.PathInstanceToFill.Get();
	FNav3DPath* N3dNavigationPath = NavigationPath != nullptr
		                                ? NavigationPath->CastPath<FNav3DPath>()
		                                : nullptr;

	if (N3dNavigationPath != nullptr)
	{
		Result.Path = PathFindingQuery.PathInstanceToFill;
		N3dNavigationPath->ResetForRepath();
	}
	else
	{
		Result.Path = Self->CreatePathInstance<FNav3DPath>(PathFindingQuery);
		NavigationPath = Result.Path.Get();
		N3dNavigationPath = NavigationPath != nullptr
			                    ? NavigationPath->CastPath<FNav3DPath>()
			                    : nullptr;
	}

	if (NavigationPath != nullptr && PathFindingQuery.QueryFilter.IsValid())
	{
		// Add small epsilon to avoid floating point equality issues
		constexpr float MinPathDist = 1.0f;
		if ((PathFindingQuery.StartLocation - PathFindingQuery.EndLocation).SizeSquared() < (MinPathDist * MinPathDist))
		{
			Result.Path->GetPathPoints().Reset();
			Result.Path->GetPathPoints().Add(FNavPathPoint(PathFindingQuery.EndLocation));
			Result.Result = ENavigationQueryResult::Success;
		}
		else
		{
			/*
			Result.Result = FNav3DPathFinder::GetPath(
				*N3dNavigationPath,
				*Self,
				PathFindingQuery.StartLocation,
				PathFindingQuery.EndLocation,
				NavAgentProperties,
				PathFindingQuery.QueryFilter);
				*/
		}
	}

	return Result;
}

void FNav3DVolumeNavigationData::RebuildDirtyBounds(const TArray<FBox>& DirtyBounds)
{
	// Clean up invalid occluders first
	DynamicOccluders.RemoveAllSwap([](const TWeakObjectPtr<const AActor>& Existing)
	{
		return !Existing.IsValid();
	});

	UE_LOG(LogNav3D, Verbose, TEXT("RebuildDirtyBounds starting - Total dynamic occluders: %d"),
	       DynamicOccluders.Num());

	for (const auto& DynamicOccluder : DynamicOccluders)
	{
		if (const AActor* Occluder = DynamicOccluder.Get())
		{
			UE_LOG(LogNav3D, Verbose, TEXT("  Active occluder: %s"), *Occluder->GetActorNameOrLabel());
		}
	}

	// Track nodes that will need cover updates
	TSet<FNav3DNodeAddress> AffectedNodes;

	// Rebuild navigation for each dirty bounds
	for (const FBox& Bounds : DirtyBounds)
	{
		UE_LOG(LogNav3D, Verbose, TEXT("Rebuilding bounds: %s"), *Bounds.ToString());

		// Get nodes that were free before rebuild for potential cover updates
		const FBox ExpandedBounds = Bounds.ExpandBy(GetData().GetLayer(0).GetNodeExtent());
		for (LayerIndex LayerIdx = 0; LayerIdx < GetData().GetLayerCount(); LayerIdx++)
		{
			const auto& Layer = GetData().GetLayer(LayerIdx);
			for (int32 NodeIdx = 0; NodeIdx < Layer.GetNodes().Num(); NodeIdx++)
			{
				const FNav3DNodeAddress NodeAddress(LayerIdx, NodeIdx);
				const FVector NodePos = GetNodePositionFromAddress(NodeAddress, true);
				const float NodeExtent = GetNodeExtentFromNodeAddress(NodeAddress);

				if (IsNodeInBounds(NodePos, NodeExtent, ExpandedBounds))
				{
					const auto& Node = GetNodeFromAddress(NodeAddress);
					if (!Node.HasChildren() && !(LayerIdx == 0 && Node.FirstChild.IsValid()))
					{
						AffectedNodes.Add(NodeAddress);
					}
				}
			}
		}

		// Perform the actual rebuild
		RebuildLeafNodesInBounds(Bounds);

		// Add newly free nodes to affected set
		for (LayerIndex LayerIdx = 0; LayerIdx < GetData().GetLayerCount(); LayerIdx++)
		{
			const auto& Layer = GetData().GetLayer(LayerIdx);
			for (int32 NodeIdx = 0; NodeIdx < Layer.GetNodes().Num(); NodeIdx++)
			{
				const FNav3DNodeAddress NodeAddress(LayerIdx, NodeIdx);
				const FVector NodePos = GetNodePositionFromAddress(NodeAddress, true);
				const float NodeExtent = GetNodeExtentFromNodeAddress(NodeAddress);

				if (IsNodeInBounds(NodePos, NodeExtent, ExpandedBounds))
				{
					const auto& Node = GetNodeFromAddress(NodeAddress);
					if (!Node.HasChildren() && !(LayerIdx == 0 && Node.FirstChild.IsValid()))
					{
						AffectedNodes.Add(NodeAddress);
					}
				}
			}
		}
	}
}

void ANav3DData::RegisterDynamicOccluder(const AActor* Occluder)
{
	if (!Occluder)
	{
		UE_LOG(LogNav3D, Verbose, TEXT("RegisterDynamicOccluder called with null Occluder"));
		return;
	}

	const FBox OccluderBounds = Occluder->GetComponentsBoundingBox(true);
	UE_LOG(LogNav3D, Verbose,
	       TEXT("ANav3DData::RegisterDynamicOccluder for %s - Volume count: %d, Occluder bounds: %s"),
	       *Occluder->GetActorNameOrLabel(), ChunkActors.Num(), *OccluderBounds.ToString());

	bool bAnyIntersection = false;
	for (ANav3DDataChunkActor* ChunkActor : ChunkActors)
	{
		if (!ChunkActor) continue;
		
		for (const UNav3DDataChunk* Chunk : ChunkActor->Nav3DChunks)
		{
			if (!Chunk) continue;

			if (FNav3DVolumeNavigationData* VolumeNavData =
				const_cast<FNav3DVolumeNavigationData*>(Chunk->GetVolumeNavigationData()))
			{
				const FBox& NavBounds = VolumeNavData->GetVolumeBounds();

				if (NavBounds.Intersect(OccluderBounds))
				{
					bAnyIntersection = true;
					UE_LOG(LogNav3D, Verbose, TEXT("Registering occluder %s with volume at %s"),
					       *Occluder->GetActorNameOrLabel(), *NavBounds.ToString());
					VolumeNavData->AddDynamicOccluder(Occluder);
				}
			}
		}
	}

	if (!bAnyIntersection)
	{
		UE_LOG(LogNav3D, Verbose, TEXT("No intersecting volumes found for occluder %s"),
		       *Occluder->GetActorNameOrLabel());
	}
}

void ANav3DData::UnregisterDynamicOccluder(const AActor* Occluder)
{
	for (ANav3DDataChunkActor* ChunkActor : ChunkActors)
	{
		if (!ChunkActor) continue;
		
		for (const UNav3DDataChunk* Chunk : ChunkActor->Nav3DChunks)
		{
			if (!Chunk) continue;

			if (FNav3DVolumeNavigationData* VolumeNavData =
				const_cast<FNav3DVolumeNavigationData*>(Chunk->GetVolumeNavigationData()))
			{
				VolumeNavData->RemoveDynamicOccluder(Occluder);
			}
		}
	}
}

void ANav3DData::RebuildDirtyBounds(const TArray<FBox>& DirtyBounds)
{
	UE_LOG(LogNav3D, Verbose, TEXT("Nav3DData: Processing %d dirty bounds"), DirtyBounds.Num());

	for (ANav3DDataChunkActor* ChunkActor : ChunkActors)
	{
		if (!ChunkActor) continue;
		
		for (const UNav3DDataChunk* Chunk : ChunkActor->Nav3DChunks)
		{
			if (!Chunk) continue;

			if (FNav3DVolumeNavigationData* VolumeNavData =
				const_cast<FNav3DVolumeNavigationData*>(Chunk->GetVolumeNavigationData()))
			{
				const FBox& VolumeBounds = VolumeNavData->GetVolumeBounds();

				bool bIntersects = false;
				for (const FBox& DirtyBound : DirtyBounds)
				{
					if (VolumeBounds.Intersect(DirtyBound))
					{
						bIntersects = true;
						UE_LOG(LogNav3D, Verbose, TEXT("Nav3DData: Found intersecting volume at %s"),
						       *VolumeBounds.ToString());
						break;
					}
				}

				if (bIntersects)
				{
					VolumeNavData->RebuildDirtyBounds(DirtyBounds);
				}
			}
		}
	}

	RequestDrawingUpdate();
	InvalidateAffectedPaths(DirtyBounds);
}

bool ANav3DData::InitializeTacticalReasoning()
{
	// Make sure we only initialize if tactical reasoning is enabled
	if (!TacticalSettings.bEnableTacticalReasoning)
	{
		return true;
	}
    
	// Only create a new instance if we don't already have one
	if (!TacticalReasoning.IsValid())
	{
		UE_LOG(LogNav3D, Verbose, TEXT("Creating new FNav3DTacticalReasoning instance"));
		TacticalReasoning = MakeUnique<FNav3DTacticalReasoning>();
	}
    
	// Initialize the tactical reasoning (this is safe to call multiple times)
	if (TacticalReasoning.IsValid())
	{
		TacticalReasoning->SetNavDataRef(this);
		return true;
	}
	else
	{
		UE_LOG(LogNav3D, Error, TEXT("Failed to create TacticalReasoning instance"));
		return false;
	}
}

void ANav3DData::BuildTacticalData()
{
	// No-op under new architecture: tactical data is built per chunk and consolidated on load/unload
}

// =============================================================================
// CONSOLIDATED TACTICAL DATA MANAGEMENT
// =============================================================================

void ANav3DData::OnChunkActorLoaded(const ANav3DDataChunkActor* ChunkActor)
{
	if (!ChunkActor || !TacticalSettings.bEnableTacticalReasoning)
	{
		return;
	}
	
	UE_LOG(LogNav3D, Verbose, TEXT("OnChunkActorLoaded: %s"), *ChunkActor->GetName());
	
	bNeedsTacticalRebuild = true;
	if (const UWorld* World = GetWorld())
	{
		FTimerManager& TimerManager = World->GetTimerManager();
		TimerManager.SetTimer(DeferredTacticalRebuildHandle,
			FTimerDelegate::CreateUObject(this, &ANav3DData::PerformDeferredTacticalRefresh),
			0.1f, false);
	}
}

void ANav3DData::OnChunkActorUnloaded(ANav3DDataChunkActor* ChunkActor)
{
	if (!TacticalSettings.bEnableTacticalReasoning)
	{
		return;
	}
	
	UE_LOG(LogNav3D, Verbose, TEXT("OnChunkActorUnloaded: %s"), *GetNameSafe(ChunkActor));
	
	// Remove this chunk from both consolidated data formats
	ConsolidatedTacticalData.SourceChunks.Remove(ChunkActor);
	ConsolidatedCompactTacticalData.SourceChunks.Remove(ChunkActor);
	
	bNeedsTacticalRebuild = true;
	if (const UWorld* World = GetWorld())
	{
		FTimerManager& TimerManager = World->GetTimerManager();
		TimerManager.SetTimer(DeferredTacticalRebuildHandle,
			FTimerDelegate::CreateUObject(this, &ANav3DData::PerformDeferredTacticalRefresh),
			0.1f, false);
	}
}

uint16 ANav3DData::GetVolumeIDForGlobalRegion(const uint16 GlobalRegionId) const
{
	if (const FRegionMapping* Mapping = GlobalToLocalRegionMapping.Find(GlobalRegionId))
	{
		return Mapping->VolumeID;
	}
	return 0;
}

uint8 ANav3DData::GetLocalRegionIndexForGlobalRegion(const uint16 GlobalRegionId) const
{
	if (const FRegionMapping* Mapping = GlobalToLocalRegionMapping.Find(GlobalRegionId))
	{
		return Mapping->LocalRegionIndex;
	}
	return 0;
}

void ANav3DData::RefreshConsolidatedTacticalData()
{
	QUICK_SCOPE_CYCLE_COUNTER(STAT_Nav3D_RebuildConsolidatedTactical);
	
	if (!TacticalSettings.bEnableTacticalReasoning)
	{
		return;
	}
	
	// Clear existing data
	ConsolidatedTacticalData.Reset();
	
	// Collect all loaded chunks with tactical data
	TArray<ANav3DDataChunkActor*> LoadedChunks;
	for (ANav3DDataChunkActor* ChunkActor : ChunkActors)
	{
		if (ChunkActor && ChunkActor->HasTacticalData())
		{
			LoadedChunks.Add(ChunkActor);
		}
	}
	
	if (LoadedChunks.Num() == 0)
	{
		UE_LOG(LogNav3D, Verbose, TEXT("No chunks with tactical data loaded"));
		return;
	}
	
	UE_LOG(LogNav3D, Log, TEXT("Rebuilding consolidated tactical data from %d chunks"), LoadedChunks.Num());
	
	// Step 1: Consolidate all regions from loaded chunks
	ConsolidateRegionsFromChunks(LoadedChunks);
	
	// Step 2: Build cross-chunk adjacency from boundary interfaces
	BuildCrossChunkAdjacency(LoadedChunks);
	
	// Step 3: Build cross-chunk visibility using sample-based raycasting
	BuildCrossChunkVisibility(LoadedChunks);
	
	// Step 4: Prune regions to limit using density-focused strategy (32 regions max for testing)
	constexpr int8 MaxRegions = 64;
	if (ConsolidatedTacticalData.GetRegionCount() > MaxRegions)
	{
		UE_LOG(LogNav3D, Log, TEXT("Region count (%d) exceeds limit, applying density-focused pruning"), 
		       ConsolidatedTacticalData.GetRegionCount());
		
		// Calculate volume bounds from all loaded chunks
		FBox VolumeBounds(ForceInit);
		for (const ANav3DDataChunkActor* ChunkActor : LoadedChunks)
		{
			if (ChunkActor)
			{
				VolumeBounds += ChunkActor->DataChunkActorBounds;
			}
		}
		
		// Apply density-focused region pruning
		const TArray<int32> SelectedRegionIds = FDensityFocusedPruningStrategy::PruneRegionsToLimit(
			ConsolidatedTacticalData, VolumeBounds, LoadedChunks, MaxRegions);
		
		// Filter consolidated data to only include selected regions
		FilterConsolidatedDataToSelectedRegions(SelectedRegionIds);
		
		UE_LOG(LogNav3D, Log, TEXT("Density-focused pruning applied: reduced from %d to %d regions"),
		       ConsolidatedTacticalData.GetRegionCount(), SelectedRegionIds.Num());
	}
	
	// Update source chunks
	ConsolidatedTacticalData.SourceChunks.Empty();
	for (ANav3DDataChunkActor* ChunkActor : LoadedChunks)
	{
		ConsolidatedTacticalData.SourceChunks.Add(ChunkActor);
	}
	
	// Update loaded region IDs for filtering
	UpdateLoadedRegionIds();
	
	UE_LOG(LogNav3D, Log, TEXT("Rebuilt consolidated tactical data: %d regions from %d chunks"),
	       ConsolidatedTacticalData.GetRegionCount(), LoadedChunks.Num());
	
	// Only force drawing update if no chunks are currently building
	// This prevents access violations during the build process
	bool bAnyChunkBuilding = false;
	for (const ANav3DDataChunkActor* ChunkActor : ChunkActors)
	{
		if (ChunkActor && ChunkActor->bIsBuilding)
		{
			bAnyChunkBuilding = true;
			break;
		}
	}
	
	if (!bAnyChunkBuilding)
	{
		// Force drawing update to show new tactical data
		RequestDrawingUpdate();
	}
	else
	{
		UE_LOG(LogNav3D, Verbose, TEXT("Skipping tactical data drawing update - build still in progress"));
	}
}

const FConsolidatedTacticalData& ANav3DData::GetConsolidatedTacticalData() const
{
#if WITH_EDITOR
	if (ConsolidatedTacticalData.IsEmpty() || bConsolidatedDataDirty)
	{
		UE_LOG(LogNav3D, Verbose, TEXT("Converting compact data to consolidated format for debug/UI"));
		RefreshConsolidatedDataFromCompact();
	}
#endif

	return ConsolidatedTacticalData;
}

void ANav3DData::RefreshConsolidatedDataFromCompact() const
{
	if (!TacticalSettings.bEnableTacticalReasoning)
	{
		const_cast<ANav3DData*>(this)->ConsolidatedTacticalData.Reset();
		bConsolidatedDataDirty = false;
		return;
	}

	// Find chunks with compact data
	TArray<ANav3DDataChunkActor*> ChunksWithCompactData;
	for (ANav3DDataChunkActor* ChunkActor : ChunkActors)
	{
		if (ChunkActor && !ChunkActor->CompactTacticalData.IsEmpty())
		{
			ChunksWithCompactData.Add(ChunkActor);
		}
	}

	if (ChunksWithCompactData.Num() == 0)
	{
		UE_LOG(LogNav3D, Verbose, TEXT("No chunks with compact tactical data found"));
		const_cast<ANav3DData*>(this)->ConsolidatedTacticalData.Reset();
		bConsolidatedDataDirty = false;
		return;
	}

	UE_LOG(LogNav3D, Log, TEXT("Converting compact tactical data from %d chunks to Build format"), 
	       ChunksWithCompactData.Num());

	// Use the existing BuildConsolidatedCompactFromChunks + CompactToBuild pipeline
	const_cast<ANav3DData*>(this)->BuildConsolidatedCompactFromChunks(ChunksWithCompactData);

	if (!ConsolidatedCompactTacticalData.IsEmpty())
	{
		// Convert compact→consolidated using the proven converter
		const_cast<ANav3DData*>(this)->ConsolidatedTacticalData = 
			FNav3DTacticalDataConverter::CompactToBuild(ConsolidatedCompactTacticalData, ChunksWithCompactData);
		
		UE_LOG(LogNav3D, Log, TEXT("Successfully converted to consolidated format: %d regions, %d adjacency, %d visibility"),
		       ConsolidatedTacticalData.AllLoadedRegions.Num(),
		       ConsolidatedTacticalData.RegionAdjacency.Num(), 
		       ConsolidatedTacticalData.RegionVisibility.Num());
	}
	else
	{
		UE_LOG(LogNav3D, Warning, TEXT("Failed to build consolidated compact data"));
		const_cast<ANav3DData*>(this)->ConsolidatedTacticalData.Reset();
	}
	
	bConsolidatedDataDirty = false;
}

void ANav3DData::InvalidateConsolidatedData() const
{
	bConsolidatedDataDirty = true;
	UE_LOG(LogNav3D, Verbose, TEXT("Consolidated tactical data marked dirty - will refresh on next access"));
}

void ANav3DData::RebuildConsolidatedCompactTacticalData()
{
	QUICK_SCOPE_CYCLE_COUNTER(STAT_Nav3D_RebuildConsolidatedCompactTactical);
	
	if (!TacticalSettings.bEnableTacticalReasoning)
	{
		return;
	}
	
	// Clear existing compact data
	ConsolidatedCompactTacticalData.Reset();
	
	// Collect all loaded chunks with compact tactical data
	TArray<ANav3DDataChunkActor*> LoadedChunks;
	int32 TotalChunks = 0;
	int32 ChunksWithCompactData = 0;
	int32 ChunksWithBuildData = 0;
	
	for (ANav3DDataChunkActor* ChunkActor : ChunkActors)
	{
		TotalChunks++;
		if (ChunkActor)
		{
			if (!ChunkActor->CompactTacticalData.IsEmpty())
			{
				LoadedChunks.Add(ChunkActor);
				ChunksWithCompactData++;
			}
			if (ChunkActor->HasTacticalData())
			{
				ChunksWithBuildData++;
			}
		}
	}
	
	UE_LOG(LogNav3D, Log, TEXT("RebuildConsolidatedCompactTacticalData: %d total chunks, %d with compact data, %d with build data"), 
	       TotalChunks, ChunksWithCompactData, ChunksWithBuildData);
	
	if (LoadedChunks.Num() == 0)
	{
		UE_LOG(LogNav3D, Verbose, TEXT("No chunks with compact tactical data loaded"));
		return;
	}
	
	UE_LOG(LogNav3D, Log, TEXT("Rebuilding consolidated compact tactical data from %d chunks"), LoadedChunks.Num());
	
	// Step 1: Consolidate all compact regions from loaded chunks
	ConsolidateCompactRegionsFromChunks(LoadedChunks);
	
	// Step 2: Build global adjacency from intra-volume and cross-volume connections
	BuildGlobalCompactAdjacency(LoadedChunks);
	
	// Update source chunks
	ConsolidatedCompactTacticalData.SourceChunks.Empty();
	for (ANav3DDataChunkActor* ChunkActor : LoadedChunks)
	{
		ConsolidatedCompactTacticalData.SourceChunks.Add(ChunkActor);
	}
	
	// Update loaded region IDs for filtering
	UpdateLoadedRegionIds();
	
	UE_LOG(LogNav3D, Log, TEXT("Rebuilt consolidated compact tactical data: %d regions from %d chunks"),
	       ConsolidatedCompactTacticalData.GetRegionCount(), LoadedChunks.Num());
	
	// Only force drawing update if no chunks are currently building
	bool bAnyChunkBuilding = false;
	for (const ANav3DDataChunkActor* ChunkActor : ChunkActors)
	{
		if (ChunkActor && ChunkActor->bIsBuilding)
		{
			bAnyChunkBuilding = true;
			break;
		}
	}
	
	if (!bAnyChunkBuilding)
	{
		// Force drawing update to show new tactical data
		RequestDrawingUpdate();
	}
	else
	{
		UE_LOG(LogNav3D, Verbose, TEXT("Skipping compact tactical data drawing update - build still in progress"));
	}
}

void ANav3DData::ConsolidateRegionsFromChunks(const TArray<ANav3DDataChunkActor*>& LoadedChunks)
{
	ConsolidatedTacticalData.AllLoadedRegions.Empty();
	ConsolidatedTacticalData.RegionAdjacency.Empty();
	
	// Global region ID remapping to avoid conflicts
	TMap<int32, int32> GlobalIdRemapping;
	int32 NextGlobalId = 0;
	
	
	// Collect all regions from loaded chunks
	for (ANav3DDataChunkActor* ChunkActor : LoadedChunks)
	{
		if (!ChunkActor || !ChunkActor->HasCompactTacticalData())
		{
			continue;
		}
		
		// Convert compact regions to Build for consolidated debug data
		for (int32 LocalIndex = 0; LocalIndex < ChunkActor->CompactTacticalData.Regions.Num(); ++LocalIndex)
		{
			const FCompactRegion& CR = ChunkActor->CompactTacticalData.Regions[LocalIndex];
				FNav3DRegion BuildRegion = FNav3DTacticalDataConverter::CompactToRegion(CR, NextGlobalId);
			GlobalIdRemapping.Add(LocalIndex, NextGlobalId);
			ConsolidatedTacticalData.AllLoadedRegions.Add(BuildRegion);
			NextGlobalId++;
		}
		// Convert adjacency bitmasks using remapped IDs
		for (const auto& AdjPair : ChunkActor->CompactTacticalData.RegionAdjacency)
		{
			const int32 LocalFrom = AdjPair.Key;
			const uint64 Mask = AdjPair.Value;
			const int32* NewFrom = GlobalIdRemapping.Find(LocalFrom);
			if (!NewFrom) { continue; }
			FRegionIdArray NewAdj;
			for (int32 Bit = 0; Bit < 64; ++Bit)
			{
				if (Mask & (1ULL << Bit))
				{
					if (const int32* NewTo = GlobalIdRemapping.Find(Bit))
					{
						NewAdj.Add(*NewTo);
					}
				}
			}
			if (NewAdj.Num() > 0)
			{
				ConsolidatedTacticalData.RegionAdjacency.Add(*NewFrom, NewAdj);
			}
		}
	}
	
	UE_LOG(LogNav3D, Verbose, TEXT("Consolidated %d regions with %d adjacency entries"),
	       ConsolidatedTacticalData.AllLoadedRegions.Num(),
	       ConsolidatedTacticalData.RegionAdjacency.Num());
}

void ANav3DData::BuildCrossChunkAdjacency(const TArray<ANav3DDataChunkActor*>& LoadedChunks)
{
	// Test adjacency between all chunk pairs
	for (int32 i = 0; i < LoadedChunks.Num(); ++i)
	{
		for (int32 j = i + 1; j < LoadedChunks.Num(); ++j)
		{
			ANav3DDataChunkActor* ChunkA = LoadedChunks[i];
			ANav3DDataChunkActor* ChunkB = LoadedChunks[j];
			
			if (ChunkA->IsAdjacentToChunk(ChunkB))
			{
				BuildAdjacencyBetweenChunks(ChunkA, ChunkB);
			}
		}
	}
	
	UE_LOG(LogNav3D, Verbose, TEXT("Built cross-chunk adjacency for %d chunk pairs"), 
	       (LoadedChunks.Num() * (LoadedChunks.Num() - 1)) / 2);
}

void ANav3DData::BuildAdjacencyBetweenChunks(
	ANav3DDataChunkActor* ChunkA, 
	ANav3DDataChunkActor* ChunkB)
{
	if (!ChunkA || !ChunkB)
	{
		UE_LOG(LogNav3D, Warning, TEXT("BuildAdjacencyBetweenChunks: Invalid chunk actors"));
		return;
	}
	
	// Validate input chunks have tactical data
	if (!ChunkA->HasCompactTacticalData() || !ChunkB->HasCompactTacticalData())
	{
		UE_LOG(LogNav3D, Warning, TEXT("BuildAdjacencyBetweenChunks: Chunks missing tactical data - %s: %s, %s: %s"), 
		       *ChunkA->GetName(), ChunkA->HasTacticalData() ? TEXT("Yes") : TEXT("No"),
		       *ChunkB->GetName(), ChunkB->HasTacticalData() ? TEXT("Yes") : TEXT("No"));
		return;
	}
	
	// Validate chunks are actually adjacent
	if (!ChunkA->IsAdjacentToChunk(ChunkB))
	{
		UE_LOG(LogNav3D, Warning, TEXT("BuildAdjacencyBetweenChunks: Chunks %s and %s are not adjacent"), 
		       *ChunkA->GetName(), *ChunkB->GetName());
		return;
	}
	
	int32 ConnectionsCreated = 0;
	int32 ValidationErrors = 0;
	
	// Check each connection interface in ChunkA against ChunkB's regions
	for (const auto& InterfacePair : ChunkA->ConnectionInterfaces)
	{
		const FVector& FaceNormal = InterfacePair.Key;
		const FChunkConnectionInterface& Interface = InterfacePair.Value;
		
		// Validate interface data
		if (Interface.BoundaryRegionIds.Num() == 0)
		{
			UE_LOG(LogNav3D, Warning, TEXT("BuildAdjacencyBetweenChunks: Interface for face %s has no boundary regions"), 
			       *FaceNormal.ToString());
			ValidationErrors++;
			continue;
		}
		
		// Find the opposite face normal for ChunkB
		FVector OppositeFaceNormal = -FaceNormal;
		
		// Check if ChunkB has a matching interface
		const FChunkConnectionInterface* OppositeInterface = ChunkB->ConnectionInterfaces.Find(OppositeFaceNormal);
		if (!OppositeInterface)
		{
			UE_LOG(LogNav3D, VeryVerbose, TEXT("BuildAdjacencyBetweenChunks: No matching interface for face %s in chunk %s"), 
			       *OppositeFaceNormal.ToString(), *ChunkB->GetName());
			continue;
		}
		
		// Validate opposite interface data
		if (OppositeInterface->BoundaryRegionIds.Num() == 0)
		{
			UE_LOG(LogNav3D, Warning, TEXT("BuildAdjacencyBetweenChunks: Opposite interface for face %s has no boundary regions"), 
			       *OppositeFaceNormal.ToString());
			ValidationErrors++;
			continue;
		}
		
		// Connect boundary regions between the two chunks
		for (const int32 RegionIdA : Interface.BoundaryRegionIds)
		{
			const FBox* BoundsA = Interface.RegionBoundaryBoxes.Find(RegionIdA);
			if (!BoundsA)
			{
				UE_LOG(LogNav3D, Warning, TEXT("BuildAdjacencyBetweenChunks: Missing boundary box for region %d in chunk %s"), 
				       RegionIdA, *ChunkA->GetName());
				ValidationErrors++;
				continue;
			}
			
			// Validate bounds
			if (!BoundsA->IsValid)
			{
				UE_LOG(LogNav3D, Warning, TEXT("BuildAdjacencyBetweenChunks: Invalid boundary box for region %d in chunk %s"), 
				       RegionIdA, *ChunkA->GetName());
				ValidationErrors++;
				continue;
			}
			
			for (const int32 RegionIdB : OppositeInterface->BoundaryRegionIds)
			{
				const FBox* BoundsB = OppositeInterface->RegionBoundaryBoxes.Find(RegionIdB);
				if (!BoundsB)
				{
					UE_LOG(LogNav3D, Warning, TEXT("BuildAdjacencyBetweenChunks: Missing boundary box for region %d in chunk %s"), 
					       RegionIdB, *ChunkB->GetName());
					ValidationErrors++;
					continue;
				}
				
				// Validate bounds
				if (!BoundsB->IsValid)
				{
					UE_LOG(LogNav3D, Warning, TEXT("BuildAdjacencyBetweenChunks: Invalid boundary box for region %d in chunk %s"), 
					       RegionIdB, *ChunkB->GetName());
					ValidationErrors++;
					continue;
				}
				
				// Check if the boundary boxes overlap (indicating adjacency)
				if (BoundsA->Intersect(*BoundsB))
				{
					// Map local compact indices to consolidated compact global IDs
					auto ComputeGlobalCompactId = [&](const ANav3DDataChunkActor* Chunk, const int32 LocalIndex) -> uint16
					{
						uint16 Base = 1; // Consolidation starts at 1
						for (const ANav3DDataChunkActor* C : ChunkActors)
						{
							if (!C || !C->HasCompactTacticalData()) { continue; }
							if (C == Chunk)
							{
								if (LocalIndex >= 0 && LocalIndex < C->CompactTacticalData.Regions.Num())
								{
									return static_cast<uint16>(Base + LocalIndex);
								}
								return 0;
							}
							Base = static_cast<uint16>(Base + C->CompactTacticalData.Regions.Num());
						}
						return 0;
					};

					const uint16 GlobalRegionIdA = ComputeGlobalCompactId(ChunkA, RegionIdA);
					const uint16 GlobalRegionIdB = ComputeGlobalCompactId(ChunkB, RegionIdB);

					if (GlobalRegionIdA != 0 && GlobalRegionIdB != 0 && GlobalRegionIdA != GlobalRegionIdB)
					{
						// Update consolidated compact adjacency (bidirectional)
						uint64& MaskA = ConsolidatedCompactTacticalData.GlobalRegionAdjacency.FindOrAdd(GlobalRegionIdA);
						MaskA |= (1ULL << (GlobalRegionIdB - 1));

						uint64& MaskB = ConsolidatedCompactTacticalData.GlobalRegionAdjacency.FindOrAdd(GlobalRegionIdB);
						MaskB |= (1ULL << (GlobalRegionIdA - 1));

						ConnectionsCreated++;

						UE_LOG(LogNav3D, VeryVerbose, TEXT("Connected compact regions across chunks: %u <-> %u"),
							   GlobalRegionIdA, GlobalRegionIdB);
					}
					else
					{
						UE_LOG(LogNav3D, Warning, TEXT("BuildAdjacencyBetweenChunks: Failed to compute compact global IDs for local %d (chunk %s) and %d (chunk %s)"),
							   RegionIdA, *ChunkA->GetName(), RegionIdB, *ChunkB->GetName());
						ValidationErrors++;
					}
				}
			}
		}
	}
	
	// Log adjacency building results
	UE_LOG(LogNav3D, Log, TEXT("BuildAdjacencyBetweenChunks: %s <-> %s: %d connections created, %d validation errors"), 
	       *ChunkA->GetName(), *ChunkB->GetName(), ConnectionsCreated, ValidationErrors);
	
	// Validate adjacency consistency after building
	ValidateAdjacencyConsistency(ChunkA, ChunkB);
	
	// Update performance monitoring
	UpdatePerformanceStats();
}

void ANav3DData::ValidateAdjacencyConsistency(const ANav3DDataChunkActor* ChunkA, const ANav3DDataChunkActor* ChunkB) const
{
    if (!ChunkA || !ChunkB || !TacticalSettings.bEnableTacticalReasoning)
    {
        return;
    }

    if (!ChunkA->HasCompactTacticalData() || !ChunkB->HasCompactTacticalData())
    {
        return;
    }

    int32 ConsistencyErrors = 0;
    int32 ConsistencyWarnings = 0;

    // FIXED: Get the volume-wide region count, not chunk-local count
    const int32 VolumeWideRegionCount = GetTotalRegionsInVolume();
    
    auto ValidateChunk = [&](const ANav3DDataChunkActor* Chunk)
    {
        const FCompactTacticalData& C = Chunk->CompactTacticalData;
        const int32 ChunkLocalRegionCount = C.Regions.Num();

        // 1) Validate region ids exist and bit targets are in valid volume-wide range
        for (const auto& Pair : C.RegionAdjacency)
        {
            const uint8 RegionId = Pair.Key;
            
            // FIXED: Validate against volume-wide range (0-51), not chunk-local range
            if (RegionId >= 64 || RegionId >= VolumeWideRegionCount)
            {
                UE_LOG(LogNav3D, Warning, TEXT("Adjacency uses invalid region id: Chunk=%s Region=%d (VolumeRegionCount=%d)"), 
                       *Chunk->GetName(), RegionId, VolumeWideRegionCount);
                ConsistencyWarnings++;
            }
            
            const uint64 Mask = Pair.Value;
            for (int32 Bit = 0; Bit < 64; ++Bit)
            {
                if (Mask & (1ULL << Bit))
                {
                    // FIXED: Validate against volume-wide range (0-51), not chunk-local range
                    if (Bit >= VolumeWideRegionCount)
                    {
                        UE_LOG(LogNav3D, Error, TEXT("Adjacency target out of range: Chunk=%s %d -> %d (VolumeRegionCount=%d)"), 
                               *Chunk->GetName(), RegionId, Bit, VolumeWideRegionCount);
                        ConsistencyErrors++;
                    }
                }
            }
        }

        // 2) ConnectionInterfaces reference valid boundary regions (these are chunk-local)
        for (const auto& FacePair : Chunk->ConnectionInterfaces)
        {
            const FChunkConnectionInterface& Interface = FacePair.Value;
            for (const int32 BoundaryId : Interface.BoundaryRegionIds)
            {
                // KEEP ORIGINAL: ConnectionInterface IDs are chunk-local indices
                if (BoundaryId < 0 || BoundaryId >= ChunkLocalRegionCount)
                {
                    UE_LOG(LogNav3D, Warning, TEXT("Interface references invalid boundary region: Chunk=%s Region=%d (ChunkRegionCount=%d)"), 
                           *Chunk->GetName(), BoundaryId, ChunkLocalRegionCount);
                    ConsistencyWarnings++;
                }
            }
        }
    };

    ValidateChunk(ChunkA);
    ValidateChunk(ChunkB);

    // 3) Basic cross-chunk interface presence check (both sides should have at least one interface)
    if (ChunkA->ConnectionInterfaces.Num() == 0 || ChunkB->ConnectionInterfaces.Num() == 0)
    {
        UE_LOG(LogNav3D, Warning, TEXT("One or both chunks missing ConnectionInterfaces: %s(%d), %s(%d)"),
            *ChunkA->GetName(), ChunkA->ConnectionInterfaces.Num(), *ChunkB->GetName(), ChunkB->ConnectionInterfaces.Num());
        ConsistencyWarnings++;
    }

    UE_LOG(LogNav3D, Verbose, TEXT("ValidateAdjacencyConsistency (compact): %s <-> %s: %d errors, %d warnings"),
           *ChunkA->GetName(), *ChunkB->GetName(), ConsistencyErrors, ConsistencyWarnings);
}

int32 ANav3DData::GetTotalRegionsInVolume() const
{
    // Option 1: Count from all loaded chunks (if available)
    int32 MaxRegionId = -1;
    for (const ANav3DDataChunkActor* ChunkActor : ChunkActors)
    {
        if (ChunkActor && ChunkActor->HasCompactTacticalData())
        {
            const FCompactTacticalData& Data = ChunkActor->CompactTacticalData;
            
            // Find the highest region ID referenced in adjacency data
            for (const auto& AdjPair : Data.RegionAdjacency)
            {
                MaxRegionId = FMath::Max(MaxRegionId, static_cast<int32>(AdjPair.Key));
                
                const uint64 Mask = AdjPair.Value;
                for (int32 Bit = 0; Bit < 64; ++Bit)
                {
                    if (Mask & (1ULL << Bit))
                    {
                        MaxRegionId = FMath::Max(MaxRegionId, Bit);
                    }
                }
            }
        }
    }
    
    // Return total region count (MaxRegionId + 1, since IDs are 0-based)
    const int32 TotalRegions = MaxRegionId + 1;
    
    // Fallback to reasonable default if no data found
    return TotalRegions > 0 ? TotalRegions : 64;
}

void ANav3DData::BuildCrossChunkVisibility(const TArray<ANav3DDataChunkActor*>& LoadedChunks)
{
	if (LoadedChunks.Num() == 0)
	{
		return;
	}
    
	// Initialize tactical reasoning if not already done
	if (!TacticalReasoning.IsValid())
	{
		if (!InitializeTacticalReasoning())
		{
			UE_LOG(LogNav3D, Error, TEXT("Failed to initialize tactical reasoning for visibility build"));
			return;
		}
	}
    
	// Capture the region count before starting async operation
	const int32 RegionCount = ConsolidatedTacticalData.AllLoadedRegions.Num();
    
	UE_LOG(LogNav3D, Log, TEXT("Starting async cross-chunk visibility build for %d regions"), RegionCount);
    
	// Use the MEMBER TacticalReasoning object, not a stack-allocated one
	TacticalReasoning->BuildVisibilitySetsForLoadedRegionsAsync(ConsolidatedTacticalData, [this, RegionCount]()
	{
		UE_LOG(LogNav3D, Verbose, TEXT("Completed async cross-chunk visibility build for %d regions"), RegionCount);
        
		RequestDrawingUpdate();
	});
}

void ANav3DData::BuildVisibilitySetsForLoadedRegionsAsync(
	FConsolidatedTacticalData& ConsolidatedData, 
	TFunction<void()> OnCompleteCallback) const
{
	if (!TacticalSettings.bEnableTacticalReasoning || ConsolidatedData.IsEmpty())
	{
		if (OnCompleteCallback)
		{
			OnCompleteCallback();
		}
		return;
	}
    
	// Initialize tactical reasoning if not already done
	if (!TacticalReasoning.IsValid())
	{
		// Cast away const for initialization (this is a lazy initialization pattern)
		ANav3DData* MutableThis = const_cast<ANav3DData*>(this);
		if (!MutableThis->InitializeTacticalReasoning())
		{
			UE_LOG(LogNav3D, Error, TEXT("Failed to initialize tactical reasoning"));
			if (OnCompleteCallback)
			{
				OnCompleteCallback();
			}
			return;
		}
	}
    
	// Capture the region count before starting async operation
	const int32 RegionCount = ConsolidatedData.AllLoadedRegions.Num();
    
	// Use the tactical reasoning system to build visibility
	TacticalReasoning->BuildVisibilitySetsForLoadedRegionsAsync(ConsolidatedData, [this, RegionCount, OnCompleteCallback]()
	{
		UE_LOG(LogNav3D, Verbose, TEXT("Built visibility for %d regions"), RegionCount);
        
		if (OnCompleteCallback)
		{
			OnCompleteCallback();
		}
	});
}

// =============================================================================
// PERFORMANCE MONITORING
// =============================================================================

void ANav3DData::UpdatePerformanceStats()
{
	if (!TacticalSettings.bEnableTacticalReasoning)
	{
		return;
	}

	// Reset/accumulate compact-centric stats
	PerformanceStats.TotalRegions = 0;
	PerformanceStats.LoadedChunks = 0;
	PerformanceStats.TotalAdjacencies = 0;
	PerformanceStats.CrossChunkAdjacencies = 0;
	PerformanceStats.IntraChunkAdjacencies = 0;
	PerformanceStats.TotalVisibilityPairs = 0;
	PerformanceStats.EstimatedMemoryUsage = 0.0f;

	// Per-chunk compact stats
	for (const ANav3DDataChunkActor* Chunk : ChunkActors)
	{
		if (!Chunk || !Chunk->HasCompactTacticalData()) { continue; }
		PerformanceStats.LoadedChunks++;

		const FCompactTacticalData& C = Chunk->CompactTacticalData;
		PerformanceStats.TotalRegions += C.Regions.Num();
		PerformanceStats.TotalAdjacencies += C.RegionAdjacency.Num();

		// Memory estimate (compact only)
		PerformanceStats.EstimatedMemoryUsage += C.Regions.Num() * sizeof(FCompactRegion);
		PerformanceStats.EstimatedMemoryUsage += C.RegionAdjacency.Num() * sizeof(uint64);
		PerformanceStats.EstimatedMemoryUsage += C.VisibilityMatrix.SparseReferences.Num() * sizeof(uint64);
	}

	// Consolidated compact stats
	PerformanceStats.TotalRegions += ConsolidatedCompactTacticalData.GetRegionCount();
	PerformanceStats.TotalAdjacencies += ConsolidatedCompactTacticalData.GlobalRegionAdjacency.Num();

	// Visibility pairs: count bits across all matrices
	for (const auto& VolPair : ConsolidatedCompactTacticalData.VolumeVisibilityData)
	{
		for (const auto& RefPair : VolPair.Value.SparseReferences)
		{
			PerformanceStats.TotalVisibilityPairs += FVolumeRegionMatrix::CountBits(RefPair.Value);
		}
	}

	PerformanceStats.LastUpdateTime = FPlatformTime::Seconds();

	UE_LOG(LogNav3D, VeryVerbose, TEXT("Performance stats updated (compact): Regions=%d, AdjacencyEntries=%d, VisibilityPairs=%d, Memory=%.2f MB"),
	       PerformanceStats.TotalRegions,
	       PerformanceStats.TotalAdjacencies,
	       PerformanceStats.TotalVisibilityPairs,
	       PerformanceStats.EstimatedMemoryUsage / (1024.0f * 1024.0f));
}

float ANav3DData::EstimateMemoryUsage() const
{
	float MemoryUsage = 0.0f;
	
	// Estimate memory for regions
	MemoryUsage += ConsolidatedTacticalData.AllLoadedRegions.Num() * sizeof(FNav3DRegion);
	
	// Estimate memory for adjacency data
	for (const auto& AdjPair : ConsolidatedTacticalData.RegionAdjacency)
	{
		MemoryUsage += sizeof(int32); // Key
		MemoryUsage += sizeof(FRegionIdArray); // Value structure
		MemoryUsage += AdjPair.Value.Num() * sizeof(int32); // Array contents
	}
	
	// Estimate memory for visibility data
	for (const auto& VisPair : ConsolidatedTacticalData.RegionVisibility)
	{
		MemoryUsage += sizeof(int32); // Key
		MemoryUsage += sizeof(FRegionIdArray); // Value structure
		MemoryUsage += VisPair.Value.Num() * sizeof(int32); // Array contents
	}
	
	// Estimate memory for source chunks array
	MemoryUsage += ConsolidatedTacticalData.SourceChunks.Num() * sizeof(TWeakObjectPtr<ANav3DDataChunkActor>);
	
	return MemoryUsage;
}

void ANav3DData::LogPerformanceStats() const
{
	if (!TacticalSettings.bEnableTacticalReasoning)
	{
		UE_LOG(LogNav3D, Display, TEXT("Tactical reasoning disabled - no performance stats available"));
		return;
	}
	
	UE_LOG(LogNav3D, Display, TEXT("=== NAV3D PERFORMANCE STATS ==="));
	UE_LOG(LogNav3D, Display, TEXT("Total Regions: %d"), PerformanceStats.TotalRegions);
	UE_LOG(LogNav3D, Display, TEXT("Loaded Chunks: %d"), PerformanceStats.LoadedChunks);
	UE_LOG(LogNav3D, Display, TEXT("Total Adjacencies: %d"), PerformanceStats.TotalAdjacencies);
	UE_LOG(LogNav3D, Display, TEXT("  - Intra-chunk: %d"), PerformanceStats.IntraChunkAdjacencies);
	UE_LOG(LogNav3D, Display, TEXT("  - Cross-chunk: %d"), PerformanceStats.CrossChunkAdjacencies);
	UE_LOG(LogNav3D, Display, TEXT("Total Visibility Pairs: %d"), PerformanceStats.TotalVisibilityPairs);
	UE_LOG(LogNav3D, Display, TEXT("Estimated Memory Usage: %.2f MB"), PerformanceStats.EstimatedMemoryUsage / (1024.0f * 1024.0f));
	UE_LOG(LogNav3D, Display, TEXT("Last Update: %.2f seconds ago"), FPlatformTime::Seconds() - PerformanceStats.LastUpdateTime);
	UE_LOG(LogNav3D, Display, TEXT("==============================="));
}

// =============================================================================
// TACTICAL API (PUBLIC INTERFACE)
// =============================================================================


const FNav3DVolumeNavigationData* ANav3DData::GetVolumeNavigationDataContainingPoints(
	const TArray<FVector>& Points) const
{
	// Find the volume that contains the most points
	TMap<const FNav3DVolumeNavigationData*, int32> VolumePointCounts;
	
	for (const FVector& Point : Points)
	{
		for (ANav3DDataChunkActor* ChunkActor : ChunkActors)
		{
			if (!ChunkActor || !ChunkActor->DataChunkActorBounds.IsInside(Point))
			{
				continue;
			}
			
			for (const UNav3DDataChunk* Chunk : ChunkActor->Nav3DChunks)
			{
				if (Chunk)
				{
					if (const FNav3DVolumeNavigationData* VolumeData = Chunk->GetVolumeNavigationData())
					{
						if (VolumeData->GetVolumeBounds().IsInside(Point))
						{
							int32& Count = VolumePointCounts.FindOrAdd(VolumeData);
							Count++;
							break; // Found volume for this point
						}
					}
				}
			}
		}
	}
	
	// Return the volume with the most points
	const FNav3DVolumeNavigationData* BestVolume = nullptr;
	int32 MaxPoints = 0;
	
	for (const auto& Pair : VolumePointCounts)
	{
		if (Pair.Value > MaxPoints)
		{
			MaxPoints = Pair.Value;
			BestVolume = Pair.Key;
		}
	}
	
	return BestVolume;
}

const FNav3DVolumeNavigationData* ANav3DData::GetVolumeNavigationDataContainingPoint(const FVector& Point) const
{
	// Use spatial subsystem for fast chunk lookup
	const UNav3DWorldSubsystem* Subsystem = GetSubsystem();
	if (!Subsystem)
	{
		// Fallback to linear search using GetAllChunkActors() to avoid null entries
		for (ANav3DDataChunkActor* ChunkActor : GetAllChunkActors())
		{
			if (ChunkActor && ChunkActor->ContainsPoint(Point))
			{
				if (ChunkActor->Nav3DChunks.Num() > 0)
				{
					return ChunkActor->Nav3DChunks[0]->GetVolumeNavigationData();
				}
			}
		}
		return nullptr;
	}
	
	TArray<ANav3DDataChunkActor*> ContainingActors;
	Subsystem->QueryActorsInBounds(FBox(Point, Point), ContainingActors);
	
	// If spatial subsystem returns no results, fall back to linear search
	if (ContainingActors.Num() == 0)
	{
		UE_LOG(LogNav3D, Verbose, TEXT("GetVolumeNavigationDataContainingPoint: Spatial subsystem found no candidates, falling back to linear search"));
		for (ANav3DDataChunkActor* ChunkActor : GetAllChunkActors())
		{
			if (ChunkActor && ChunkActor->ContainsPoint(Point))
			{
				if (ChunkActor->Nav3DChunks.Num() > 0)
				{
					return ChunkActor->Nav3DChunks[0]->GetVolumeNavigationData();
				}
			}
		}
		return nullptr;
	}
	
	for (ANav3DDataChunkActor* ChunkActor : ContainingActors)
	{
		if (ChunkActor && ChunkActor->ContainsPoint(Point))
		{
			// Return navigation data from chunk
			if (ChunkActor->Nav3DChunks.Num() > 0)
			{
				return ChunkActor->Nav3DChunks[0]->GetVolumeNavigationData();
			}
		}
	}
    
	return nullptr;
}

bool ANav3DData::FindBestLocation(
	const FVector& StartPosition,
	const TArray<FVector>& ObserverPositions,
	TArray<FPositionCandidate>& OutCandidatePositions,
	const ETacticalVisibility Visibility,
	const ETacticalDistance DistancePreference,
	const ETacticalRegion RegionPreference,
	const bool bForceNewRegion,
	const bool bUseRaycasting) const
{
	// Use the unified tactical location method that handles both compact and build data
	return FindBestTacticalLocation(
		StartPosition,
		ObserverPositions,
		Visibility,
		DistancePreference,
		RegionPreference,
		bForceNewRegion,
		bUseRaycasting,
		OutCandidatePositions);
}

float ANav3DData::GetVoxelExtent() const
{
	// Get the agent radius from NavConfig
	const FNavAgentProperties& NavConfig = GetConfig();
	return NavConfig.AgentRadius * 2.0f;
}

int32 ANav3DData::GetLayerCount() const
{
	if (ChunkActors.Num() == 0) return 0;
	
	// Get layer count from first available chunk actor
	for (const ANav3DDataChunkActor* ChunkActor : ChunkActors)
	{
		if (!ChunkActor) continue;
		
		for (const UNav3DDataChunk* Chunk : ChunkActor->Nav3DChunks)
		{
			if (!Chunk) continue;
			
			const FNav3DVolumeNavigationData* VolumeData = Chunk->GetVolumeNavigationData();
			if (VolumeData && VolumeData->GetData().IsValid())
			{
				return VolumeData->GetData().GetLayerCount();
			}
		}
	}
	
	return 0;
}


// ============================================================================
// CHUNK ACTOR MANAGEMENT METHODS
// ============================================================================

void ANav3DData::RegisterChunkActor(ANav3DDataChunkActor* ChunkActor)
{
	if (!ChunkActor || ChunkActor->Nav3DChunks.Num() == 0)
	{
		UE_LOG(LogNav3D, Warning, TEXT("Cannot register chunk actor: invalid or empty"));
		return;
	}
	
	// Check if already registered
	if (ChunkActors.Contains(ChunkActor))
	{
		UE_LOG(LogNav3D, Verbose, TEXT("Chunk actor already registered: %s"), *ChunkActor->GetName());
		return;
	}
	
	ChunkActors.Add(ChunkActor);
	
	UE_LOG(LogNav3D, Log, TEXT("Registered chunk actor: %s with bounds %s"), 
	       *ChunkActor->GetName(), *ChunkActor->DataChunkActorBounds.ToString());
	NotifyChunksChanged();

	// Tactical consolidated data update
	if (TacticalSettings.bEnableTacticalReasoning)
	{
		OnChunkActorLoaded(ChunkActor);
	}
}

void ANav3DData::UnregisterChunkActor(ANav3DDataChunkActor* ChunkActor)
{
	if (!ChunkActor)
	{
		return;
	}
	
	const int32 RemovedCount = ChunkActors.RemoveAllSwap([ChunkActor](const ANav3DDataChunkActor* Actor)
	{
		return Actor == ChunkActor;
	});
	
	if (RemovedCount > 0)
	{
		UE_LOG(LogNav3D, Log, TEXT("Unregistered chunk actor: %s"), *ChunkActor->GetName());
		const FBox RemovedBounds = ChunkActor->DataChunkActorBounds;
		NotifyChunksChanged();
		if (TacticalSettings.bEnableTacticalReasoning)
		{
			OnChunkActorUnloaded(ChunkActor);
		}
		// Purge and rebuild adjacency around the removed chunk
		TArray<ANav3DDataChunkActor*> Remaining = GetAllChunkActors();
		float VoxelSize = 0.0f;
		for (ANav3DDataChunkActor* Other : Remaining)
		{
			if (!Other) continue;
			// Remove portal links pointing into removed bounds and rebuild lookup
			// No transient portal lookup to reset
			for (FNav3DChunkAdjacency& Adj : Other->ChunkAdjacency)
			{
				// Filter compact portals by removed bounds
				Adj.CompactPortals.RemoveAllSwap([&](const FCompactPortal& CP)
				{
					const UNav3DDataChunk* AnyChunk = Other->Nav3DChunks.Num() > 0 ? Other->Nav3DChunks[0] : nullptr;
					const FNav3DVolumeNavigationData* Vol = AnyChunk ? AnyChunk->GetVolumeNavigationData() : nullptr;
					const FVector WorldPos = Vol ? Vol->GetLeafNodePositionFromMortonCode(CP.Local) : FVector::ZeroVector;
					return RemovedBounds.IsInside(WorldPos);
				}, EAllowShrinking::No);

				// No lookup rebuild; CompactPortals are the source of truth
			}

			if (Other->Nav3DChunks.Num() > 0 && VoxelSize <= 0.0f)
			{
				VoxelSize = FNav3DUtils::GetChunkLeafNodeSize(Other->Nav3DChunks[0]);
			}
		}
		if (VoxelSize > 0.0f)
		{
			for (ANav3DDataChunkActor* A : Remaining)
			{
				if (!A) continue;
				if (!A->DataChunkActorBounds.ExpandBy(VoxelSize).Intersect(RemovedBounds)) continue;
				for (ANav3DDataChunkActor* B : Remaining)
				{
					if (!B || B == A) continue;
					if (A->DataChunkActorBounds.ExpandBy(VoxelSize).Intersect(B->DataChunkActorBounds))
					{
						if (static_cast<FNav3DDataGenerator*>(GetGenerator()))
						{
							FNav3DDataGenerator::BuildAdjacencyBetweenTwoChunkActors(A, B, VoxelSize);
						}
					}
				}
			}
		}
	}
}


void ANav3DData::NotifyChunksChanged()
{
#if WITH_EDITOR
	// Bump revision for details customizations to detect changes
#if WITH_EDITORONLY_DATA
	++ChunkRevision;
#endif
	RequestDrawingUpdate(true);
#endif
}

TArray<ANav3DDataChunkActor*> ANav3DData::GetAllChunkActors() const
{
	TArray<ANav3DDataChunkActor*> ValidActors;
	ValidActors.Reserve(ChunkActors.Num());
	
	int32 InvalidCount = 0;
	for (ANav3DDataChunkActor* ChunkActor : ChunkActors)
	{
		if (ChunkActor && IsValid(ChunkActor))
		{
			ValidActors.Add(ChunkActor);
		}
		else
		{
			InvalidCount++;
		}
	}
	
	// Log if we found invalid actors (but don't spam the log)
	if (InvalidCount > 0)
	{
		UE_LOG(LogNav3D, Warning, TEXT("GetAllChunkActors: Found %d invalid chunk actors out of %d total"), 
		       InvalidCount, ChunkActors.Num());
	}
	
	return ValidActors;
}

void ANav3DData::CleanupInvalidChunkActors()
{
	const int32 OriginalCount = ChunkActors.Num();
	if (OriginalCount == 0)
	{
		return;
	}
	
	// Remove invalid actors from the array
	const int32 RemovedCount = ChunkActors.RemoveAllSwap([](const ANav3DDataChunkActor* ChunkActor)
	{
		return !ChunkActor || !IsValid(ChunkActor);
	});
	
	if (RemovedCount > 0)
	{
		UE_LOG(LogNav3D, Log, TEXT("CleanupInvalidChunkActors: Removed %d invalid chunk actors (was %d, now %d)"), 
		       RemovedCount, OriginalCount, ChunkActors.Num());
		
		// Notify that chunks have changed
		NotifyChunksChanged();
		
		// Rebuild adjacency for remaining chunks since some may have been removed
		TArray<ANav3DDataChunkActor*> RemainingActors = GetAllChunkActors();
		for (ANav3DDataChunkActor* Actor : RemainingActors)
		{
			if (Actor)
			{
				// No transient portal lookup to reset
				// Adjacency will be rebuilt when needed
			}
		}
	}
}

int32 ANav3DData::GetInvalidChunkActorCount() const
{
	int32 InvalidCount = 0;
	for (const ANav3DDataChunkActor* ChunkActor : ChunkActors)
	{
		if (!ChunkActor || !IsValid(ChunkActor))
		{
			InvalidCount++;
		}
	}
	return InvalidCount;
}

void ANav3DData::CleanupInvalidChunkActorsBP()
{
	const int32 InvalidCount = GetInvalidChunkActorCount();
	if (InvalidCount > 0)
	{
		UE_LOG(LogNav3D, Log, TEXT("CleanupInvalidChunkActorsBP: Cleaning up %d invalid chunk actors"), InvalidCount);
		CleanupInvalidChunkActors();
	}
	else
	{
		UE_LOG(LogNav3D, Log, TEXT("CleanupInvalidChunkActorsBP: No invalid chunk actors found"));
	}
}

void ANav3DData::CleanupAllInvalidActors()
{
	const int32 InvalidChunkCount = GetInvalidChunkActorCount();
	
	if (InvalidChunkCount > 0)
	{
		UE_LOG(LogNav3D, Log, TEXT("CleanupAllInvalidActors: Cleaning up %d invalid chunk actors"), InvalidChunkCount);
		CleanupInvalidChunkActors();
		UE_LOG(LogNav3D, Log, TEXT("CleanupAllInvalidActors: Cleanup completed"));
	}
	else
	{
		UE_LOG(LogNav3D, Log, TEXT("CleanupAllInvalidActors: No invalid actors found"));
	}
}

TArray<FBox> ANav3DData::GetPartitionedVolumes() const
{
	TArray<FBox> Volumes;
	Volumes.Reserve(ChunkActors.Num());
	
	for (const ANav3DDataChunkActor* ChunkActor : ChunkActors)
	{
		if (ChunkActor)
		{
			Volumes.Add(ChunkActor->DataChunkActorBounds);
		}
	}
	
	return Volumes;
}

TArray<FBox> ANav3DData::GetAllDiscoverableVolumes() const
{
	// Render threads can't touch actor iterators; return cached data
	if (!IsInGameThread())
	{
		FScopeLock ScopeLock(&CachedDiscoverableVolumesMutex);
		return CachedDiscoverableVolumes;
	}

	TArray<FBox> AllVolumes;
	
	if (UWorld* World = GetWorld())
	{
		// Find all Nav3DBoundsVolume actors in the world
		for (TActorIterator<ANav3DBoundsVolume> ActorIterator(World); ActorIterator; ++ActorIterator)
		{
			if (const ANav3DBoundsVolume* BoundsVolume = *ActorIterator; BoundsVolume && IsValid(BoundsVolume))
			{
				if (const FBox VolumeBounds = BoundsVolume->GetComponentsBoundingBox(true);
					VolumeBounds.IsValid)
				{
					AllVolumes.Add(VolumeBounds);
				}
			}
		}
		
		// If no bounds volumes found, use navigation system bounds
		if (AllVolumes.Num() == 0)
		{
			if (const UNavigationSystemV1* NavSys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(World))
			{
				TArray<FBox> SupportedNavigationBounds;
				NavSys->GetNavigationBoundsForNavData(*this, SupportedNavigationBounds);
				AllVolumes = SupportedNavigationBounds;
			}
		}
	}

	{
		FScopeLock ScopeLock(&CachedDiscoverableVolumesMutex);
		CachedDiscoverableVolumes = AllVolumes;
	}
	
	return AllVolumes;
}

// =============================================================================
// DEBUG COMMANDS FOR CHUNK ADJACENCY VALIDATION
// =============================================================================

void ANav3DData::DebugPrintChunkAdjacency()
{
	UE_LOG(LogNav3D, Display, TEXT("=== CHUNK ADJACENCY DEBUG ==="));
	UE_LOG(LogNav3D, Display, TEXT("Total chunk actors: %d"), ChunkActors.Num());
	
	int32 TotalAdjacencies = 0;
	int32 ActorsWithAdjacency = 0;
	
	for (ANav3DDataChunkActor* ChunkActor : ChunkActors)
	{
		if (!ChunkActor)
		{
			continue;
		}
		
		const int32 AdjacencyCount = ChunkActor->ChunkAdjacency.Num();
		if (AdjacencyCount > 0)
		{
			ActorsWithAdjacency++;
			TotalAdjacencies += AdjacencyCount;
			
			UE_LOG(LogNav3D, Display, TEXT("Actor: %s - %d adjacencies"), 
			       *ChunkActor->GetName(), AdjacencyCount);
			
			for (const FNav3DChunkAdjacency& Adj : ChunkActor->ChunkAdjacency)
			{
				if (Adj.OtherChunkActor.IsValid())
				{
					UE_LOG(LogNav3D, Display, TEXT("  -> %s (Weight: %.2f, CompactPortals: %d)"), 
					       *Adj.OtherChunkActor->GetName(), 
					       Adj.ConnectionWeight,
					       Adj.CompactPortals.Num());
				}
				else
				{
					UE_LOG(LogNav3D, Warning, TEXT("  -> INVALID REFERENCE"));
				}
			}
		}
		else
		{
			UE_LOG(LogNav3D, Warning, TEXT("Actor: %s - NO ADJACENCY DATA"), *ChunkActor->GetName());
		}
	}
	
	UE_LOG(LogNav3D, Display, TEXT("=== SUMMARY ==="));
	UE_LOG(LogNav3D, Display, TEXT("Actors with adjacency: %d/%d"), ActorsWithAdjacency, ChunkActors.Num());
	UE_LOG(LogNav3D, Display, TEXT("Total adjacencies: %d"), TotalAdjacencies);
}

void ANav3DData::ValidateAllChunkAdjacency()
{
	UE_LOG(LogNav3D, Display, TEXT("=== CHUNK ADJACENCY VALIDATION ==="));
	
	int32 ValidationErrors = 0;
	int32 ValidationWarnings = 0;
	
	for (ANav3DDataChunkActor* ChunkActor : ChunkActors)
	{
		if (!ChunkActor)
		{
			ValidationErrors++;
			UE_LOG(LogNav3D, Error, TEXT("NULL chunk actor found in ChunkActors array"));
			continue;
		}
		
		// Check if actor has adjacency data
		if (ChunkActor->ChunkAdjacency.Num() == 0)
		{
			ValidationWarnings++;
			UE_LOG(LogNav3D, Warning, TEXT("Actor %s has no adjacency data"), *ChunkActor->GetName());
			continue;
		}
		
		// Validate each adjacency
		for (const FNav3DChunkAdjacency& Adj : ChunkActor->ChunkAdjacency)
		{
			// Check if reference is valid
			if (!Adj.OtherChunkActor.IsValid())
			{
				ValidationErrors++;
				UE_LOG(LogNav3D, Error, TEXT("Actor %s has invalid adjacency reference"), *ChunkActor->GetName());
				continue;
			}
			
			// Check if adjacency is reciprocal
			ANav3DDataChunkActor* OtherActor = Adj.OtherChunkActor.Get();
			bool bHasReciprocal = false;
			
			for (const FNav3DChunkAdjacency& OtherAdj : OtherActor->ChunkAdjacency)
			{
				if (OtherAdj.OtherChunkActor.Get() == ChunkActor)
				{
					bHasReciprocal = true;
					break;
				}
			}
			
			if (!bHasReciprocal)
			{
				ValidationErrors++;
				UE_LOG(LogNav3D, Error, TEXT("Non-reciprocal adjacency: %s -> %s"), 
				       *ChunkActor->GetName(), *OtherActor->GetName());
			}
			
			// Check if portals are valid (compact only)
			if (Adj.CompactPortals.Num() == 0)
			{
				ValidationWarnings++;
				UE_LOG(LogNav3D, Warning, TEXT("Actor %s -> %s has no compact portals"), 
				       *ChunkActor->GetName(), *OtherActor->GetName());
			}
		}
	}
	
	UE_LOG(LogNav3D, Display, TEXT("=== VALIDATION COMPLETE ==="));
	UE_LOG(LogNav3D, Display, TEXT("Errors: %d, Warnings: %d"), ValidationErrors, ValidationWarnings);
	
	if (ValidationErrors == 0)
	{
		UE_LOG(LogNav3D, Display, TEXT("✅ Chunk adjacency validation PASSED"));
	}
	else
	{
		UE_LOG(LogNav3D, Error, TEXT("❌ Chunk adjacency validation FAILED with %d errors"), ValidationErrors);
	}
}

bool ANav3DData::ValidateConsolidatedTacticalData() const
{
	if (!TacticalSettings.bEnableTacticalReasoning)
	{
		UE_LOG(LogNav3D, Warning, TEXT("ValidateConsolidatedTacticalData: Tactical reasoning is disabled"));
		return false;
	}
	
	if (ConsolidatedCompactTacticalData.IsEmpty())
	{
		UE_LOG(LogNav3D, Warning, TEXT("ValidateConsolidatedTacticalData: No compact tactical data to validate"));
		return false;
	}
	
	const auto& CompactData = ConsolidatedCompactTacticalData;
	int32 ValidationErrors = 0;
	bool bIsValid = true;
	
	// Validate compact regions
	for (const auto& RegionPair : CompactData.AllLoadedRegions)
	{
		const uint16 GlobalRegionId = RegionPair.Key;
		const FCompactRegion& CompactRegion = RegionPair.Value;
		
		// LayerIndex sanity check
		if (CompactRegion.LayerIndex > 10)
		{
			UE_LOG(LogNav3D, Error, TEXT("Compact region %d has suspicious layer index: %d"), GlobalRegionId, CompactRegion.LayerIndex);
			ValidationErrors++;
			bIsValid = false;
		}
	}
	
	// Validate compact adjacency bitmasks
	for (const auto& AdjPair : CompactData.GlobalRegionAdjacency)
	{
		const uint16 RegionId = AdjPair.Key;
		const uint64 AdjacencyMask = AdjPair.Value;
		
		// Region must exist
		if (!CompactData.AllLoadedRegions.Contains(RegionId))
		{
			UE_LOG(LogNav3D, Error, TEXT("Adjacency entry for non-existent compact region %d"), RegionId);
			ValidationErrors++;
			bIsValid = false;
		}
		
		// Reasonable number of connections
		const int32 ConnectionCount = FVolumeRegionMatrix::CountBits(AdjacencyMask);
		if (ConnectionCount > 32)
		{
			UE_LOG(LogNav3D, Warning, TEXT("Compact region %d has suspicious adjacency count: %d"), RegionId, ConnectionCount);
		}
	}
	
	UE_LOG(LogNav3D, Log, TEXT("Compact tactical data validation: %s (%d errors)"), bIsValid ? TEXT("PASSED") : TEXT("FAILED"), ValidationErrors);
	return bIsValid;
}

void ANav3DData::FilterConsolidatedDataToSelectedRegions(const TArray<int32>& SelectedRegionIds)
{
	if (SelectedRegionIds.Num() == 0)
	{
		UE_LOG(LogNav3D, Warning, TEXT("FilterConsolidatedDataToSelectedRegions: No regions selected"));
		return;
	}
	
	// Create a set for fast lookup and mapping from old IDs to new sequential IDs
	TSet<int32> SelectedSet(SelectedRegionIds);
	TMap<int32, int32> OldToNewIdMapping;
	
	// Create mapping from old region IDs to new sequential IDs (0 to SelectedRegionIds.Num()-1)
	for (int32 i = 0; i < SelectedRegionIds.Num(); ++i)
	{
		OldToNewIdMapping.Add(SelectedRegionIds[i], i);
	}
	
	// Filter and renumber regions
	TArray<FNav3DRegion> FilteredRegions;
	for (const FNav3DRegion& Region : ConsolidatedTacticalData.AllLoadedRegions)
	{
		if (SelectedSet.Contains(Region.Id))
		{
			FNav3DRegion RenumberedRegion = Region;
			RenumberedRegion.Id = OldToNewIdMapping[Region.Id];
			FilteredRegions.Add(RenumberedRegion);
		}
	}
	
	// Filter and renumber adjacency data
	TMap<int32, FRegionIdArray> FilteredAdjacency;
	for (const auto& AdjPair : ConsolidatedTacticalData.RegionAdjacency)
	{
		int32 OldRegionId = AdjPair.Key;
		if (SelectedSet.Contains(OldRegionId))
		{
			int32 NewRegionId = OldToNewIdMapping[OldRegionId];
			FRegionIdArray FilteredAdjacentIds;
			for (int32 OldAdjacentId : AdjPair.Value.GetArray())
			{
				if (SelectedSet.Contains(OldAdjacentId))
				{
					int32 NewAdjacentId = OldToNewIdMapping[OldAdjacentId];
					FilteredAdjacentIds.Add(NewAdjacentId);
				}
			}
			FilteredAdjacency.Add(NewRegionId, FilteredAdjacentIds);
		}
	}
	
	// Filter and renumber visibility data
	TMap<int32, FRegionIdArray> FilteredVisibility;
	for (const auto& VisPair : ConsolidatedTacticalData.RegionVisibility)
	{
		int32 OldRegionId = VisPair.Key;
		if (SelectedSet.Contains(OldRegionId))
		{
			int32 NewRegionId = OldToNewIdMapping[OldRegionId];
			FRegionIdArray FilteredVisibleIds;
			for (int32 OldVisibleId : VisPair.Value.GetArray())
			{
				if (SelectedSet.Contains(OldVisibleId))
				{
					int32 NewVisibleId = OldToNewIdMapping[OldVisibleId];
					FilteredVisibleIds.Add(NewVisibleId);
				}
			}
			FilteredVisibility.Add(NewRegionId, FilteredVisibleIds);
		}
	}
	
	// Update consolidated data
	ConsolidatedTacticalData.AllLoadedRegions = FilteredRegions;
	ConsolidatedTacticalData.RegionAdjacency = FilteredAdjacency;
	ConsolidatedTacticalData.RegionVisibility = FilteredVisibility;
	
	UE_LOG(LogNav3D, Log, TEXT("Filtered and renumbered consolidated data: %d regions (IDs 0-%d), %d adjacency entries, %d visibility entries"),
	       FilteredRegions.Num(), FilteredRegions.Num() - 1, FilteredAdjacency.Num(), FilteredVisibility.Num());
}

// =============================================================================
// COMPACT CONSOLIDATED TACTICAL DATA HELPER METHODS
// =============================================================================

void ANav3DData::ConsolidateCompactRegionsFromChunks(const TArray<ANav3DDataChunkActor*>& LoadedChunks)
{
	ConsolidatedCompactTacticalData.AllLoadedRegions.Empty();
	GlobalToLocalRegionMapping.Empty();

	uint16 GlobalRegionId = 1; // Start from 1 to avoid 0 (invalid)

	for (ANav3DDataChunkActor* ChunkActor : LoadedChunks)
	{
		if (!ChunkActor || ChunkActor->CompactTacticalData.IsEmpty())
		{
			continue;
		}

		const FCompactTacticalData& CompactData = ChunkActor->CompactTacticalData;
		const uint16 VolumeID = CompactData.VolumeID;

		for (int32 LocalRegionIndex = 0; LocalRegionIndex < CompactData.Regions.Num(); ++LocalRegionIndex)
		{
			const FCompactRegion& LocalRegion = CompactData.Regions[LocalRegionIndex];

			FRegionMapping Mapping;
			Mapping.VolumeID = VolumeID;
			Mapping.LocalRegionIndex = static_cast<uint8>(LocalRegionIndex);
			Mapping.ChunkActor = ChunkActor;
			GlobalToLocalRegionMapping.Add(GlobalRegionId, Mapping);

			ConsolidatedCompactTacticalData.AllLoadedRegions.Add(GlobalRegionId, LocalRegion);

			UE_LOG(LogNav3D, VeryVerbose, TEXT("Added global region %d from volume %d, local index %d"),
				GlobalRegionId, VolumeID, LocalRegionIndex);

			GlobalRegionId = (GlobalRegionId == 0xFFFF ? 1 : static_cast<uint16>(GlobalRegionId + 1));
		}
	}

	UE_LOG(LogNav3D, Log, TEXT("Consolidated %d compact regions from %d chunks"),
		ConsolidatedCompactTacticalData.AllLoadedRegions.Num(), LoadedChunks.Num());
}

void ANav3DData::BuildGlobalCompactAdjacency(const TArray<ANav3DDataChunkActor*>& LoadedChunks)
{
	ConsolidatedCompactTacticalData.GlobalRegionAdjacency.Empty();
	
	uint16 GlobalRegionId = 1; // Start from 1 to match region consolidation
	
	for (const ANav3DDataChunkActor* ChunkActor : LoadedChunks)
	{
		if (!ChunkActor || ChunkActor->CompactTacticalData.IsEmpty())
		{
			continue;
		}
		
		const FCompactTacticalData& CompactData = ChunkActor->CompactTacticalData;
		
		// Add intra-volume adjacency
		for (const auto& AdjacencyPair : CompactData.RegionAdjacency)
		{
			const uint8 LocalRegionId = AdjacencyPair.Key;
			const uint64 AdjacencyMask = AdjacencyPair.Value;
			
			// Convert to global region ID
			uint16 GlobalRegionIdForChunk = GlobalRegionId + LocalRegionId;
			
			// Convert local adjacency mask to global adjacency mask
			uint64 GlobalAdjacencyMask = 0;
			for (int32 BitIndex = 0; BitIndex < 64; ++BitIndex)
			{
				if (AdjacencyMask & (1ULL << BitIndex))
				{
					const uint16 AdjacentGlobalId = GlobalRegionId + BitIndex;
					GlobalAdjacencyMask |= (1ULL << (AdjacentGlobalId - 1)); // Adjust for 1-based indexing
				}
			}
			
			if (GlobalAdjacencyMask != 0)
			{
				ConsolidatedCompactTacticalData.GlobalRegionAdjacency.Add(GlobalRegionIdForChunk, GlobalAdjacencyMask);
			}
		}
		
		GlobalRegionId += CompactData.Regions.Num();
	}
	
	UE_LOG(LogNav3D, Log, TEXT("Built global compact adjacency for %d regions"), 
	       ConsolidatedCompactTacticalData.GlobalRegionAdjacency.Num());
}

void ANav3DData::UpdateLoadedRegionIds()
{
	LoadedRegionIds.Empty();
	
	// Collect region IDs from compact tactical data only
	for (ANav3DDataChunkActor* ChunkActor : ChunkActors)
	{
		if (!ChunkActor || !ChunkActor->HasCompactTacticalData())
		{
			continue;
		}
		
		for (int32 i = 0; i < ChunkActor->CompactTacticalData.Regions.Num(); ++i)
		{
			// Create a unique ID combining chunk index and local region index
			const int32 ChunkIndex = ChunkActors.IndexOfByKey(ChunkActor);
			int32 UniqueRegionId = (ChunkIndex * 64) + i; // Each chunk gets 64 region slots
			LoadedRegionIds.Add(UniqueRegionId);
		}
	}
	
	UE_LOG(LogNav3D, Verbose, TEXT("Updated loaded region IDs: %d compact regions from %d chunks"), 
	       LoadedRegionIds.Num(), ChunkActors.Num());
}

bool ANav3DData::FindBestTacticalLocation(
	const FVector& StartPosition,
	const TArray<FVector>& ObserverPositions,
	const ETacticalVisibility Visibility,
	const ETacticalDistance DistancePreference,
	const ETacticalRegion RegionPreference,
	const bool bForceNewRegion,
	const bool bUseRaycasting,
	TArray<FPositionCandidate>& OutCandidatePositions) const
{
	// Ensure tactical reasoning is available on demand when enabled
	if (TacticalSettings.bEnableTacticalReasoning && !TacticalReasoning.IsValid())
	{
		const_cast<ANav3DData*>(this)->InitializeTacticalReasoning();
	}

	if (!TacticalSettings.bEnableTacticalReasoning || !TacticalReasoning.IsValid())
	{
		UE_LOG(LogNav3D, Warning, TEXT("FindBestTacticalLocation: Tactical reasoning not available"));
		return false;
	}
	
	// Ensure compact data is built if empty
	if (ConsolidatedCompactTacticalData.IsEmpty())
	{
		UE_LOG(LogNav3D, Verbose, TEXT("FindBestTacticalLocation: Compact data is empty, rebuilding..."));
		const_cast<ANav3DData*>(this)->RebuildConsolidatedCompactTacticalData();
	}
	
	UE_LOG(LogNav3D, Verbose, TEXT("FindBestTacticalLocation: Using compact tactical data"));
	
	// Debug: Log available regions in compact data
	UE_LOG(LogNav3D, Verbose, TEXT("FindBestTacticalLocation: Compact data has %d regions"), 
		ConsolidatedCompactTacticalData.AllLoadedRegions.Num());
	for (const auto& Pair : ConsolidatedCompactTacticalData.AllLoadedRegions)
	{
		UE_LOG(LogNav3D, Verbose, TEXT("  Region %d at center %s"), 
			Pair.Key, *Pair.Value.Center.ToString());
	}
	
	return TacticalReasoning->FindBestLocationFromCompact(
		ConsolidatedCompactTacticalData,
		StartPosition,
		ObserverPositions,
		Visibility,
		DistancePreference,
		RegionPreference,
		bForceNewRegion,
		bUseRaycasting,
		OutCandidatePositions);
}

void ANav3DData::RebuildConsolidatedTacticalDataFromCompact()
{
#if WITH_EDITOR || !UE_BUILD_SHIPPING
	if (!TacticalSettings.bEnableTacticalReasoning || ConsolidatedCompactTacticalData.IsEmpty())
	{
		return;
	}

	UE_LOG(LogNav3D, Verbose, TEXT("Rebuilding consolidated tactical data from compact format for debug rendering"));

	// Clear existing consolidated data
	ConsolidatedTacticalData.Reset();

	// Get chunks with compact data for the converter
	TArray<ANav3DDataChunkActor*> ChunksWithCompactData;
	for (ANav3DDataChunkActor* Chunk : GetChunkActors())
	{
		if (Chunk && !Chunk->CompactTacticalData.IsEmpty())
		{
			ChunksWithCompactData.Add(Chunk);
		}
	}

	if (ChunksWithCompactData.Num() > 0)
	{
		// Convert compact → build via converter for debug tools
		ConsolidatedTacticalData = FNav3DTacticalDataConverter::CompactToBuild(
			ConsolidatedCompactTacticalData, ChunksWithCompactData);

		UE_LOG(LogNav3D, Verbose, TEXT("Converted %d compact regions to consolidated format"),
			   ConsolidatedTacticalData.AllLoadedRegions.Num());
	}
	else
	{
		UE_LOG(LogNav3D, Warning, TEXT("No chunks with compact data found for conversion"));
	}

	RequestDrawingUpdate();
#endif
}

void ANav3DData::PerformDeferredTacticalRefresh()
{
	if (!bNeedsTacticalRebuild)
	{
		return;
	}

	UE_LOG(LogNav3D, Log, TEXT("PerformDeferredTacticalRefresh: Starting tactical data refresh"));

	// Just mark consolidated data as dirty - don't build it proactively
	InvalidateConsolidatedData();

	bNeedsTacticalRebuild = false;

	// Trigger drawing update - this will call GetConsolidatedTacticalData() on-demand
	RequestDrawingUpdate();

	UE_LOG(LogNav3D, Log, TEXT("PerformDeferredTacticalRefresh: Completed - consolidated data available on-demand"));
}

#if WITH_EDITORONLY_DATA
int32 ANav3DData::GetChunkRevision() const
{
	return ChunkRevision;
}

void ANav3DData::IncrementChunkRevision()
{
	ChunkRevision++;
	NotifyChunksChanged();
}

void ANav3DData::OnTacticalBuildCompleted(const TArray<FBox>& UpdatedVolumes)
{
	// Increment the chunk revision to trigger automatic inspector refresh
	ChunkRevision++;
	UE_LOG(LogNav3D, Log, TEXT("Incremented ChunkRevision to %d after tactical build"), ChunkRevision);

	// Update tactical performance stats if enabled
	if (TacticalSettings.bEnableTacticalReasoning)
	{
		UpdatePerformanceStats();
	}

	// Notify chunks changed to trigger any additional refresh mechanisms
	NotifyChunksChanged();

	UE_LOG(LogNav3D, Log, TEXT("Tactical build completed for %d volumes"), UpdatedVolumes.Num());
}
#endif

void ANav3DData::RebuildTacticalDataForVolume(const TArray<ANav3DDataChunkActor*>& VolumeChunks, const FBox& VolumeBounds)
{
	if (!TacticalSettings.bEnableTacticalReasoning)
	{
		UE_LOG(LogNav3D, Warning, TEXT("RebuildTacticalDataForVolume: Tactical reasoning is disabled"));
		return;
	}

	if (!TacticalReasoning.IsValid())
	{
		if (!InitializeTacticalReasoning())
		{
			UE_LOG(LogNav3D, Error, TEXT("RebuildTacticalDataForVolume: Failed to initialize tactical reasoning"));
			return;
		}
	}

	UE_LOG(LogNav3D, Log, TEXT("Rebuilding tactical data for volume %s with %d chunks"), *VolumeBounds.ToString(), VolumeChunks.Num());
	TacticalReasoning->BuildTacticalDataForVolume(VolumeChunks, VolumeBounds);

	// Notify editor/UI
#if WITH_EDITORONLY_DATA
	OnTacticalBuildCompleted({ VolumeBounds });
#endif

	// Broadcast delegate for listeners
	if (OnTacticalBuildCompletedDelegate.IsBound())
	{
		OnTacticalBuildCompletedDelegate.Broadcast(this, { VolumeBounds });
	}
}
