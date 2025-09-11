#include "Nav3DData.h"
#include "Nav3DDataChunk.h"
#include "Nav3DDataGenerator.h"
#include "Nav3DNavDataRenderingComponent.h"
#include "Nav3DDataChunkActor.h"
#include "Nav3DWorldSubsystem.h"
#include "Pathfinding/Nav3DPathFinder.h"
#include "Pathfinding/Search/Nav3DPathFindingSearch.h"
#include "Pathfinding/Nav3DQueryFilter.h"
#include <AI/NavDataGenerator.h>
#include <DrawDebugHelpers.h>
#include <NavigationSystem.h>
#include <HAL/CriticalSection.h>
#include "EngineUtils.h"
#include "Nav3D.h"
#include "Nav3DUtils.h"
#include "Raycasting/Nav3DRaycaster.h"
#include "Tactical/Nav3DTacticalReasoning.h"
#include "Components/PrimitiveComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Components/SphereComponent.h"
#include "Components/BoxComponent.h"
#include "Components/CapsuleComponent.h"
#include "LandscapeMeshCollisionComponent.h"
#include "LandscapeHeightfieldCollisionComponent.h"
#include "Nav3DBoundsVolume.h"
#include "Nav3DTacticalActor.h"
#include "PhysicsEngine/BodySetup.h"
#include "Engine/StaticMesh.h"
#include "Engine/OverlapResult.h"
#include "Internationalization/Text.h"
#include "Internationalization/Internationalization.h"

#if WITH_EDITOR
#include <ObjectEditorUtils.h>
#endif

FNav3DGenerationFinishedDelegate ANav3DData::GenerationFinishedDelegate;
const FNav3DTacticalData ANav3DData::EmptyTacticalData;

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
	
	// Check tactical actor integrity and clean up invalid actors
	const int32 InvalidTacticalCount = GetInvalidTacticalActorCount();
	if (InvalidTacticalCount > 0)
	{
		UE_LOG(LogNav3D, Warning, TEXT("Found %d invalid tactical actors, cleaning up"), InvalidTacticalCount);
		CleanupInvalidTacticalActors();
	}
	
	// Count valid tactical actors after cleanup
	int32 ValidTacticalActors = 0;
	for (const ANav3DTacticalActor* TacticalActor : TacticalActors)
	{
		if (TacticalActor && IsValid(TacticalActor))
		{
			ValidTacticalActors++;
		}
	}
	UE_LOG(LogNav3D, Display, TEXT("Tactical Actors - Valid: %d, Total: %d"), 
	       ValidTacticalActors, TacticalActors.Num());
	
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
static FString FormatNumber(int32 Number)
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

	if (const auto* World = GetWorld())
	{
		if (const auto* NavigationSystemBase = World->GetNavigationSystem();
			NavigationSystemBase != nullptr && NavigationSystemBase->IsWorldInitDone())
		{
			CheckToDiscardSubLevelNavData(*NavigationSystemBase);
		}
		else
		{
			UNavigationSystemBase::OnNavigationInitStartStaticDelegate().AddUObject(
				this, &ThisClass::CheckToDiscardSubLevelNavData);
		}
	}

	RecreateDefaultFilter();
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

ENavigationQueryResult::Type ANav3DData::CalcPathLengthAndCost(
	const FVector& PathStart, const FVector& PathEnd,
	FVector::FReal& OutPathLength, FVector::FReal& OutPathCost,
	const FSharedConstNavQueryFilter Filter, const UObject* Querier) const
{
	if ((PathStart - PathEnd).IsNearlyZero())
	{
		OutPathLength = 0.f;
		return ENavigationQueryResult::Success;
	}

	auto* VolumeNavData = GetVolumeNavigationDataContainingPoints({PathStart, PathEnd});

	if (VolumeNavData == nullptr)
	{
		return ENavigationQueryResult::Error;
	}

	const TSharedRef<FNav3DPath> NavigationPath = MakeShareable(new FNav3DPath());
	const auto NavAgentProps = FNav3DUtils::GetNavAgentPropsFromQuerier(Querier);

	const ENavigationQueryResult::Type Result = FNav3DPathFinder::GetPath(
		NavigationPath.Get(), *this, PathStart, PathEnd, NavAgentProps, Filter);

	if (Result == ENavigationQueryResult::Success ||
		(Result == ENavigationQueryResult::Fail && NavigationPath->IsPartial()))
	{
		OutPathLength = NavigationPath->GetLength();
		OutPathCost = NavigationPath->GetCost();
	}

	return Result;
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
				UE_LOG(LogNav3D, Display, TEXT("Tactical rebuild requested"));
				
				// Rebuild tactical data if we have valid navigation data
				if (ChunkActors.Num() > 0)
				{
					BuildTacticalData();
				} else
				{
					UE_LOG(LogNav3D, Warning, TEXT("Cannot rebuild, no valid navigation data"));
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



const FNav3DVolumeNavigationData* ANav3DData::GetVolumeNavigationDataContainingPoints(
	const TArray<FVector>& Points) const
{
	// Find a chunk actor that contains all points using GetAllChunkActors() to avoid null entries
	for (ANav3DDataChunkActor* ChunkActor : GetAllChunkActors())
	{
		if (!ChunkActor) continue;
		
		bool bContainsAllPoints = true;
		for (const FVector& Point : Points)
		{
			if (!ChunkActor->ContainsPoint(Point))
			{
				bContainsAllPoints = false;
				break;
			}
		}
		
		if (bContainsAllPoints && ChunkActor->Nav3DChunks.Num() > 0)
		{
			return ChunkActor->Nav3DChunks[0]->GetVolumeNavigationData();
		}
	}
	
	return nullptr;
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

void ANav3DData::AnalyzeSpatialClustering(const TArray<FVector>& ObjectPositions, const TArray<FBox>& ObjectBounds, const FBox& VolumeBounds, int32 NumCandidateObjects)
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


void ANav3DData::EstimateOctreeSize(const FBox& VolumeBounds, float EmptyGridRatio, int32 MaxLayers, float LeafNodeSize)
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
		// Clean up invalid tactical actors before rebuilding
		const int32 InvalidTacticalCount = const_cast<ANav3DData*>(this)->GetInvalidTacticalActorCount();
		if (InvalidTacticalCount > 0)
		{
			UE_LOG(LogNav3D, Log, TEXT("BuildNavigationData: Cleaning up %d invalid tactical actors before rebuild"), InvalidTacticalCount);
			const_cast<ANav3DData*>(this)->CleanupInvalidTacticalActors();
		}
		
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
	// Clean up invalid tactical actors before building
	const int32 InvalidTacticalCount = GetInvalidTacticalActorCount();
	if (InvalidTacticalCount > 0)
	{
		UE_LOG(LogNav3D, Log, TEXT("BuildSingleVolume: Cleaning up %d invalid tactical actors before rebuild"), InvalidTacticalCount);
		CleanupInvalidTacticalActors();
	}
	
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
	
	// Clean up invalid tactical actors before rebuilding
	const int32 InvalidTacticalCount = GetInvalidTacticalActorCount();
	if (InvalidTacticalCount > 0)
	{
		UE_LOG(LogNav3D, Log, TEXT("RebuildSingleChunk: Cleaning up %d invalid tactical actors before rebuild"), InvalidTacticalCount);
		CleanupInvalidTacticalActors();
	}
	
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
	
	UE_LOG(LogNav3D, Log, TEXT("Rebuilding tactical data for %d chunk actors"), ChunkActors.Num());
	
	// Destroy existing tactical actors
	TArray<ANav3DTacticalActor*> TacticalActorsToDestroy;
	TacticalActorsToDestroy.Reserve(TacticalActors.Num());
	for (ANav3DTacticalActor* TacticalActor : TacticalActors)
	{
		if (TacticalActor)
		{
			TacticalActorsToDestroy.Add(TacticalActor);
		}
	}
	
	for (ANav3DTacticalActor* ActorToDestroy : TacticalActorsToDestroy)
	{
		UE_LOG(LogNav3D, Log, TEXT("Destroying tactical actor before rebuild: %s"), *ActorToDestroy->GetName());
		UnregisterTacticalActor(ActorToDestroy);
		GetWorld()->DestroyActor(ActorToDestroy);
	}
	
	// Clear the tactical actors array
	TacticalActors.Empty();
	
	// Reset tactical generation flag to allow rebuild
	if (FNavDataGenerator* BaseGenerator = GetGenerator())
	{
		if (FNav3DDataGenerator* Generator = static_cast<FNav3DDataGenerator*>(BaseGenerator))
		{
			Generator->ResetTacticalGenerationFlag();
			Generator->StartTacticalGeneration();
		}
	}
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

void ANav3DData::OnNavigationDataGenerationFinished()
{
	if (UWorld* World = GetWorld())
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
	
	// Ensure cross-volume graphs are (re)built after nav generation regardless of tactical reasoning
	BuildTacticalData();
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
			Result.Result = FNav3DPathFinder::GetPath(
				*N3dNavigationPath,
				*Self,
				PathFindingQuery.StartLocation,
				PathFindingQuery.EndLocation,
				NavAgentProperties,
				PathFindingQuery.QueryFilter);
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
		TacticalReasoning->Initialize(this);
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
	// Ensure one tactical actor per discoverable volume, regardless of tactical reasoning flag
	{
		const TArray<FBox> Volumes = GetAllDiscoverableVolumes();
		for (const FBox& VolumeBounds : Volumes)
		{
			bool bHasTA = false;
			for (const ANav3DTacticalActor* TA : TacticalActors)
			{
				if (TA && TA->OwningVolumeBounds.Equals(VolumeBounds))
				{
					bHasTA = true;
					break;
				}
			}
			if (!bHasTA)
			{
				if (UWorld* World = GetWorld())
				{
					FActorSpawnParameters SpawnParams; SpawnParams.Owner = this; SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
					if (ANav3DTacticalActor* TA = World->SpawnActor<ANav3DTacticalActor>(SpawnParams))
					{
						TA->SetTacticalActorBounds(VolumeBounds);
						TA->OwningVolumeBounds = VolumeBounds;
						RegisterTacticalActor(TA);
					}
				}
			}
		}
	}
	
	// Always (re)build cross-volume graphs for all tactical actors first
	for (ANav3DTacticalActor* TacticalActor : TacticalActors)
	{
		if (!TacticalActor)
		{
			continue;
		}
		TArray<ANav3DDataChunkActor*> RelevantChunks;
		if (UNav3DWorldSubsystem* Subsystem = GetSubsystem())
		{
			Subsystem->QueryActorsInBounds(TacticalActor->TacticalActorBounds, RelevantChunks);
		}
		else
		{
			RelevantChunks = GetAllChunkActors();
		}
		TacticalActor->BuildCrossVolumeGraph(RelevantChunks);
	}
	
	// Then build tactical reasoning data if enabled
	if (TacticalSettings.bEnableTacticalReasoning)
	{
		if (InitializeTacticalReasoning())
		{
			TacticalReasoning->BuildGlobalTacticalData(ChunkActors);
		}
	}
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
	if (!TacticalSettings.bEnableTacticalReasoning || !TacticalReasoning.IsValid())
	{
		UE_LOG(LogNav3D, Warning, TEXT("FindBestLocation: Tactical reasoning not enabled or not initialized"));
		return false;
	}

	// Call the implementation with the found tactical data
	return TacticalReasoning->FindBestLocation(
		GetTacticalDataAtPosition(StartPosition),
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

const FNav3DTacticalData& ANav3DData::GetTacticalDataAtPosition(const FVector& Position) const
{
	// First, try to find a tactical actor containing this position
	for (const ANav3DTacticalActor* TacticalActor : TacticalActors)
	{
		if (TacticalActor && TacticalActor->ContainsPoint(Position))
		{
			return TacticalActor->TacticalData;
		}
	}
	
	// Fallback: Try to find a volume containing this position (for backward compatibility)
	if (const FNav3DVolumeNavigationData* VolumeData = GetVolumeNavigationDataContainingPoint(Position))
	{
		return VolumeData->TacticalData;
	}
    
	// If no volume contains this position, fall back to the first volume if any exist
	if (ChunkActors.Num() > 0)
	{
		for (const ANav3DDataChunkActor* ChunkActor : ChunkActors)
		{
			if (!ChunkActor) continue;
			
			for (const UNav3DDataChunk* Chunk : ChunkActor->Nav3DChunks)
			{
				if (!Chunk) continue;

				if (const FNav3DVolumeNavigationData* VolumeData = Chunk->GetVolumeNavigationData())
				{
					return VolumeData->TacticalData;
				}
			}
		}
	}
    
	// Empty tactical data as last resort
	return EmptyTacticalData;
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
		// Purge and rebuild adjacency around the removed chunk
		TArray<ANav3DDataChunkActor*> Remaining = GetAllChunkActors();
		float VoxelSize = 0.0f;
		for (ANav3DDataChunkActor* Other : Remaining)
		{
			if (!Other) continue;
			// Remove portal links pointing into removed bounds and rebuild lookup
			Other->PortalLookup.Reset();
			for (FNav3DChunkAdjacency& Adj : Other->ChunkAdjacency)
			{
				Adj.Connections.RemoveAllSwap([&](const FNav3DVoxelConnection& Conn)
				{
					const FVector LocalPos = FVector(FNav3DUtils::GetVectorFromMortonCode(Conn.Local));
					const FVector WorldPos = Other->GetActorTransform().TransformPosition(LocalPos);
					return RemovedBounds.IsInside(WorldPos);
				}, EAllowShrinking::No);
				for (const FNav3DVoxelConnection& Conn : Adj.Connections)
				{
					Other->PortalLookup.FindOrAdd(Conn.LocalVolumeIndex).Add(Conn.Local, Conn);
				}
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
				Actor->PortalLookup.Reset();
				// Adjacency will be rebuilt when needed
			}
		}
	}
}

int32 ANav3DData::GetInvalidChunkActorCount() const
{
	int32 InvalidCount = 0;
	for (ANav3DDataChunkActor* ChunkActor : ChunkActors)
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

// ============================================================================
// Tactical Actor Management
// ============================================================================

void ANav3DData::RegisterTacticalActor(ANav3DTacticalActor* TacticalActor)
{
	if (!TacticalActor)
	{
		UE_LOG(LogNav3D, Warning, TEXT("Cannot register tactical actor: invalid"));
		return;
	}
	
	// Check if already registered
	if (TacticalActors.Contains(TacticalActor))
	{
		UE_LOG(LogNav3D, Verbose, TEXT("Tactical actor already registered: %s"), *TacticalActor->GetName());
		return;
	}
	
	// Enforce uniqueness per owning volume
	if (TacticalActor->OwningVolumeBounds.IsValid)
	{
		const int32 Removed = TacticalActors.RemoveAllSwap([&](const ANav3DTacticalActor* TA)
		{
			return TA && TA->OwningVolumeBounds.Equals(TacticalActor->OwningVolumeBounds);
		});
		if (Removed > 0)
		{
			UE_LOG(LogNav3D, Log, TEXT("RegisterTacticalActor: Removed %d existing tactical actors for volume %s"), Removed, *TacticalActor->OwningVolumeBounds.ToString());
		}
	}
	
	TacticalActors.Add(TacticalActor);
	
	// Build cross-volume graph from chunk actors intersecting this tactical actor's bounds
	TArray<ANav3DDataChunkActor*> RelevantChunks;
	if (UNav3DWorldSubsystem* Subsystem = GetSubsystem())
	{
		Subsystem->QueryActorsInBounds(TacticalActor->TacticalActorBounds, RelevantChunks);
	}
	else
	{
		// Fallback: use all known chunk actors
		RelevantChunks = GetAllChunkActors();
	}
	TacticalActor->BuildCrossVolumeGraph(RelevantChunks);
	
	UE_LOG(LogNav3D, Log, TEXT("Registered tactical actor: %s with bounds %s (CV connections: %d)"), 
	       *TacticalActor->GetName(), *TacticalActor->TacticalActorBounds.ToString(), TacticalActor->GetCrossVolumeGraph().GetConnectionCount());
}

void ANav3DData::UnregisterTacticalActor(ANav3DTacticalActor* TacticalActor)
{
	if (!TacticalActor)
	{
		return;
	}
	
	const int32 RemovedCount = TacticalActors.RemoveAllSwap([TacticalActor](const ANav3DTacticalActor* Actor)
	{
		return Actor == TacticalActor;
	});
	
	if (RemovedCount > 0)
	{
		UE_LOG(LogNav3D, Log, TEXT("Unregistered tactical actor: %s"), *TacticalActor->GetName());
	}
}

void ANav3DData::ClearAllTacticalActors()
{
	if (TacticalActors.Num() == 0)
	{
		return;
	}
	
	UE_LOG(LogNav3D, Log, TEXT("Clearing all %d tactical actors before navigation rebuild"), TacticalActors.Num());
	
	// Destroy all tactical actors
	for (ANav3DTacticalActor* TacticalActor : TacticalActors)
	{
		if (TacticalActor)
		{
			UE_LOG(LogNav3D, Log, TEXT("Destroying tactical actor: %s"), *TacticalActor->GetName());
			GetWorld()->DestroyActor(TacticalActor);
		}
	}
	
	// Clear the tactical actors array
	TacticalActors.Empty();
	
	UE_LOG(LogNav3D, Log, TEXT("All tactical actors cleared"));
}

TArray<ANav3DTacticalActor*> ANav3DData::GetAllTacticalActors() const
{
	TArray<ANav3DTacticalActor*> ValidActors;
	ValidActors.Reserve(TacticalActors.Num());
	
	int32 InvalidCount = 0;
	for (ANav3DTacticalActor* TacticalActor : TacticalActors)
	{
		if (TacticalActor && IsValid(TacticalActor))
		{
			ValidActors.Add(TacticalActor);
		}
		else
		{
			InvalidCount++;
		}
	}
	
	// Log if we found invalid actors (but don't spam the log)
	if (InvalidCount > 0)
	{
		UE_LOG(LogNav3D, Warning, TEXT("GetAllTacticalActors: Found %d invalid tactical actors out of %d total"), 
		       InvalidCount, TacticalActors.Num());
	}
	
	return ValidActors;
}

void ANav3DData::CleanupInvalidTacticalActors()
{
	const int32 OriginalCount = TacticalActors.Num();
	if (OriginalCount == 0)
	{
		return;
	}
	
	// Remove invalid actors from the array
	const int32 RemovedCount = TacticalActors.RemoveAllSwap([](const ANav3DTacticalActor* TacticalActor)
	{
		return !TacticalActor || !IsValid(TacticalActor);
	});
	
	if (RemovedCount > 0)
	{
		UE_LOG(LogNav3D, Log, TEXT("CleanupInvalidTacticalActors: Removed %d invalid tactical actors (was %d, now %d)"), 
		       RemovedCount, OriginalCount, TacticalActors.Num());
	}
}

int32 ANav3DData::GetInvalidTacticalActorCount() const
{
	int32 InvalidCount = 0;
	for (ANav3DTacticalActor* TacticalActor : TacticalActors)
	{
		if (!TacticalActor || !IsValid(TacticalActor))
		{
			InvalidCount++;
		}
	}
	return InvalidCount;
}

void ANav3DData::CleanupInvalidTacticalActorsBP()
{
	const int32 InvalidCount = GetInvalidTacticalActorCount();
	if (InvalidCount > 0)
	{
		UE_LOG(LogNav3D, Log, TEXT("CleanupInvalidTacticalActorsBP: Cleaning up %d invalid tactical actors"), InvalidCount);
		CleanupInvalidTacticalActors();
	}
	else
	{
		UE_LOG(LogNav3D, Log, TEXT("CleanupInvalidTacticalActorsBP: No invalid tactical actors found"));
	}
}

void ANav3DData::CleanupAllInvalidActors()
{
	const int32 InvalidChunkCount = GetInvalidChunkActorCount();
	const int32 InvalidTacticalCount = GetInvalidTacticalActorCount();
	
	if (InvalidChunkCount > 0 || InvalidTacticalCount > 0)
	{
		UE_LOG(LogNav3D, Log, TEXT("CleanupAllInvalidActors: Cleaning up %d invalid chunk actors and %d invalid tactical actors"), 
		       InvalidChunkCount, InvalidTacticalCount);
		
		if (InvalidChunkCount > 0)
		{
			CleanupInvalidChunkActors();
		}
		
		if (InvalidTacticalCount > 0)
		{
			CleanupInvalidTacticalActors();
		}
		
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
	
	return AllVolumes;
}