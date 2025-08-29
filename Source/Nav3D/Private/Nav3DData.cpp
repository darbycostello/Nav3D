#include "Nav3DData.h"
#include "Nav3DDataChunk.h"
#include "Nav3DDataGenerator.h"
#include "Nav3DNavDataRenderingComponent.h"
#include "Nav3DVersion.h"
#include "Pathfinding/Nav3DPathFinder.h"
#include "Pathfinding/Search/Nav3DPathFindingSearch.h"
#include "Pathfinding/Nav3DQueryFilter.h"
#include <AI/NavDataGenerator.h>
#include <DrawDebugHelpers.h>
#include <NavigationSystem.h>
#include "Nav3D.h"
#include "Nav3DUtils.h"
#include "Raycasting/Nav3DRaycaster.h"
#include "Tactical/Nav3DTacticalReasoning.h"

#if WITH_EDITOR
#include <EditorBuildUtils.h>
#include <ObjectEditorUtils.h>
#endif

FNav3DGenerationFinishedDelegate ANav3DData::GenerationFinishedDelegate;
const FNav3DTacticalData ANav3DData::EmptyTacticalData;

FNav3DVolumeDebugData::FNav3DVolumeDebugData() :
	bDebugDrawBounds(false),
	bDebugDrawNodeCoords(false),
	bDebugDrawMortonCodes(false),
	bDebugDrawLayers(false),
	LayerIndexToDraw(0),
	bDebugDrawOccludedVoxels(true),
	bDebugDrawFreeVoxels(false),
	bDebugDrawActivePaths(false)
{
}

ANav3DData::ANav3DData() : MaxSimultaneousBoxGenerationJobsCount(8), TimeSinceLastUpdate(0.0f), Version(ENav3DVersion::Latest)
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
		const auto* NavigationSystemBase = World->GetNavigationSystem();
		if (NavigationSystemBase != nullptr &&
			NavigationSystemBase->IsWorldInitDone())
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

void ANav3DData::Serialize(FArchive& Archive)
{
	Super::Serialize(Archive);

	Archive << Version;
	const bool bIsVersionValid = Version >= ENav3DVersion::MinCompatible && Version <= ENav3DVersion::Latest;

	uint32 N3dSizeBytes = 0;
	const auto N3dSizePosition = Archive.Tell();

	Archive << N3dSizeBytes;

	if (Archive.IsLoading())
	{
		if (!bIsVersionValid)
		{
			UE_LOG(LogNav3D, Warning, TEXT("%s: ANav3DData: Invalid nav mesh version %d. "
				       "Nav mesh needs to be rebuilt.\n"), *GetFullName(), static_cast<int32>(Version));

			// Instead of killing the object, just skip the data
			Archive.Seek(N3dSizePosition + N3dSizeBytes);
			return;
		}

		if (N3dSizeBytes > 4)
		{
			SerializeNav3DData(Archive, Version);
#if !(UE_BUILD_SHIPPING)
			RequestDrawingUpdate();
#endif
		}
		else
		{
			// empty, just skip over this data
			Archive.Seek(N3dSizePosition + N3dSizeBytes);
			VolumeNavigationData.Reset();
		}
	}
	else
	{
		SerializeNav3DData(Archive, Version);

		const int64 CurrentPosition = Archive.Tell();

		N3dSizeBytes = CurrentPosition - N3dSizePosition;

		Archive.Seek(N3dSizePosition);
		Archive << N3dSizeBytes;
		Archive.Seek(CurrentPosition);
	}
}

void ANav3DData::CleanUp()
{
	Super::CleanUp();
	ResetGenerator();
}

bool ANav3DData::NeedsRebuild() const
{
	const auto NeedsRebuild = VolumeNavigationData.FindByPredicate(
		[](const FNav3DVolumeNavigationData& Data)
		{
			return !Data.GetData().IsValid();
		}) != nullptr;

	if (NavDataGenerator.IsValid())
	{
		return NeedsRebuild || NavDataGenerator->GetNumRemaningBuildTasks() > 0;
	}

	const bool Result = Super::NeedsRebuild();
	UE_LOG(LogNav3D, Verbose, TEXT("Nav3DData NeedsRebuild: %s = %d"), *GetName(), Result);

	return NeedsRebuild;
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

	const auto NavigationBoundsNum = VolumeNavigationData.Num();

	if (NavigationBoundsNum == 0)
	{
		return Result;
	}

	TArray<int> NavigationBoundsIndices;
	NavigationBoundsIndices.Reserve(VolumeNavigationData.Num());

	for (auto Index = 0; Index < NavigationBoundsNum; Index++)
	{
		NavigationBoundsIndices.Add(Index);
	}

	// Shuffle the array
	for (int Index = NavigationBoundsIndices.Num() - 1; Index > 0; --Index)
	{
		const auto NewIndex = FMath::RandRange(0, Index);
		Swap(NavigationBoundsIndices[Index], NavigationBoundsIndices[NewIndex]);
	}

	do
	{
		const auto Index = NavigationBoundsIndices.Pop(EAllowShrinking::No);
		const auto& NavigationData = VolumeNavigationData[Index];

		const auto RandomPoint = NavigationData.GetRandomPoint();
		if (RandomPoint.IsSet())
		{
			Result = RandomPoint.GetValue();
			break;
		}
	}
	while (NavigationBoundsIndices.Num() > 0);

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
		if (UNav3DDataChunk* NavigationDataChunk = GetNavigationDataChunk(Level))
		{
			for (const auto& ChunkNavData : NavigationDataChunk->NavigationData)
			{
				if (VolumeNavigationData.FindByPredicate(
					[&ChunkNavData](const FNav3DVolumeNavigationData& NavigationData)
					{
						return ChunkNavData.GetVolumeBounds() ==
							NavigationData.GetVolumeBounds();
					}) == nullptr)
				{
					VolumeNavigationData.Add(ChunkNavData);
				}
			}

			RequestDrawingUpdate();
		}
	}
}

void ANav3DData::OnStreamingLevelRemoved(ULevel* Level, UWorld*)
{
	QUICK_SCOPE_CYCLE_COUNTER(STAT_RecastNavMesh_OnStreamingLevelRemoved);

	if (SupportsStreaming())
	{
		if (UNav3DDataChunk* NavigationDataChunk = GetNavigationDataChunk(Level))
		{
			for (const auto& ChunkNavData : NavigationDataChunk->NavigationData)
			{
				VolumeNavigationData.RemoveAllSwap([&ChunkNavData](
					const auto& NavData)
					{
						return ChunkNavData.GetVolumeBounds() == NavData.GetVolumeBounds();
					});
			}

			RequestDrawingUpdate();
		}
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

#if ENABLE_DRAW_DEBUG

	if (bEnableDrawing && DebugData.bDebugDrawActivePaths)
	{
		for (auto ActivePath : ActivePaths)
		{
			if (!ActivePath.IsValid())
			{
				continue;
			}

			const TSharedPtr<FNavigationPath> ActivePathPtr =
				ActivePath.Pin();
			const auto& PathPoints = ActivePathPtr->GetPathPoints();

			for (auto PathPointIndex = 1; PathPointIndex < PathPoints.Num();
			     ++PathPointIndex)
			{
				const auto& From = PathPoints[PathPointIndex - 1].Location;
				const auto& To = PathPoints[PathPointIndex].Location;

				DrawDebugLine(GetWorld(), From, To, FColor::Red, false, -1, SDPG_World,
				              5.0f);
				DrawDebugCone(GetWorld(), To, From - To, 50.0f, 0.25f, 0.25f, 16,
				              FColor::Red, false, -1, SDPG_World, 5.0f);
			}
		}
	}
#endif
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
				if (VolumeNavigationData.Num() > 0)
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
	for (const auto& NavBoundsData : VolumeNavigationData)
	{
		const auto OctreeDataMemSize = NavBoundsData.GetData().GetAllocatedSize();
		NavigationMemSize += OctreeDataMemSize;
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

	for (const auto& Bounds : VolumeNavigationData)
	{
		BoundingBox += Bounds.GetData().GetNavigationBounds();
	}

	return BoundingBox;
}

void ANav3DData::RemoveDataInBounds(const FBox& Bounds)
{
	VolumeNavigationData.RemoveAllSwap(
		[&Bounds](const FNav3DVolumeNavigationData& Data)
		{
			return Data.GetVolumeBounds() == Bounds;
		});
}

void ANav3DData::AddVolumeNavigationData(FNav3DVolumeNavigationData Data)
{
	VolumeNavigationData.Emplace(MoveTemp(Data));
}

const FNav3DVolumeNavigationData* ANav3DData::GetVolumeNavigationDataContainingPoints(
	const TArray<FVector>& Points) const
{
	return VolumeNavigationData.FindByPredicate(
		[this, &Points](const FNav3DVolumeNavigationData& Data)
		{
			const auto& Bounds = Data.GetData().GetNavigationBounds();
			for (const auto& Point : Points)
			{
				if (!Bounds.IsInside(Point))
				{
					return false;
				}
			}
			return true;
		});
}

void ANav3DData::UpdateNavVersion() { Version = ENav3DVersion::Latest; }

void ANav3DData::SerializeNav3DData(FArchive& Archive, const ENav3DVersion Nav3DVersion)
{
	if (Archive.IsLoading())
	{
		auto VolumeCount = VolumeNavigationData.Num();
		Archive << VolumeCount;
		VolumeNavigationData.Reset(VolumeCount);
		VolumeNavigationData.SetNum(VolumeCount);

		for (auto Index = 0; Index < VolumeCount; Index++)
		{
			VolumeNavigationData[Index].Serialize(Archive, Nav3DVersion);
		}
	}
	else
	{
		// When saving, don't serialize the whole VolumeNavigationData array as it
		// may contain navigation data from chunks added by streaming levels
		TArray<FNav3DVolumeNavigationData> LevelVolumeNavigationData;

		if (SupportsStreaming() &&
			FNavigationSystem::GetCurrent<const UNavigationSystemV1>(GetWorld()) !=
			nullptr)
		{
			const auto& LevelNavigableBounds = GetNavigableBoundsInLevel(GetLevel());

			TArray<bool> NavigationDataIndicesToKeep;
			NavigationDataIndicesToKeep.SetNum(VolumeNavigationData.Num());

			for (const auto& NavigableBounds : LevelNavigableBounds)
			{
				const auto Index = VolumeNavigationData.IndexOfByPredicate(
					[&NavigableBounds](const auto& NavigationData)
					{
						return !NavigationData.IsInNavigationDataChunk()
							&& NavigationData.GetVolumeBounds() == NavigableBounds;
					});

				if (Index != INDEX_NONE)
				{
					NavigationDataIndicesToKeep[Index] = true;
				}
			}

			for (auto Index = VolumeNavigationData.Num() - 1; Index >= 0; --Index)
			{
				if (NavigationDataIndicesToKeep[Index])
				{
					LevelVolumeNavigationData.Add(VolumeNavigationData[Index]);
				}
			}
		}
		else
		{
			LevelVolumeNavigationData = VolumeNavigationData;
		}

		auto VolumeCount = LevelVolumeNavigationData.Num();
		Archive << VolumeCount;

		for (auto Index = 0; Index < VolumeCount; Index++)
		{
			LevelVolumeNavigationData[Index].Serialize(Archive, Nav3DVersion);
		}
	}
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
	VolumeNavigationData.Reset();
	RequestDrawingUpdate();
}

void ANav3DData::BuildNavigationData() const
{
#if WITH_EDITOR
	if (!GIsPlayInEditorWorld)
	{
		// Use the editor build utils in editor - this properly handles async building
		FEditorBuildUtils::EditorBuild(GetWorld(), FBuildOptions::BuildAIPaths);
		return;
	}
#endif

	// For runtime (and PIE) use navigation system directly
	if (UWorld* World = GetWorld())
	{
		if (UNavigationSystemV1* NavSys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(World))
		{
			NavSys->CancelBuild();
			NavSys->Build();
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
						const auto& LevelNavBounds = GetNavigableBoundsInLevel(Level);

						TArray<int32> NavigationDataIndices;
						NavigationDataIndices.Reserve(LevelNavBounds.Num());

						for (const auto& NavBounds : LevelNavBounds)
						{
							if (const auto Index = VolumeNavigationData.IndexOfByPredicate(
									[&NavBounds](const FNav3DVolumeNavigationData& Data)
									{
										const auto& Bounds = Data.GetData().GetVolumeBounds();
										return Bounds == NavBounds;
									});
								Index != INDEX_NONE)
							{
								NavigationDataIndices.Add(Index);
							}
						}

						if (NavigationDataIndices.Num() > 0)
						{
							if (NavigationDataChunk == nullptr)
							{
								NavigationDataChunk = NewObject<UNav3DDataChunk>(Level);
								NavigationDataChunk->NavigationDataName = GetFName();
								Level->NavDataChunks.Add(NavigationDataChunk);
							}

							for (const auto Index : NavigationDataIndices)
							{
								NavigationDataChunk->AddNavigationData(
									VolumeNavigationData[Index]);
							}

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

			RequestDrawingUpdate(true);
#endif // WITH_EDITOR

			// Build tactical data if enabled
			if (TacticalSettings.bEnableTacticalReasoning)
			{
				if (VolumeNavigationData.Num() > 0)
				{
					UE_LOG(LogNav3D, Verbose, TEXT("Building tactical data after navigation generation"));
					BuildTacticalData();
				}
				else
				{
					UE_LOG(LogNav3D, Warning, TEXT("Cannot rebuild tactical data, no valid navigation data"));
				}
			}

			if (UNavigationSystemV1* NavSys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(World))
			{
				NavSys->OnNavigationGenerationFinished(*this);
			}

			GenerationFinishedDelegate.Broadcast(this);
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
	       *Occluder->GetActorNameOrLabel(), VolumeNavigationData.Num(), *OccluderBounds.ToString());

	bool bAnyIntersection = false;
	for (auto& VolumeNavData : VolumeNavigationData)
	{
		const FBox& NavBounds = VolumeNavData.GetVolumeBounds();

		if (NavBounds.Intersect(OccluderBounds))
		{
			bAnyIntersection = true;
			UE_LOG(LogNav3D, Verbose, TEXT("Registering occluder %s with volume at %s"),
			       *Occluder->GetActorNameOrLabel(), *NavBounds.ToString());
			VolumeNavData.AddDynamicOccluder(Occluder);
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
	for (auto& VolumeNavData : VolumeNavigationData)
	{
		VolumeNavData.RemoveDynamicOccluder(Occluder);
	}
}

void ANav3DData::RebuildDirtyBounds(const TArray<FBox>& DirtyBounds)
{
	UE_LOG(LogNav3D, Verbose, TEXT("Nav3DData: Processing %d dirty bounds"), DirtyBounds.Num());

	for (auto& VolumeNavData : VolumeNavigationData)
	{
		const FBox& VolumeBounds = VolumeNavData.GetVolumeBounds();

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
			VolumeNavData.RebuildDirtyBounds(DirtyBounds);
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
	if (TacticalSettings.bEnableTacticalReasoning)
	{
		if (InitializeTacticalReasoning())
		{
			for (const FNav3DVolumeNavigationData& Volume : VolumeNavigationData)
			{
				TacticalReasoning->BuildTacticalData(Volume.GetVolumeBounds());
			}
		}
	}
}

const FNav3DVolumeNavigationData* ANav3DData::GetVolumeNavigationDataContainingPoint(const FVector& Point) const
{
	for (const FNav3DVolumeNavigationData& Volume : VolumeNavigationData)
	{
		if (Volume.GetVolumeBounds().IsInside(Point))
		{
			return &Volume;
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
	bool bForceNewRegion,
	bool bUseRaycasting) const
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
	return VolumeNavigationData.GetData()->GetLayerCount();
}

const FNav3DTacticalData& ANav3DData::GetTacticalDataAtPosition(const FVector& Position) const
{
	// Try to find a volume containing this position
	if (const FNav3DVolumeNavigationData* VolumeData = GetVolumeNavigationDataContainingPoint(Position))
	{
		return VolumeData->TacticalData;
	}
    
	// If no volume contains this position, fall back to the first volume if any exist
	if (VolumeNavigationData.Num() > 0)
	{
		return VolumeNavigationData[0].TacticalData;
	}
    
	// Empty tactical data as last resort
	return EmptyTacticalData;
}
