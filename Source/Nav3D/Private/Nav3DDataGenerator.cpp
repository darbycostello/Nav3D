#include "Nav3DDataGenerator.h"
#include "Nav3DData.h"
#include "Nav3DVolumeNavigationData.h"
#include "Nav3DSettings.h"
#include "Nav3DDataChunkActor.h"
#include "Nav3DUtils.h"
#include <GameFramework/PlayerController.h>
#include <NavigationSystem.h>
#include "EngineUtils.h"
#include "Nav3D.h"
#include "Engine/World.h"
#include "HAL/PlatformTime.h"
#include "Nav3DBoundsVolume.h"
#include "Tactical/Nav3DTacticalReasoning.h"

FNav3DVolumeNavigationDataGenerator::FNav3DVolumeNavigationDataGenerator(
	FNav3DDataGenerator& NavigationDataGenerator, const FBox& VolumeBounds)
	: ParentGenerator(NavigationDataGenerator), BoundsNavigationData(),
	  VolumeBounds(VolumeBounds)
{
	NavDataConfig = NavigationDataGenerator.GetOwner()->GetConfig();
}

bool FNav3DVolumeNavigationDataGenerator::DoWork()
{
	UE_LOG(LogNav3D, Log, TEXT("[Vol %s] Starting Nav3D volume generation"), *VolumeBounds.ToString());

	FNav3DVolumeNavigationDataSettings GenerationSettings;
	GenerationSettings.GenerationSettings = ParentGenerator.GetGenerationSettings();
	GenerationSettings.World = ParentGenerator.GetWorld();
	GenerationSettings.VoxelExtent = NavDataConfig.AgentRadius * 2.0f;
	GenerationSettings.TacticalSettings = ParentGenerator.GetOwner()->TacticalSettings;
	// Provide debug label/index for per-volume logging
	GenerationSettings.DebugLabel = TEXT("");
	GenerationSettings.DebugVolumeIndex = ParentGenerator.GetNumRunningBuildTasks() +
		ParentGenerator.GetPendingBoundsDataGenerationElements().Num();
	BoundsNavigationData.GenerateNavigationData(VolumeBounds, GenerationSettings);

	UE_LOG(LogNav3D, Log, TEXT("[Vol %s] Completed Nav3D volume generation"), *VolumeBounds.ToString());
	
	return true;
}


FNav3DDataGenerator::FNav3DDataGenerator(ANav3DData& NavigationData)
	: NavigationData(NavigationData), MaximumGeneratorTaskCount(2),
	  IsInitialized(false)
{
}

void FNav3DDataGenerator::Init()
{
	GenerationSettings = NavigationData.GenerationSettings;

	UpdateNavigationBounds();

	const int32 WorkerThreadsCount =
		FTaskGraphInterface::Get().GetNumWorkerThreads();
	MaximumGeneratorTaskCount =
		FMath::Min(FMath::Max(WorkerThreadsCount * 2, 1),
		           NavigationData.GenerationSettings.MaxSimultaneousBoxGenerationJobsCount);
	UE_LOG(LogNav3D, Verbose,
	       TEXT("Using max of %d workers to build Nav3D navigation."),
	       MaximumGeneratorTaskCount);

	// Clear any previous global cancel before starting a new build session
	FNav3DVolumeNavigationData::ClearCancelBuildAll();
}

bool FNav3DDataGenerator::RebuildAll()
{
	// Clean up invalid chunk actors before rebuilding
	const int32 InvalidCount = NavigationData.GetInvalidChunkActorCount();
	if (InvalidCount > 0)
	{
		UE_LOG(LogNav3D, Log, TEXT("RebuildAll: Cleaning up %d invalid chunk actors before rebuild"), InvalidCount);
		NavigationData.CleanupInvalidChunkActors();
	}
	
	UpdateNavigationBounds();

	TArray<FNavigationDirtyArea> DirtyAreas;
	DirtyAreas.Reserve(RegisteredNavigationBounds.Num());

	for (const auto& RegisteredBounds : RegisteredNavigationBounds)
	{
		DirtyAreas.Emplace(FNavigationDirtyArea(RegisteredBounds, ENavigationDirtyFlag::None));
	}

	RebuildDirtyAreas(DirtyAreas);

	NavigationData.RequestDrawingUpdate();
	return true;
}

void FNav3DDataGenerator::EnsureBuildCompletion()
{
	if (GetNumRemaningBuildTasks() > 0)
	{
		StartChunkedBuildCompletion();
		return;
	}
	NavigationData.RequestDrawingUpdate();
}

void FNav3DDataGenerator::StartChunkedBuildCompletion()
{
	if (const UWorld* World = GetWorld())
	{
		FTimerDelegate Delegate;
		Delegate.BindRaw(this, &FNav3DDataGenerator::ProcessBuildChunk);
		World->GetTimerManager().SetTimer(ChunkedBuildTimerHandle, Delegate, 0.1f, true);
	}
}

void FNav3DDataGenerator::ProcessBuildChunk()
{
	static constexpr float MaxChunkTimeSeconds = 0.05f;
	const double StartTime = FPlatformTime::Seconds();

	const int32 TasksToProcessCount =
		(FNav3DVolumeNavigationData::IsCancelRequested() 
			? 0 
			: (MaximumGeneratorTaskCount - RunningBoundsDataGenerationElements.Num()));
	ProcessAsyncTasks(TasksToProcessCount);

	bool bHasTimeRemaining = true;
	int32 ProcessCounter = 0;
	while (bHasTimeRemaining && GetNumRemaningBuildTasks() > 0)
	{
		ProcessAsyncTasks(1);

		const double CurrentTime = FPlatformTime::Seconds();
		bHasTimeRemaining = (CurrentTime - StartTime) < MaxChunkTimeSeconds;

		if (++ProcessCounter % 10 == 0)
		{
			break;
		}
	}

	if (GetNumRemaningBuildTasks() == 0)
	{
		if (const UWorld* World = GetWorld())
		{
			World->GetTimerManager().ClearTimer(ChunkedBuildTimerHandle);
		}

		// Build tactical data after all async tasks are complete
		// This is a separate step that happens after the navigation build is fully complete
		if (NavigationData.TacticalSettings.bEnableTacticalReasoning)
		{
			UE_LOG(LogNav3D, Log, TEXT("************************************************************"));
			UE_LOG(LogNav3D, Log, TEXT("All async navigation tasks complete - building tactical data"));
			UE_LOG(LogNav3D, Log, TEXT("************************************************************"));
			
			// Initialize tactical reasoning once at the beginning
			if (!NavigationData.TacticalReasoning.IsValid())
			{
				UE_LOG(LogNav3D, Log, TEXT("Initializing tactical reasoning for build"));
				if (!NavigationData.InitializeTacticalReasoning())
				{
					UE_LOG(LogNav3D, Error, TEXT("Failed to initialize tactical reasoning"));
					// Continue without tactical data rather than crashing
				}
				else
				{
					UE_LOG(LogNav3D, Log, TEXT("Tactical reasoning initialized successfully"));
				}
			}
			else
			{
				UE_LOG(LogNav3D, Verbose, TEXT("Tactical reasoning already initialized"));
			}
			
			// Group chunk actors by Nav3DBoundsVolume using the same method as the inspector
			TMap<FBox, TArray<ANav3DDataChunkActor*>> VolumeToChunks;
			const TArray<ANav3DDataChunkActor*> AllChunkActors = NavigationData.GetAllChunkActors();

			// Get discoverable volumes the same way the inspector does
			const TArray<FBox> Volumes = NavigationData.GetAllDiscoverableVolumes();

			for (ANav3DDataChunkActor* ChunkActor : AllChunkActors)
			{
				if (!ChunkActor) continue;
    
				const FVector ChunkCenter = ChunkActor->DataChunkActorBounds.GetCenter();
    
				// Find which volume contains this chunk using the same logic as inspector
				bool bFoundVolume = false;
				for (const FBox& VolumeBounds : Volumes)
				{
					if (VolumeBounds.IsInside(ChunkCenter))
					{
						VolumeToChunks.FindOrAdd(VolumeBounds).Add(ChunkActor);
						UE_LOG(LogNav3D, Verbose, TEXT("Grouped chunk %s under volume %s"), 
							   *ChunkActor->GetName(), *VolumeBounds.ToString());
						bFoundVolume = true;
						break;
					}
				}
    
				if (!bFoundVolume)
				{
					UE_LOG(LogNav3D, Warning, TEXT("Could not find containing volume for chunk %s at %s"), 
						   *ChunkActor->GetName(), *ChunkCenter.ToString());
				}
			}

			int32 VolumesBuilt = 0;
			TArray<FBox> BuiltVolumes;
			for (const auto& Pair : VolumeToChunks)
			{
				const FBox& VolumeBounds = Pair.Key;
				const TArray<ANav3DDataChunkActor*>& VolumeChunks = Pair.Value;
    
				if (NavigationData.TacticalReasoning.IsValid())
				{
					UE_LOG(LogNav3D, Log, TEXT("Building tactical data for volume [%s] with %d chunks"), 
						   *VolumeBounds.ToString(), VolumeChunks.Num());
        
					NavigationData.TacticalReasoning->BuildTacticalDataForVolume(VolumeChunks, VolumeBounds);
					VolumesBuilt++;
					BuiltVolumes.Add(VolumeBounds);
				}
			}
			UE_LOG(LogNav3D, Log, TEXT("Built tactical data for %d volumes"), VolumesBuilt);

			// Notify editor/UI and any listeners
#if WITH_EDITORONLY_DATA
			NavigationData.OnTacticalBuildCompleted(BuiltVolumes);
#endif
			if (NavigationData.OnTacticalBuildCompletedDelegate.IsBound())
			{
				NavigationData.OnTacticalBuildCompletedDelegate.Broadcast(&NavigationData, BuiltVolumes);
			}
		}
	}
}

void FNav3DDataGenerator::CancelBuild()
{
	// Do not clear the pump timer here; let it drain so completion is reached and popup can hide
	PendingBoundsDataGenerationElements.Empty();

	// Signal cooperative cancel so workers bail quickly
	FNav3DVolumeNavigationData::RequestCancelBuildAll();
}

void FNav3DDataGenerator::TickAsyncBuild(float DeltaSeconds)
{
	const UNavigationSystemV1* NavigationSystem =
		FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld());
	if (!ensureMsgf(NavigationSystem != nullptr,
	                TEXT("FRecastNavMeshGenerator can't found valid navigation "
		                "system: Owner=[%s] World=[%s]"),
	                *GetFullNameSafe(GetOwner()), *GetFullNameSafe(GetWorld())))
	{
		return;
	}

	const int32 RunningTasksCount = NavigationSystem->GetNumRunningBuildTasks();

	const int32 TasksToSubmitCount =
		FNav3DVolumeNavigationData::IsCancelRequested() ? 0 : (MaximumGeneratorTaskCount - RunningTasksCount);

	const auto FinishedBoxes = ProcessAsyncTasks(TasksToSubmitCount);

	if (FinishedBoxes.Num() > 0)
	{
		NavigationData.OnNavigationDataUpdatedInBounds(FinishedBoxes);
		NavigationData.RequestDrawingUpdate();
	}
}

void FNav3DDataGenerator::OnNavigationBoundsChanged()
{
	UpdateNavigationBounds();
}

void FNav3DDataGenerator::RebuildDirtyAreas(const TArray<FNavigationDirtyArea>& DirtyAreas)
{
	for (const auto& DirtyArea : DirtyAreas)
	{
		const auto MatchingBounds = RegisteredNavigationBounds.FilterByPredicate(
			[&DirtyArea](const FBox& Box)
			{
				return Box == DirtyArea.Bounds || Box.IsInside(DirtyArea.Bounds) ||
					Box.Intersect(DirtyArea.Bounds);
			});

		for (const auto& MatchingBoundsElement : MatchingBounds)
		{
			// Don't add another pending generation if one is already there for the
			// navigation bounds the dirty area is in
			if (PendingBoundsDataGenerationElements.FindByPredicate(
				[&MatchingBoundsElement](
				const FPendingBoundsDataGenerationElement& PendingElement)
				{
					return PendingElement.VolumeBounds == MatchingBoundsElement;
				}) == nullptr)
			{
				FPendingBoundsDataGenerationElement PendingBoxElement;
				PendingBoxElement.VolumeBounds = MatchingBoundsElement;
				PendingBoundsDataGenerationElements.Emplace(PendingBoxElement);
			}
		}
	}

	// Sort tiles by proximity to players
	if (PendingBoundsDataGenerationElements.Num() > 0)
	{
		SortPendingBounds();
	}
}

bool FNav3DDataGenerator::IsBuildInProgressCheckDirty() const
{
	return RunningBoundsDataGenerationElements.Num() ||
		PendingBoundsDataGenerationElements.Num();
}

int32 FNav3DDataGenerator::GetNumRemaningBuildTasks() const
{
	return RunningBoundsDataGenerationElements.Num() +
		PendingBoundsDataGenerationElements.Num();
}

int32 FNav3DDataGenerator::GetNumRunningBuildTasks() const
{
	return RunningBoundsDataGenerationElements.Num();
}

void FNav3DDataGenerator::GetSeedLocations(TArray<FVector2D>& SeedLocations,
                                           const UWorld& World)
{
	// Collect players positions
	for (FConstPlayerControllerIterator PlayerIterator =
		     World.GetPlayerControllerIterator();
	     PlayerIterator; ++PlayerIterator)
	{
		if (const auto* PlayerController = PlayerIterator->Get())
		{
			if (const auto Pawn = PlayerController->GetPawn())
			{
				const FVector2D SeedLocation(Pawn->GetActorLocation());
				SeedLocations.Add(SeedLocation);
			}
		}
	}
}

void FNav3DDataGenerator::SortPendingBounds()
{
	if (const UWorld* CurrentWorld = GetWorld())
	{
		TArray<FVector2D> SeedLocations;
		GetSeedLocations(SeedLocations, *CurrentWorld);

		if (SeedLocations.Num() == 0)
		{
			SeedLocations.Add(FVector2D(TotalNavigationBounds.GetCenter()));
		}

		if (SeedLocations.Num() > 0)
		{
			for (auto& Element : PendingBoundsDataGenerationElements)
			{
				FVector2D TileCenter2D = FVector2D(Element.VolumeBounds.GetCenter());
				for (const auto& SeedLocation : SeedLocations)
				{
					Element.SeedDistance =
						FMath::Min(Element.SeedDistance,
						           FVector2D::DistSquared(TileCenter2D, SeedLocation));
				}
			}

			PendingBoundsDataGenerationElements.Sort();
		}
	}
}

void FNav3DDataGenerator::UpdateNavigationBounds()
{
	UE_LOG(LogNav3D, Log, TEXT("Updating navigation bounds with automatic partitioning..."));

	// If we're in a targeted single-volume (chunk) build, do not recompute from world volumes.
	if (bIsSingleVolumeBuild && RegisteredNavigationBounds.Num() > 0)
	{
		TotalNavigationBounds = RegisteredNavigationBounds[0];
		UE_LOG(LogNav3D, Log, TEXT("Single-target build: skipping global partitioning. Using %s"), *TotalNavigationBounds.ToString());
		return;
	}
	
	// Get original bounds from Nav3DBoundsVolume actors
	TArray<FBox> OriginalVolumes = GetOriginalNavigationBounds();
	
	// Apply automatic partitioning to each volume
	RegisteredNavigationBounds.Reset();
	for (const FBox& OriginalVolume : OriginalVolumes)
	{
		TArray<FBox> PartitionedVolumes = PartitionVolumeIfNeeded(OriginalVolume);
		RegisteredNavigationBounds.Append(PartitionedVolumes);
		
		UE_LOG(LogNav3D, Log, TEXT("Volume %s partitioned into %d sub-volumes"), 
		       *OriginalVolume.ToString(), PartitionedVolumes.Num());
	}
	
	ValidatePartitionedVolumes(TArray<FBox>(RegisteredNavigationBounds));
	
	// Calculate total bounds
	FBox BoundsSum(ForceInit);
	for (const FBox& Box : RegisteredNavigationBounds)
	{
		BoundsSum += Box;
	}
	TotalNavigationBounds = BoundsSum;
	
	UE_LOG(LogNav3D, Log, TEXT("Total volumes after partitioning: %d"), RegisteredNavigationBounds.Num());
}

TArray<FBox> FNav3DDataGenerator::ProcessAsyncTasks(const int32 TaskToProcessCount)
{
	const bool HasTasksAtStart = GetNumRemaningBuildTasks() > 0;

	int32 ProcessedTasksCount = 0;
	if (!FNav3DVolumeNavigationData::IsCancelRequested())
	{
		for (int32 ElementIndex = PendingBoundsDataGenerationElements.Num() - 1;
		     ElementIndex >= 0 && ProcessedTasksCount < TaskToProcessCount;
		     ElementIndex--)
		{
			FPendingBoundsDataGenerationElement& PendingElement =
				PendingBoundsDataGenerationElements[ElementIndex];
			FRunningBoundsDataGenerationElement RunningElement(
				PendingElement.VolumeBounds);

			if (RunningBoundsDataGenerationElements.Contains(RunningElement))
			{
				continue;
			}

			TUniquePtr<FNav3DBoxGeneratorTask> Task =
				MakeUnique<FNav3DBoxGeneratorTask>(
					CreateBoxNavigationGenerator(PendingElement.VolumeBounds));

			RunningElement.AsyncTask = Task.Release();

			UE_LOG(LogNav3D, Log, TEXT("Starting volume build: %s (running=%d)"), *PendingElement.VolumeBounds.ToString(), RunningBoundsDataGenerationElements.Num()+1);
			RunningElement.AsyncTask->StartBackgroundTask();

			RunningBoundsDataGenerationElements.Add(RunningElement);

			PendingBoundsDataGenerationElements.RemoveAt(ElementIndex, 1, EAllowShrinking::No);
			ProcessedTasksCount++;
		}
	}

	if (ProcessedTasksCount > 0 &&
		PendingBoundsDataGenerationElements.Num() == 0)
	{
		PendingBoundsDataGenerationElements.Empty(64);
	}

	TArray<FBox> FinishedBoxes;

	for (int32 Index = RunningBoundsDataGenerationElements.Num() - 1; Index >= 0;
	     --Index)
	{
		FRunningBoundsDataGenerationElement& Element =
			RunningBoundsDataGenerationElements[Index];
		check(Element.AsyncTask != nullptr);

		if (!Element.AsyncTask->IsDone())
		{
			continue;
		}

		if (Element.ShouldDiscard)
		{
			continue;
		}

		auto& BoxGenerator =
			*Element.AsyncTask->GetTask().BoxNavigationDataGenerator;
		FNav3DVolumeNavigationData GeneratedData = BoxGenerator.GetBoundsNavigationData();

		// Create chunk actor for this volume
		if (ANav3DDataChunkActor* ChunkActor = CreateChunkActorForVolume(Element.VolumeBounds, GeneratedData))
		{
			NavigationData.RegisterChunkActor(ChunkActor);
			UE_LOG(LogNav3D, Log, TEXT("Finished volume build: %s (remaining running=%d pending=%d)"), *Element.VolumeBounds.ToString(), RunningBoundsDataGenerationElements.Num()-1, PendingBoundsDataGenerationElements.Num());
		}

		FinishedBoxes.Emplace(MoveTemp(Element.VolumeBounds));

		delete Element.AsyncTask;
		Element.AsyncTask = nullptr;
		RunningBoundsDataGenerationElements.RemoveAtSwap(Index, 1, EAllowShrinking::No);
	}

	const bool HasTasksAtEnd = GetNumRemaningBuildTasks() > 0;
	if (HasTasksAtStart && !HasTasksAtEnd)
	{
		// Build adjacency between all chunk actors
		const TArray<ANav3DDataChunkActor*> AllChunkActors = NavigationData.GetAllChunkActors();
		if (AllChunkActors.Num() > 0)
		{
			BuildAdjacencyBetweenChunkActors(AllChunkActors);
		}
		
		
		// Log build completion timing and connection statistics
		if (NavigationData.SingleVolumeBuildStartTime > 0.0)
		{
			const double BuildEndTime = FPlatformTime::Seconds();
			const double BuildDuration = BuildEndTime - NavigationData.SingleVolumeBuildStartTime;
			
			// Count total compact portals across all chunk actors
			int32 TotalConnections = 0;
			for (ANav3DDataChunkActor* ChunkActor : AllChunkActors)
			{
				if (ChunkActor)
				{
					for (const FNav3DChunkAdjacency& Adjacency : ChunkActor->ChunkAdjacency)
					{
						TotalConnections += Adjacency.CompactPortals.Num();
					}
				}
			}
			
			UE_LOG(LogNav3D, Log, TEXT("Nav3D navigation build completed in %.3f seconds (tactical data will be built separately)"), BuildDuration);
			UE_LOG(LogNav3D, Log, TEXT("Total compact portals created between chunks: %d"), TotalConnections);
			
			// Reset the build start time
			NavigationData.SingleVolumeBuildStartTime = 0.0;
		}
		
		// QUICK_SCOPE_CYCLE_COUNTER(STAT_RecastNavMeshGenerator_OnNavMeshGenerationFinished);
		UE_LOG(LogNav3D, Log, TEXT("Navigation build completed - calling OnNavigationDataGenerationFinished"));
		
		// Navigation build is complete (tactical data will be built separately after async tasks finish)
		NavigationData.OnNavigationDataGenerationFinished();
		UE_LOG(LogNav3D, Log, TEXT("OnNavigationDataGenerationFinished completed"));
		
		UE_LOG(LogNav3D, Log, TEXT("ProcessAsyncTasks completed successfully - all navigation build tasks finished"));
	}

	return FinishedBoxes;
}

TSharedRef<FNav3DVolumeNavigationDataGenerator>
FNav3DDataGenerator::CreateBoxNavigationGenerator(const FBox& Box)
{
	// SCOPE_CYCLE_COUNTER(STAT_Nav3D_CreateBoxNavigationGenerator);

	TSharedRef<FNav3DVolumeNavigationDataGenerator> BoxNavigationDataGenerator =
		MakeShareable(new FNav3DVolumeNavigationDataGenerator(*this, Box));
	return BoxNavigationDataGenerator;
}

void FNav3DDataGenerator::SetBuildTargetVolume(const FBox& VolumeBounds)
{
	// Store current bounds for restoration
	OriginalNavigationBounds = RegisteredNavigationBounds;
	
	// Set only the target volume
	RegisteredNavigationBounds.Reset();
	RegisteredNavigationBounds.Add(VolumeBounds);
	
	// Update total bounds
	TotalNavigationBounds = VolumeBounds;
	
	// Mark as single volume build
	bIsSingleVolumeBuild = true;
	
	UE_LOG(LogNav3D, Log, TEXT("Set build target to single volume: %s"), *VolumeBounds.ToString());
}

void FNav3DDataGenerator::RestoreAllVolumes()
{
	// Restore original bounds
	RegisteredNavigationBounds = OriginalNavigationBounds;
	
	// Recalculate total bounds
	FBox BoundsSum(ForceInit);
	for (const FBox& Box : RegisteredNavigationBounds)
	{
		BoundsSum += Box;
	}
	TotalNavigationBounds = BoundsSum;
	
	// Clear single volume build flag
	bIsSingleVolumeBuild = false;
	
	UE_LOG(LogNav3D, Log, TEXT("Restored all volumes: %d total"), RegisteredNavigationBounds.Num());
}

// ============================================================================
// VOLUME PARTITIONING IMPLEMENTATION
// ============================================================================

TArray<FBox> FNav3DDataGenerator::GetOriginalNavigationBounds() const
{
	TArray<FBox> OriginalVolumes;
	
	if (const UNavigationSystemV1* NavigationSystem =
		FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld()))
	{
		if (!NavigationSystem->ShouldGenerateNavigationEverywhere())
		{
			TArray<FBox> SupportedNavigationBounds;
			NavigationSystem->GetNavigationBoundsForNavData(
				NavigationData, SupportedNavigationBounds);
			OriginalVolumes = SupportedNavigationBounds;
		}
		else
		{
			if (const FBox WorldBounds = NavigationSystem->GetWorldBounds(); WorldBounds.IsValid)
			{
				OriginalVolumes.Add(WorldBounds);
			}
		}
	}
	
	return OriginalVolumes;
}

TArray<FBox> FNav3DDataGenerator::PartitionVolumeIfNeeded(const FBox& OriginalVolume)
{
	const UNav3DSettings* Settings = UNav3DSettings::Get();
	
	// Check if partitioning is enabled
	if (!Settings->bEnableAutomaticVolumePartitioning)
	{
		UE_LOG(LogNav3D, Verbose, TEXT("Automatic partitioning disabled, keeping original volume"));
		return {OriginalVolume};
	}
	
	const float MaxSize = Settings->MaxVolumePartitionSize;
	const int32 MaxSubVolumes = Settings->MaxSubVolumesPerAxis;
	const bool bPreferCubes = Settings->bPreferCubePartitions;

	const FVector VolumeSize = OriginalVolume.GetSize();
	
	// Calculate divisions needed per axis
	int32 XDiv = FMath::CeilToInt(VolumeSize.X / MaxSize);
	int32 YDiv = FMath::CeilToInt(VolumeSize.Y / MaxSize);
	int32 ZDiv = FMath::CeilToInt(VolumeSize.Z / MaxSize);
	
	// Clamp to maximum subdivisions
	XDiv = FMath::Min(XDiv, MaxSubVolumes);
	YDiv = FMath::Min(YDiv, MaxSubVolumes);
	ZDiv = FMath::Min(ZDiv, MaxSubVolumes);
	
	// Optimize for cubic partitions if preferred
	if (bPreferCubes && (XDiv != YDiv || YDiv != ZDiv))
	{
		const int32 MaxDiv = FMath::Max3(XDiv, YDiv, ZDiv);
		XDiv = YDiv = ZDiv = MaxDiv;
		
		UE_LOG(LogNav3D, Verbose, TEXT("Adjusted to cubic partitions: %dx%dx%d"), XDiv, YDiv, ZDiv);
	}
	
	// Check if partitioning is actually needed
	if (XDiv == 1 && YDiv == 1 && ZDiv == 1)
	{
		UE_LOG(LogNav3D, Verbose, TEXT("Volume %s does not need partitioning"), *OriginalVolume.ToString());
		return {OriginalVolume};
	}
	
	// Create sub-volumes
	TArray<FBox> SubVolumes;
	SubVolumes.Reserve(XDiv * YDiv * ZDiv);

	const FVector SubVolumeSize = VolumeSize / FVector(XDiv, YDiv, ZDiv);
	
	UE_LOG(LogNav3D, Log, TEXT("Partitioning volume %s (%s) into %dx%dx%d sub-volumes of size %s"), 
	       *OriginalVolume.ToString(), *VolumeSize.ToString(), XDiv, YDiv, ZDiv, *SubVolumeSize.ToString());
	
	for (int32 X = 0; X < XDiv; X++)
	{
		for (int32 Y = 0; Y < YDiv; Y++)
		{
			for (int32 Z = 0; Z < ZDiv; Z++)
			{
				FVector SubVolumeMin = OriginalVolume.Min + FVector(X, Y, Z) * SubVolumeSize;
				FVector SubVolumeMax = SubVolumeMin + SubVolumeSize;
				
				// Ensure last sub-volume in each axis extends to original bounds
				if (X == XDiv - 1) SubVolumeMax.X = OriginalVolume.Max.X;
				if (Y == YDiv - 1) SubVolumeMax.Y = OriginalVolume.Max.Y;
				if (Z == ZDiv - 1) SubVolumeMax.Z = OriginalVolume.Max.Z;
				
				FBox SubVolume(SubVolumeMin, SubVolumeMax);
				SubVolumes.Add(SubVolume);
				
				UE_LOG(LogNav3D, VeryVerbose, TEXT("  Sub-volume [%d,%d,%d]: %s"), 
				       X, Y, Z, *SubVolume.ToString());
			}
		}
	}
	
	return SubVolumes;
}

void FNav3DDataGenerator::ValidatePartitionedVolumes(const TArray<FBox>& Volumes)
{
	// Validation checks
	for (int32 i = 0; i < Volumes.Num(); i++)
	{
		const FBox& Volume = Volumes[i];
		
		if (!Volume.IsValid)
		{
			UE_LOG(LogNav3D, Error, TEXT("Invalid volume at index %d: %s"), i, *Volume.ToString());
			continue;
		}
	}
}

// ============================================================================
// CHUNK ACTOR CREATION AND MANAGEMENT
// ============================================================================

ANav3DDataChunkActor* FNav3DDataGenerator::CreateChunkActorForVolume(
	const FBox& VolumeBounds, const FNav3DVolumeNavigationData& NavData) const
{
	UWorld* World = GetWorld();
	if (!World)
	{
		UE_LOG(LogNav3D, Error, TEXT("Cannot create chunk actor: No valid world"));
		return nullptr;
	}
	
	// Create chunk actor
	FActorSpawnParameters SpawnParams;
	SpawnParams.bDeferConstruction = false;
	SpawnParams.bCreateActorPackage = World->IsPartitionedWorld(); // Only create packages for world partition
	
	ANav3DDataChunkActor* ChunkActor = World->SpawnActor<ANav3DDataChunkActor>(SpawnParams);
	if (!ChunkActor)
	{
		UE_LOG(LogNav3D, Error, TEXT("Failed to spawn chunk actor for volume %s"), *VolumeBounds.ToString());
		return nullptr;
	}
	
	// Configure chunk actor
    ChunkActor->SetDataChunkActorBounds(VolumeBounds);
	
	// Get the chunk index for naming
	int32 ChunkIndex = 0;
	if (NavigationData.IsValidLowLevel())
	{
		ChunkIndex = NavigationData.ChunkActors.Num();
	}
	
	ChunkActor->SetActorLabel(FString::Printf(TEXT("Nav3dChunk_%d"), ChunkIndex));
	
	// Position the chunk actor at the center of its bounds
	ChunkActor->SetActorLocation(VolumeBounds.GetCenter());

#if WITH_EDITOR
	// Place all chunk actors under folder designated by owning Nav3DBoundsVolume
	{
		FString FolderName;
		for (TActorIterator<ANav3DBoundsVolume> It(World); It; ++It)
		{
			const ANav3DBoundsVolume* BoundsVolume = *It;
			if (!BoundsVolume || !IsValid(BoundsVolume)) { continue; }
			const FBox OwnerBounds = BoundsVolume->GetComponentsBoundingBox(true);
			if (OwnerBounds.IsInside(VolumeBounds.Min) && OwnerBounds.IsInside(VolumeBounds.Max))
			{
				FString OwnerLabel = BoundsVolume->GetActorLabel();
				OwnerLabel.ReplaceInline(TEXT("/"), TEXT("_"));
				OwnerLabel.ReplaceInline(TEXT("\\"), TEXT("_"));
				OwnerLabel.ReplaceInline(TEXT(":"), TEXT("_"));
				OwnerLabel.ReplaceInline(TEXT("*"), TEXT("_"));
				OwnerLabel.ReplaceInline(TEXT("?"), TEXT("_"));
				OwnerLabel.ReplaceInline(TEXT("\""), TEXT("_"));
				OwnerLabel.ReplaceInline(TEXT("<"), TEXT("_"));
				OwnerLabel.ReplaceInline(TEXT(">"), TEXT("_"));
				OwnerLabel.ReplaceInline(TEXT("|"), TEXT("_"));
				FolderName = FString::Printf(TEXT("%s-Chunks"), *OwnerLabel);
				break;
			}
		}
		if (!FolderName.IsEmpty())
		{
			ChunkActor->SetFolderPath(FName(*FolderName));
		}
	}
#endif
	
	// Create navigation data chunk
	UNav3DDataChunk* Chunk = NewObject<UNav3DDataChunk>(ChunkActor);
	Chunk->AddNavigationData(const_cast<FNav3DVolumeNavigationData&>(NavData));
	ChunkActor->Nav3DChunks.Add(Chunk);
	
	// Build boundary voxels
	FNav3DUtils::IdentifyBoundaryVoxels(Chunk);
	
	// Initialize for appropriate level type
	if (World->IsPartitionedWorld())
	{
		ChunkActor->InitializeForWorldPartition();
	}
	else
	{
		ChunkActor->InitializeForStandardLevel();
	}

	// Defer tactical data building until after the build is complete
	// This prevents access violations during the build process
	if (NavigationData.TacticalSettings.bEnableTacticalReasoning)
	{
		UE_LOG(LogNav3D, Verbose, TEXT("Tactical reasoning enabled - will build tactical data after build completion"));
		// Mark chunk as needing tactical data build
		ChunkActor->bNeedsTacticalDataBuild = true;
	}
	else
	{
		UE_LOG(LogNav3D, Verbose, TEXT("Tactical reasoning disabled, skipping tactical data generation"));
	}
	
	// Mark as built
	ChunkActor->bIsBuilt = true;
	ChunkActor->bIsBuilding = false;
	ChunkActor->bNeedsRebuild = false;
	
	UE_LOG(LogNav3D, Log, TEXT("Created chunk actor for volume %s"), *VolumeBounds.ToString());
	
	return ChunkActor;
}

void FNav3DDataGenerator::BuildAdjacencyBetweenChunkActors(const TArray<ANav3DDataChunkActor*>& ChunkActors) const
{
	UE_LOG(LogNav3D, Log, TEXT("Building adjacency between %d chunk actors"), ChunkActors.Num());
	
	TArray<ANav3DDataChunkActor*> AllChunkActors = NavigationData.GetAllChunkActors();
	
	// Build adjacency between all chunk actors (both new and existing)
	for (int32 i = 0; i < AllChunkActors.Num(); ++i)
	{
		for (int32 j = i + 1; j < AllChunkActors.Num(); ++j)
		{
			ANav3DDataChunkActor* ActorA = AllChunkActors[i];
			ANav3DDataChunkActor* ActorB = AllChunkActors[j];
			
			if (!ActorA || !ActorB) continue;
			
			// Get voxel size for adjacency testing
			float VoxelSize = 0.0f;
			if (ActorA->Nav3DChunks.Num() > 0)
			{
				VoxelSize = FNav3DUtils::GetChunkLeafNodeSize(ActorA->Nav3DChunks[0]);
			}
			else
			{
				UE_LOG(LogNav3D, Verbose, TEXT("Actor %s has no Nav3DChunks; skipping adjacency test vs %s"),
				       *ActorA->GetName(), *ActorB->GetName());
			}
			
			if (VoxelSize <= 0.0f)
			{
				UE_LOG(LogNav3D, Verbose, TEXT("VoxelSize invalid (%.3f) for pair %s <-> %s; skipping"),
				       VoxelSize, *ActorA->GetName(), *ActorB->GetName());
				continue;
			}
			
			// Check if actors are adjacent
			const FBox ExpandedA = ActorA->DataChunkActorBounds.ExpandBy(VoxelSize);
			const bool bIntersects = ExpandedA.Intersect(ActorB->DataChunkActorBounds);
			UE_LOG(LogNav3D, Verbose, TEXT("Bounds test %s <-> %s | VoxelSize=%.3f | ExpandedA=%s | B=%s | Intersect=%s"),
			       *ActorA->GetName(), *ActorB->GetName(), VoxelSize,
			       *ExpandedA.ToString(), *ActorB->DataChunkActorBounds.ToString(), bIntersects ? TEXT("true") : TEXT("false"));
			if (bIntersects)
			{
				// Reuse exact same adjacency building logic from world partition
				BuildAdjacencyBetweenTwoChunkActors(ActorA, ActorB, VoxelSize);
			}
			else
			{
				UE_LOG(LogNav3D, Verbose, TEXT("Actors not adjacent after bounds test: %s <-> %s"),
				       *ActorA->GetName(), *ActorB->GetName());
			}
		}
	}
	
	UE_LOG(LogNav3D, Log, TEXT("Adjacency building complete"));
}

void FNav3DDataGenerator::BuildAdjacencyBetweenTwoChunkActors(ANav3DDataChunkActor* ActorA, ANav3DDataChunkActor* ActorB, float VoxelSize)
{
	if (!ActorA || !ActorB || ActorA == ActorB)
	{
		return;
	}
	
	UE_LOG(LogNav3D, Verbose, TEXT("Building adjacency between chunk actors %s and %s"), 
	       *ActorA->GetName(), *ActorB->GetName());
	
	// Ensure boundary voxels are identified
	for (UNav3DDataChunk* Chunk : ActorA->Nav3DChunks)
	{
		if (Chunk && Chunk->BoundaryVoxels.Num() == 0)
		{
			FNav3DUtils::IdentifyBoundaryVoxels(Chunk);
			UE_LOG(LogNav3D, VeryVerbose, TEXT("Identified %d boundary voxels for chunk in %s"), Chunk->BoundaryVoxels.Num(), *ActorA->GetName());
		}
	}
	
	for (UNav3DDataChunk* Chunk : ActorB->Nav3DChunks)
	{
		if (Chunk && Chunk->BoundaryVoxels.Num() == 0)
		{
			FNav3DUtils::IdentifyBoundaryVoxels(Chunk);
			UE_LOG(LogNav3D, VeryVerbose, TEXT("Identified %d boundary voxels for chunk in %s"), Chunk->BoundaryVoxels.Num(), *ActorB->GetName());
		}
	}
	
	// Add connections to both actors using ChunkAdjacency
	int32 TotalConnectionsAdded = 0;
	
	// Build adjacency between chunks in different actors
	TArray<FNav3DVoxelConnection> ConnectionsAB;
	for (UNav3DDataChunk* A : ActorA->Nav3DChunks)
	{
		if (!A) continue;
		for (UNav3DDataChunk* B : ActorB->Nav3DChunks)
		{
			if (!B) continue;
			if (!FNav3DUtils::AreChunksAdjacent(A, B, VoxelSize))
			{
				UE_LOG(LogNav3D, VeryVerbose, TEXT("Chunks not adjacent: %s<->%s | ABoundary=%d BBoundary=%d | VoxelSize=%.3f"),
				       *ActorA->GetName(), *ActorB->GetName(), A->BoundaryVoxels.Num(), B->BoundaryVoxels.Num(), VoxelSize);
				continue;
			}
			else
			{
				UE_LOG(LogNav3D, Verbose, TEXT("Chunks adjacent: %s<->%s | ABoundary=%d BBoundary=%d | Building connections"),
				       *ActorA->GetName(), *ActorB->GetName(), A->BoundaryVoxels.Num(), B->BoundaryVoxels.Num());
			}

			// Get volume data for proper world position conversion
			const FNav3DVolumeNavigationData* VolumeA = A->GetVolumeNavigationData();
			const FNav3DVolumeNavigationData* VolumeB = B->GetVolumeNavigationData();
			
			if (!VolumeA || !VolumeB)
			{
				UE_LOG(LogNav3D, Warning, TEXT("BuildAdjacencyBetweenTwoChunkActors: Missing volume data for chunks"));
				continue;
			}
			
			float AdjacencyClearance = VolumeA->GetSettings().GenerationSettings.AdjacencyClearance;
			// Temporarily increase clearance to test if this is the issue
			AdjacencyClearance = FMath::Max(AdjacencyClearance, VoxelSize * 0.5f); // Use at least half voxel size
			UE_LOG(LogNav3D, Verbose, TEXT("Using AdjacencyClearance=%.2f (original=%.2f, VoxelSize=%.2f) for adjacency between %s and %s"), 
				AdjacencyClearance, VolumeA->GetSettings().GenerationSettings.AdjacencyClearance, VoxelSize, *ActorA->GetName(), *ActorB->GetName());
			
			// Early bounds check - if volumes don't share a face, skip detailed comparison
			const FBox& BoundsA = VolumeA->GetNavigationBounds();
			const FBox& BoundsB = VolumeB->GetNavigationBounds();
			
			// Check if volumes actually share a face (not just intersect)
			bool bShareFace = false;
			float SharedFaceDistance = FLT_MAX;
			uint8 FaceA = 0, FaceB = 0;  // Which faces are shared
			
			// Check each axis for face sharing with appropriate tolerance
			// Use a tolerance based on voxel size since volumes can have gaps up to voxel size
			const float FaceTolerance = VoxelSize * 1.5f;  // Allow up to 1.5 voxel sizes gap
			
			UE_LOG(LogNav3D, VeryVerbose, TEXT("Face detection: VoxelSize=%.2f, FaceTolerance=%.2f"), VoxelSize, FaceTolerance);
			UE_LOG(LogNav3D, VeryVerbose, TEXT("  BoundsA: Min=%s, Max=%s"), *BoundsA.Min.ToString(), *BoundsA.Max.ToString());
			UE_LOG(LogNav3D, VeryVerbose, TEXT("  BoundsB: Min=%s, Max=%s"), *BoundsB.Min.ToString(), *BoundsB.Max.ToString());
			
			// Check X-axis faces
			const float XDist1 = FMath::Abs(BoundsA.Max.X - BoundsB.Min.X);
			const float XDist2 = FMath::Abs(BoundsB.Max.X - BoundsA.Min.X);
			UE_LOG(LogNav3D, VeryVerbose, TEXT("  X-axis: A.Max.X-B.Min.X=%.2f, B.Max.X-A.Min.X=%.2f (tolerance=%.2f)"), XDist1, XDist2, FaceTolerance);
			
			if (FMath::IsNearlyEqual(BoundsA.Max.X, BoundsB.Min.X, FaceTolerance))
			{
				bShareFace = true;
				SharedFaceDistance = XDist1;
				FaceA = 1; FaceB = 2;  // A's +X face touches B's -X face
				UE_LOG(LogNav3D, VeryVerbose, TEXT("  -> X face match: A+X touches B-X, distance=%.2f"), XDist1);
			}
			else if (FMath::IsNearlyEqual(BoundsB.Max.X, BoundsA.Min.X, FaceTolerance))
			{
				bShareFace = true;
				SharedFaceDistance = XDist2;
				FaceA = 2; FaceB = 1;  // A's -X face touches B's +X face
				UE_LOG(LogNav3D, VeryVerbose, TEXT("  -> X face match: A-X touches B+X, distance=%.2f"), XDist2);
			}
			
			// Check Y-axis faces
			const float YDist1 = FMath::Abs(BoundsA.Max.Y - BoundsB.Min.Y);
			const float YDist2 = FMath::Abs(BoundsB.Max.Y - BoundsA.Min.Y);
			UE_LOG(LogNav3D, VeryVerbose, TEXT("  Y-axis: A.Max.Y-B.Min.Y=%.2f, B.Max.Y-A.Min.Y=%.2f (tolerance=%.2f)"), YDist1, YDist2, FaceTolerance);
			
			if (!bShareFace && FMath::IsNearlyEqual(BoundsA.Max.Y, BoundsB.Min.Y, FaceTolerance))
			{
				bShareFace = true;
				SharedFaceDistance = YDist1;
				FaceA = 4; FaceB = 8;  // A's +Y face touches B's -Y face
				UE_LOG(LogNav3D, VeryVerbose, TEXT("  -> Y face match: A+Y touches B-Y, distance=%.2f"), YDist1);
			}
			else if (!bShareFace && FMath::IsNearlyEqual(BoundsB.Max.Y, BoundsA.Min.Y, FaceTolerance))
			{
				bShareFace = true;
				SharedFaceDistance = YDist2;
				FaceA = 8; FaceB = 4;  // A's -Y face touches B's +Y face
				UE_LOG(LogNav3D, VeryVerbose, TEXT("  -> Y face match: A-Y touches B+Y, distance=%.2f"), YDist2);
			}
			
			// Check Z-axis faces
			const float ZDist1 = FMath::Abs(BoundsA.Max.Z - BoundsB.Min.Z);
			const float ZDist2 = FMath::Abs(BoundsB.Max.Z - BoundsA.Min.Z);
			UE_LOG(LogNav3D, VeryVerbose, TEXT("  Z-axis: A.Max.Z-B.Min.Z=%.2f, B.Max.Z-A.Min.Z=%.2f (tolerance=%.2f)"), ZDist1, ZDist2, FaceTolerance);
			
			if (!bShareFace && FMath::IsNearlyEqual(BoundsA.Max.Z, BoundsB.Min.Z, FaceTolerance))
			{
				bShareFace = true;
				SharedFaceDistance = ZDist1;
				FaceA = 16; FaceB = 32;  // A's +Z face touches B's -Z face
				UE_LOG(LogNav3D, VeryVerbose, TEXT("  -> Z face match: A+Z touches B-Z, distance=%.2f"), ZDist1);
			}
			else if (!bShareFace && FMath::IsNearlyEqual(BoundsB.Max.Z, BoundsA.Min.Z, FaceTolerance))
			{
				bShareFace = true;
				SharedFaceDistance = ZDist2;
				FaceA = 32; FaceB = 16;  // A's -Z face touches B's +Z face
				UE_LOG(LogNav3D, VeryVerbose, TEXT("  -> Z face match: A-Z touches B+Z, distance=%.2f"), ZDist2);
			}
			
			if (!bShareFace)
			{
				UE_LOG(LogNav3D, Verbose, TEXT("Volumes %s and %s don't share a face - skipping detailed adjacency check"), 
					*ActorA->GetName(), *ActorB->GetName());
				continue;
			}
			
			UE_LOG(LogNav3D, Verbose, TEXT("Volumes %s and %s share a face with distance %.2f - checking adjacency (FaceA=%d, FaceB=%d)"), 
				*ActorA->GetName(), *ActorB->GetName(), SharedFaceDistance, FaceA, FaceB);
			
			// Debug: Log boundary voxel counts and face flags
			UE_LOG(LogNav3D, Verbose, TEXT("Boundary voxel analysis: A has %d boundary voxels, B has %d boundary voxels"), 
				A->BoundaryVoxels.Num(), B->BoundaryVoxels.Num());
			
			int32 AVoxelsOnFace = 0, BVoxelsOnFace = 0;
			for (const FNav3DEdgeVoxel& VoxelA : A->BoundaryVoxels)
			{
				bool bVoxelAOnSharedFace = false;
				if (FaceA == 1) bVoxelAOnSharedFace = VoxelA.bOnMaxXFace;
				else if (FaceA == 2) bVoxelAOnSharedFace = VoxelA.bOnMinXFace;
				else if (FaceA == 4) bVoxelAOnSharedFace = VoxelA.bOnMaxYFace;
				else if (FaceA == 8) bVoxelAOnSharedFace = VoxelA.bOnMinYFace;
				else if (FaceA == 16) bVoxelAOnSharedFace = VoxelA.bOnMaxZFace;
				else if (FaceA == 32) bVoxelAOnSharedFace = VoxelA.bOnMinZFace;
				if (bVoxelAOnSharedFace) AVoxelsOnFace++;
			}
			
			for (const FNav3DEdgeVoxel& VoxelB : B->BoundaryVoxels)
			{
				bool bVoxelBOnSharedFace = false;
				if (FaceB == 1) bVoxelBOnSharedFace = VoxelB.bOnMaxXFace;
				else if (FaceB == 2) bVoxelBOnSharedFace = VoxelB.bOnMinXFace;
				else if (FaceB == 4) bVoxelBOnSharedFace = VoxelB.bOnMaxYFace;
				else if (FaceB == 8) bVoxelBOnSharedFace = VoxelB.bOnMinYFace;
				else if (FaceB == 16) bVoxelBOnSharedFace = VoxelB.bOnMaxZFace;
				else if (FaceB == 32) bVoxelBOnSharedFace = VoxelB.bOnMinZFace;
				if (bVoxelBOnSharedFace) BVoxelsOnFace++;
			}
			
			UE_LOG(LogNav3D, Verbose, TEXT("Face analysis: FaceA=%d, FaceB=%d, AVoxelsOnFace=%d, BVoxelsOnFace=%d"), 
				FaceA, FaceB, AVoxelsOnFace, BVoxelsOnFace);
			
			// Bucket by Local morton and keep 3 nearest per local
			TMap<uint64, TArray<FNav3DVoxelConnection>> LocalToConns;
			int32 VoxelComparisons = 0;
			
			for (const FNav3DEdgeVoxel& VoxelA : A->BoundaryVoxels)
			{
				// Only check voxels on the shared face for volume A
				bool bVoxelAOnSharedFace = false;
				if (FaceA == 1) bVoxelAOnSharedFace = VoxelA.bOnMaxXFace;
				else if (FaceA == 2) bVoxelAOnSharedFace = VoxelA.bOnMinXFace;
				else if (FaceA == 4) bVoxelAOnSharedFace = VoxelA.bOnMaxYFace;
				else if (FaceA == 8) bVoxelAOnSharedFace = VoxelA.bOnMinYFace;
				else if (FaceA == 16) bVoxelAOnSharedFace = VoxelA.bOnMaxZFace;
				else if (FaceA == 32) bVoxelAOnSharedFace = VoxelA.bOnMinZFace;
				
				if (!bVoxelAOnSharedFace) continue;
				
				// Get world position for voxel A
				FVector PosA;
				if (VoxelA.LayerIndex == 0)
				{
					PosA = VolumeA->GetLeafNodePositionFromMortonCode(VoxelA.Morton);
				}
				else
				{
					PosA = VolumeA->GetNodePositionFromLayerAndMortonCode(VoxelA.LayerIndex, VoxelA.Morton);
				}

				TArray<FNav3DVoxelConnection>& Bucket = LocalToConns.FindOrAdd(VoxelA.Morton);
				for (const FNav3DEdgeVoxel& VoxelB : B->BoundaryVoxels)
				{
					// Only check voxels on the shared face for volume B
					bool bVoxelBOnSharedFace = false;
					if (FaceB == 1) bVoxelBOnSharedFace = VoxelB.bOnMaxXFace;
					else if (FaceB == 2) bVoxelBOnSharedFace = VoxelB.bOnMinXFace;
					else if (FaceB == 4) bVoxelBOnSharedFace = VoxelB.bOnMaxYFace;
					else if (FaceB == 8) bVoxelBOnSharedFace = VoxelB.bOnMinYFace;
					else if (FaceB == 16) bVoxelBOnSharedFace = VoxelB.bOnMaxZFace;
					else if (FaceB == 32) bVoxelBOnSharedFace = VoxelB.bOnMinZFace;
					
					if (!bVoxelBOnSharedFace) continue;
					
					VoxelComparisons++;
					
					// Get world position for voxel B
					FVector PosB;
					if (VoxelB.LayerIndex == 0)
					{
						PosB = VolumeB->GetLeafNodePositionFromMortonCode(VoxelB.Morton);
					}
					else
					{
						PosB = VolumeB->GetNodePositionFromLayerAndMortonCode(VoxelB.LayerIndex, VoxelB.Morton);
					}

					// Get voxel extents for each layer
					float VoxelExtentA, VoxelExtentB;
					if (VoxelA.LayerIndex == 0)
					{
						VoxelExtentA = VolumeA->GetData().GetLeafNodes().GetLeafNodeExtent();
					}
					else
					{
						VoxelExtentA = VolumeA->GetData().GetLayer(VoxelA.LayerIndex).GetNodeExtent();
					}
					
					if (VoxelB.LayerIndex == 0)
					{
						VoxelExtentB = VolumeB->GetData().GetLeafNodes().GetLeafNodeExtent();
					}
					else
					{
						VoxelExtentB = VolumeB->GetData().GetLayer(VoxelB.LayerIndex).GetNodeExtent();
					}
					
					// Use the smaller voxel extent for face sharing check
					const float MinVoxelExtent = FMath::Min(VoxelExtentA, VoxelExtentB);
					const float CenterToCenterDist = FVector::Dist(PosA, PosB);
					const float Threshold = MinVoxelExtent + AdjacencyClearance;
					
					if (FNav3DUtils::CheckVoxelFaceAdjacency(VoxelA, VoxelB, VolumeA, VolumeB, FaceA, FaceB, AdjacencyClearance))
					{
						UE_LOG(LogNav3D, VeryVerbose, TEXT("  -> CONNECTION: CenterToCenter=%.2f <= Threshold=%.2f"), CenterToCenterDist, Threshold);
						
						FNav3DVoxelConnection Conn;
						Conn.Local = VoxelA.Morton;
						Conn.LocalVolumeIndex = VoxelA.VolumeIndex;
						Conn.LocalChunkIndex = 0;
						Conn.Remote = VoxelB.Morton;
						Conn.RemoteVolumeIndex = VoxelB.VolumeIndex;
						Conn.RemoteChunkIndex = 0;
						Conn.Distance = CenterToCenterDist; // Store center-to-center distance

						// Insert sorted and cap to 3
						int32 InsertIdx = 0;
						while (InsertIdx < Bucket.Num() && Bucket[InsertIdx].Distance <= CenterToCenterDist) { ++InsertIdx; }
						Bucket.Insert(Conn, InsertIdx);
						if (Bucket.Num() > 3) { Bucket.SetNum(3, EAllowShrinking::No); }
					}
				}
			}
			
			UE_LOG(LogNav3D, Verbose, TEXT("Completed %d voxel comparisons for %s <-> %s"), 
				VoxelComparisons, *ActorA->GetName(), *ActorB->GetName());
			
			
			for (const auto& Pair : LocalToConns)
			{
				for (const TArray<FNav3DVoxelConnection>& Connections = Pair.Value;
					const FNav3DVoxelConnection& Conn : Connections)
				{
					// Find or create adjacency entry for ActorA -> ActorB
					FNav3DChunkAdjacency* AdjacencyAB = ActorA->ChunkAdjacency.FindByPredicate([ActorB](const FNav3DChunkAdjacency& Adj)
					{
						return Adj.OtherChunkActor.Get() == ActorB;
					});
					
					if (!AdjacencyAB)
					{
						FNav3DChunkAdjacency NewAdjacency;
						NewAdjacency.OtherChunkActor = ActorB;
						
						// Calculate spatial relationship data
						NewAdjacency.SharedFaceNormal = (ActorB->DataChunkActorBounds.GetCenter() - 
														ActorA->DataChunkActorBounds.GetCenter()).GetSafeNormal();
						NewAdjacency.ConnectionWeight = CalculateConnectionWeight(ActorA, ActorB);
						
						ActorA->ChunkAdjacency.Add(NewAdjacency);
						AdjacencyAB = &ActorA->ChunkAdjacency.Last();
					}
					
					// Directly add compact portal instead of Build connection
					{
						FCompactPortal CPab; CPab.Local = Conn.Local; CPab.Remote = Conn.Remote;
						AdjacencyAB->CompactPortals.Add(CPab);
					}
					
					// Find or create adjacency entry for ActorB -> ActorA (reverse connection)
					FNav3DChunkAdjacency* AdjacencyBA = ActorB->ChunkAdjacency.FindByPredicate([ActorA](const FNav3DChunkAdjacency& Adj)
					{
						return Adj.OtherChunkActor.Get() == ActorA;
					});
					
					if (!AdjacencyBA)
					{
						FNav3DChunkAdjacency NewAdjacency;
						NewAdjacency.OtherChunkActor = ActorA;
						
						// Calculate spatial relationship data (reverse direction)
						NewAdjacency.SharedFaceNormal = (ActorA->DataChunkActorBounds.GetCenter() - 
														ActorB->DataChunkActorBounds.GetCenter()).GetSafeNormal();
						NewAdjacency.ConnectionWeight = CalculateConnectionWeight(ActorB, ActorA);
						
						ActorB->ChunkAdjacency.Add(NewAdjacency);
						AdjacencyBA = &ActorB->ChunkAdjacency.Last();
					}
					
					// Add reverse compact portal
					{
						FCompactPortal CPba; CPba.Local = Conn.Remote; CPba.Remote = Conn.Local;
						AdjacencyBA->CompactPortals.Add(CPba);
						TotalConnectionsAdded++;
					}
				}
			}
		}
	}
	
	UE_LOG(LogNav3D, Verbose, TEXT("Built %d compact portals between %s and %s"), 
	       TotalConnectionsAdded, *ActorA->GetName(), *ActorB->GetName());
}

void FNav3DDataGenerator::StartTacticalGeneration()
{
	if (bTacticalGenerationInProgress)
	{
		UE_LOG(LogNav3D, Warning, TEXT("Tactical generation already in progress"));
		return;
	}
	
	if (!NavigationData.TacticalSettings.bEnableTacticalReasoning)
	{
		UE_LOG(LogNav3D, Warning, TEXT("Tactical reasoning is disabled"));
		return;
	}
	
	UE_LOG(LogNav3D, Log, TEXT("Starting tactical generation"));
	bTacticalGenerationInProgress = true;
	CurrentTacticalLayer = 0;
	
	// Start processing tactical generation
	ProcessTacticalGeneration();
}

void FNav3DDataGenerator::ProcessTacticalGeneration()
{
	if (!bTacticalGenerationInProgress)
	{
		return;
	}
	
	// Get all chunk actors
	const TArray<ANav3DDataChunkActor*> ChunkActors = NavigationData.GetChunkActors();
	if (ChunkActors.Num() == 0)
	{
		UE_LOG(LogNav3D, Warning, TEXT("No chunk actors available for tactical generation"));
		ResetTacticalGenerationFlag();
		return;
	}
	
	// Initialize tactical reasoning if needed
	if (!NavigationData.InitializeTacticalReasoning())
	{
		UE_LOG(LogNav3D, Error, TEXT("Failed to initialize tactical reasoning"));
		ResetTacticalGenerationFlag();
		return;
	}
	
	// Build tactical data for all chunks
	NavigationData.BuildTacticalData();
	
	UE_LOG(LogNav3D, Log, TEXT("Tactical generation completed"));
	ResetTacticalGenerationFlag();
}

float FNav3DDataGenerator::CalculateConnectionWeight(const ANav3DDataChunkActor* FromChunk, const ANav3DDataChunkActor* ToChunk)
{
	if (!FromChunk || !ToChunk)
	{
		return 1.0f;
	}
	
	// Calculate distance-based weight
	const float Distance = FVector::Dist(
		FromChunk->DataChunkActorBounds.GetCenter(),
		ToChunk->DataChunkActorBounds.GetCenter()
	);
	
	// Add size-based penalty for larger chunks (harder to navigate)
	const float SizePenalty = FromChunk->DataChunkActorBounds.GetSize().GetMax() * 0.1f;
	
	// Base weight of 1.0, with distance and size modifiers
	return 1.0f + (Distance * 0.01f) + SizePenalty;
}

FNav3DVoxelConnection FNav3DDataGenerator::CompactPortalToVoxelConnection(
	const FCompactPortal& CompactPortal,
	const FNav3DChunkAdjacency&,
	const int32 LocalVolumeIndex,
	const int32 RemoteVolumeIndex)
{
	FNav3DVoxelConnection Conn;
	Conn.Local = CompactPortal.Local;
	Conn.Remote = CompactPortal.Remote;
	Conn.LocalVolumeIndex = LocalVolumeIndex;
	Conn.RemoteVolumeIndex = RemoteVolumeIndex;
	Conn.LocalChunkIndex = 0;
	Conn.RemoteChunkIndex = 0;
	Conn.Distance = 0.0f;
	return Conn;
}

FNav3DActorPortal FNav3DDataGenerator::CompactPortalToActorPortal(
	const FCompactPortal& CompactPortal,
	const FNav3DChunkAdjacency& Adjacency,
	ANav3DDataChunkActor* FromActor,
	ANav3DDataChunkActor* ToActor)
{
	FNav3DActorPortal P;
	P.From = FromActor;
	P.To = ToActor;
	P.Connection = CompactPortalToVoxelConnection(CompactPortal, Adjacency, 0, 0);
	return P;
}

void FNav3DDataGenerator::ResetTacticalGenerationFlag()
{
	bTacticalGenerationInProgress = false;
	CurrentTacticalLayer = 0;
	
	// Clear any pending tactical generation timer
	if (TacticalGenerationTimerHandle.IsValid())
	{
		if (const UWorld* World = GetWorld())
		{
			World->GetTimerManager().ClearTimer(TacticalGenerationTimerHandle);
		}
		TacticalGenerationTimerHandle.Invalidate();
	}
}