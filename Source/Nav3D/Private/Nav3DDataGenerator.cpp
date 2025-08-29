#include "Nav3DDataGenerator.h"
#include "Nav3DData.h"
#include <GameFramework/PlayerController.h>
#include <NavigationSystem.h>
#include "Nav3D.h"

FNav3DVolumeNavigationDataGenerator::FNav3DVolumeNavigationDataGenerator(
	FNav3DDataGenerator& NavigationDataGenerator, const FBox& VolumeBounds)
	: ParentGenerator(NavigationDataGenerator), BoundsNavigationData(),
	  VolumeBounds(VolumeBounds)
{
	NavDataConfig = NavigationDataGenerator.GetOwner()->GetConfig();
}

bool FNav3DVolumeNavigationDataGenerator::DoWork()
{
	FNav3DVolumeNavigationDataSettings GenerationSettings;
	GenerationSettings.GenerationSettings = ParentGenerator.GetGenerationSettings();
	GenerationSettings.World = ParentGenerator.GetWorld();
	GenerationSettings.VoxelExtent = NavDataConfig.AgentRadius * 2.0f;
	GenerationSettings.TacticalSettings = ParentGenerator.GetOwner()->TacticalSettings;
	BoundsNavigationData.GenerateNavigationData(VolumeBounds, GenerationSettings);
	
	return true;
}

TStatId FNav3DBoxGeneratorWrapper::GetStatId()
{
	RETURN_QUICK_DECLARE_CYCLE_STAT(FNav3DBoxGenerator, STATGROUP_ThreadPoolAsyncTasks);
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
		           NavigationData.MaxSimultaneousBoxGenerationJobsCount);
	UE_LOG(LogNav3D, Verbose,
	       TEXT("Using max of %d workers to build Nav3D navigation."),
	       MaximumGeneratorTaskCount);
}

bool FNav3DDataGenerator::RebuildAll()
{
	NavigationData.UpdateNavVersion();

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
	const bool HadTasks = GetNumRemaningBuildTasks() > 0;

	do
	{
		const int32 TasksToProcessCount =
			MaximumGeneratorTaskCount - RunningBoundsDataGenerationElements.Num();
		ProcessAsyncTasks(TasksToProcessCount);

		// Block until tasks are finished
		for (const auto& Element : RunningBoundsDataGenerationElements)
		{
			Element.AsyncTask->EnsureCompletion();
		}
	}
	while (GetNumRemaningBuildTasks() > 0);

	if (HadTasks)
	{
		NavigationData.RequestDrawingUpdate();
	}
}

void FNav3DDataGenerator::CancelBuild()
{
	PendingBoundsDataGenerationElements.Empty();

	for (auto& Element : RunningBoundsDataGenerationElements)
	{
		if (Element.AsyncTask)
		{
			Element.AsyncTask->EnsureCompletion();
			delete Element.AsyncTask;
			Element.AsyncTask = nullptr;
		}
	}

	RunningBoundsDataGenerationElements.Empty();
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
		MaximumGeneratorTaskCount - RunningTasksCount;

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

				NavigationData.RemoveDataInBounds(MatchingBoundsElement);
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
	if (const UNavigationSystemV1* NavigationSystem =
		FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld()))
	{
		if (!NavigationSystem->ShouldGenerateNavigationEverywhere())
		{
			FBox BoundsSum(ForceInit);
			{
				TArray<FBox> SupportedNavigationBounds;
				NavigationSystem->GetNavigationBoundsForNavData(
					NavigationData, SupportedNavigationBounds);

				RegisteredNavigationBounds.Reset(SupportedNavigationBounds.Num());

				for (const auto& Box : SupportedNavigationBounds)
				{
					RegisteredNavigationBounds.Add(Box);
					BoundsSum += Box;
				}
			}
			TotalNavigationBounds = BoundsSum;
		}
		else
		{
			RegisteredNavigationBounds.Reset(1);
			TotalNavigationBounds = NavigationSystem->GetWorldBounds();
			if (!TotalNavigationBounds.IsValid)
			{
				RegisteredNavigationBounds.Add(TotalNavigationBounds);
			}
		}
	}
	else
	{
		TotalNavigationBounds = FBox(ForceInit);
	}
}

TArray<FBox> FNav3DDataGenerator::ProcessAsyncTasks(const int32 TaskToProcessCount)
{
	const bool HasTasksAtStart = GetNumRemaningBuildTasks() > 0;

	int32 ProcessedTasksCount = 0;
	// Submit pending tile elements
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

		RunningElement.AsyncTask->StartBackgroundTask();

		RunningBoundsDataGenerationElements.Add(RunningElement);

		PendingBoundsDataGenerationElements.RemoveAt(ElementIndex, 1, EAllowShrinking::No);
		ProcessedTasksCount++;
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
		// QUICK_SCOPE_CYCLE_COUNTER(STAT_RecastNavMeshGenerator_ProcessTileTasks_FinishedTasks);

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

		NavigationData.AddVolumeNavigationData(
			BoxGenerator.GetBoundsNavigationData());

		FinishedBoxes.Emplace(MoveTemp(Element.VolumeBounds));

		delete Element.AsyncTask;
		Element.AsyncTask = nullptr;
		RunningBoundsDataGenerationElements.RemoveAtSwap(Index, 1, EAllowShrinking::No);
	}

	const bool HasTasksAtEnd = GetNumRemaningBuildTasks() > 0;
	if (HasTasksAtStart && !HasTasksAtEnd)
	{
		// QUICK_SCOPE_CYCLE_COUNTER(STAT_RecastNavMeshGenerator_OnNavMeshGenerationFinished);
		NavigationData.OnNavigationDataGenerationFinished();
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
