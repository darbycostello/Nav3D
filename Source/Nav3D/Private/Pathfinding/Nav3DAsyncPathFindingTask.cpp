#include "Pathfinding/Nav3DAsyncPathFindingTask.h"

#include "Nav3D.h"
#include "Nav3DData.h"
#include "GameFramework/Actor.h"
#include "NavigationSystem.h"
#include "GameFramework/NavMovementComponent.h"
#include "UObject/WeakObjectPtrTemplates.h"
#include "NavFilters/NavigationQueryFilter.h"
#include "Pathfinding/Nav3DPathFinder.h"

class NAV3D_API FNav3DAsyncPathFindingTask : public FNonAbandonableTask
{
	friend class FAutoDeleteAsyncTask<FNav3DAsyncPathFindingTask>;

public:
	FNav3DAsyncPathFindingTask(
		const FVector& StartLoc,
		const FVector& EndLoc,
		const ANav3DData* NavData,
		const FNavAgentProperties& AgentProps,
		const FSharedConstNavQueryFilter& QueryFilter,
		TWeakObjectPtr<UNav3DAsyncPathfindingTask> InTaskOwner,
		const TSharedPtr<FNav3DPath>& InPathResult) :
		StartLocation(StartLoc),
		EndLocation(EndLoc),
		Nav3DData(NavData),
		NavAgentProperties(AgentProps),
		Filter(QueryFilter),
		TaskOwner(InTaskOwner),
		PathResult(InPathResult)
	{
	}

protected:
	void DoWork() const
	{
		UE_LOG(LogNav3D, Verbose, TEXT("Nav3D background task DoWork started"));

		if (!Nav3DData.IsValid() || !PathResult.IsValid())
		{
			UE_LOG(LogNav3D, Error, TEXT("Nav3D background task failed - Invalid data"));
			BroadcastResultOnGameThread(false);
			return;
		}

		UE_LOG(LogNav3D, Verbose, TEXT("Starting pathfinding - Start: %s, End: %s"),
		       *StartLocation.ToString(), *EndLocation.ToString());

		// Perform pathfinding directly into shared path object
		const ENavigationQueryResult::Type Result = FNav3DPathFinder::GetPath(
			*PathResult.Get(),
			*Nav3DData.Get(),
			StartLocation,
			EndLocation,
			NavAgentProperties,
			Filter);

		const bool bSuccess = (Result == ENavigationQueryResult::Success);

		UE_LOG(LogNav3D, Verbose, TEXT("Pathfinding completed - Success: %d, Result: %d"),
		       bSuccess, static_cast<int32>(Result));

		if (!bSuccess)
		{
			UE_LOG(LogNav3D, Warning, TEXT("Pathfinding failed"));
		}
		else
		{
			UE_LOG(LogNav3D, Verbose, TEXT("Path points: %d"), PathResult->GetPathPoints().Num());
		}

		BroadcastResultOnGameThread(bSuccess);
	}

	void BroadcastResultOnGameThread(bool bSuccess) const
	{
		AsyncTask(ENamedThreads::GameThread, [WeakTaskOwner=TaskOwner, PathResult=PathResult, bSuccess]()
		{
			if (!WeakTaskOwner.IsValid())
			{
				if (!WeakTaskOwner.IsValid() || !PathResult.IsValid())
				{
					UE_LOG(LogNav3D, Warning, TEXT("Could not broadcast results - Invalid task or path"));
					return;
				}
			}

			const FNav3DPathData PathData = PathResult->CreatePathData();
			WeakTaskOwner->OnComplete.Broadcast(bSuccess, PathData);
			WeakTaskOwner->EndTask();
		});
	}

	static FORCEINLINE TStatId GetStatId()
	{
		RETURN_QUICK_DECLARE_CYCLE_STAT(FNav3DAsyncPathFindingTaskImpl, STATGROUP_ThreadPoolAsyncTasks);
	}

private:
	FVector StartLocation;
	FVector EndLocation;
	TWeakObjectPtr<const ANav3DData> Nav3DData;
	FNavAgentProperties NavAgentProperties;
	FSharedConstNavQueryFilter Filter;
	TWeakObjectPtr<UNav3DAsyncPathfindingTask> TaskOwner;
	TSharedPtr<FNav3DPath> PathResult;
};

UNav3DAsyncPathfindingTask* UNav3DAsyncPathfindingTask::FindPathAsync(
	UObject* WorldContextObject,
	const FVector StartLoc,
	const FVector EndLoc,
	AActor* PathfindingContext,
	TSubclassOf<UNavigationQueryFilter> FilterClass)
{
	UWorld* World = nullptr;

	if (WorldContextObject != nullptr)
	{
		World = GEngine->GetWorldFromContextObject(WorldContextObject, EGetWorldErrorMode::LogAndReturnNull);
	}
	if (World == nullptr && PathfindingContext != nullptr)
	{
		World = GEngine->GetWorldFromContextObject(PathfindingContext, EGetWorldErrorMode::LogAndReturnNull);
	}

	if (!World)
	{
		UE_LOG(LogNav3D, Error, TEXT("FindPathAsync failed - No valid World from any source"));
		return nullptr;
	}

	UNav3DAsyncPathfindingTask* Task = NewObject<UNav3DAsyncPathfindingTask>(World);
	if (!Task)
	{
		UE_LOG(LogNav3D, Error, TEXT("Failed to create Nav3D async task object"));
		return nullptr;
	}

	// Store movement component reference
	if (PathfindingContext)
	{
		if (UNavMovementComponent* MovementComp = PathfindingContext->FindComponentByClass<UNavMovementComponent>())
		{
			Task->MovementComponent = MovementComp;
			UE_LOG(LogNav3D, Verbose, TEXT("Found movement component %s"), *MovementComp->GetName());
		}
		else
		{
			UE_LOG(LogNav3D, Warning, TEXT("No NavMovementComponent found on PathfindingContext"));
		}
	}

	Task->StartLocation = StartLoc;
	Task->EndLocation = EndLoc;
	Task->QueryFilterClass = FilterClass;
	Task->WorldPtr = World;

	UE_LOG(LogNav3D, Verbose, TEXT("Created Nav3D task: %s"), *GetNameSafe(Task));
	if (FilterClass)
	{
		UE_LOG(LogNav3D, Verbose, TEXT("Using filter class: %s"), *GetNameSafe(FilterClass));
	}

	Task->Activate();
	return Task;
}

void UNav3DAsyncPathfindingTask::Activate()
{
	UE_LOG(LogNav3D, Verbose, TEXT("Nav3D task Activate() called"));

	UWorld* World = WorldPtr.Get();
	if (!World)
	{
		UE_LOG(LogNav3D, Error, TEXT("Invalid world"));
		OnComplete.Broadcast(false, FNav3DPathData());
		EndTask();
		return;
	}

	const UNavigationSystemV1* NavSys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(World);
	if (!NavSys)
	{
		UE_LOG(LogNav3D, Error, TEXT("Invalid navigation system"));
		OnComplete.Broadcast(false, FNav3DPathData());
		EndTask();
		return;
	}

	if (!MovementComponent.IsValid())
	{
		UE_LOG(LogNav3D, Error, TEXT("Invalid movement component"));
		OnComplete.Broadcast(false, FNav3DPathData());
		EndTask();
		return;
	}

	// Set the nav agent properties according to the task owner's movement component.
	const FNavAgentProperties& NavAgentProps = MovementComponent->GetNavAgentPropertiesRef();
	UE_LOG(LogNav3D, Verbose, TEXT("Using Nav Agent Properties from %s - Radius: %f, Height: %f"),
	       *MovementComponent->GetName(), NavAgentProps.AgentRadius, NavAgentProps.AgentHeight);

	const ANavigationData* NavigationData = NavSys->GetNavDataForProps(NavAgentProps);

	const ANav3DData* Nav3DData = Cast<ANav3DData>(NavigationData);
	if (!Nav3DData)
	{
		UE_LOG(LogNav3D, Error, TEXT("Invalid Nav3DData cast from navigation data"));
		OnComplete.Broadcast(false, FNav3DPathData());
		EndTask();
		return;
	}

	// Create query filter
	FSharedConstNavQueryFilter QueryFilter;
	if (QueryFilterClass)
	{
		QueryFilter = UNavigationQueryFilter::GetQueryFilter(*Nav3DData, nullptr, QueryFilterClass);
	}
	else
	{
		QueryFilter = Nav3DData->GetDefaultQueryFilter();
	}

	if (!QueryFilter.IsValid())
	{
		UE_LOG(LogNav3D, Error, TEXT("Invalid query filter"));
		OnComplete.Broadcast(false, FNav3DPathData());
		EndTask();
		return;
	}

	// Create shared path result that will be populated by async task
	PathResult = MakeShared<FNav3DPath>();

	// Start async task with weak ptr to self and shared path
	(new FAutoDeleteAsyncTask<FNav3DAsyncPathFindingTask>(
		StartLocation,
		EndLocation,
		Nav3DData,
		NavAgentProps,
		QueryFilter,
		this,
		PathResult))->StartBackgroundTask();
}
