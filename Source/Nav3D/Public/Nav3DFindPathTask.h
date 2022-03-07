#pragma once

#include "Nav3DComponent.h"
#include "Nav3DStructs.h"
#include "Async/Async.h"
#include "Async/AsyncWork.h"

class FNav3DFindPathTask : public FNonAbandonableTask
{
	friend class FAutoDeleteAsyncTask<FNav3DFindPathTask>;

public:
	FNav3DFindPathTask(
		UNav3DComponent* Nav3DComponent,
		const FNav3DOctreeEdge StartEdge,
		const FNav3DOctreeEdge TargetEdge,
		const FVector& StartLocation,
		const FVector& TargetLocation,
		FNav3DPathFindingConfig& Config,
        FNav3DPathSharedPtr& Path,
		const FFindPathTaskCompleteDynamicDelegate Complete) :
	
		Nav3DComponent(Nav3DComponent),
		StartEdge(StartEdge),
		TargetEdge(TargetEdge),
		StartLocation(StartLocation),
		TargetLocation(TargetLocation),
		Config(Config),
		Path(Path),
		TaskComplete(Complete)
	{}

protected:
	TWeakObjectPtr<UNav3DComponent> Nav3DComponent;
	FNav3DOctreeEdge StartEdge;
	FNav3DOctreeEdge TargetEdge;
	FVector StartLocation;
	FVector TargetLocation;
	FNav3DPathFindingConfig Config;
    FNav3DPathSharedPtr Path;
	FFindPathTaskCompleteDynamicDelegate TaskComplete;

	void DoWork() const {
		// Capture the local variables
		TWeakObjectPtr<UNav3DComponent> Comp = Nav3DComponent;
		FNav3DPathSharedPtr PathPtr = Path;
		FNav3DPathFindingConfig ConfigCopy = Config;
		FFindPathTaskCompleteDynamicDelegate TaskCompleteCopy = TaskComplete;
		if (!Comp.IsValid())
		{
			UE_LOG(LogTemp, Error, TEXT("Invalid UNav3DComponent, FNav3DFindPathTask abort"));
			return;
		}

		Nav3DComponent->ExecutePathFinding(StartEdge, TargetEdge, StartLocation, TargetLocation, Config, *Path.Get());
		Nav3DComponent->AddPathStartLocation(*Path.Get());

		
		// Run the path pruning, smoothing and debug draw back on the game thread
		AsyncTask(ENamedThreads::GameThread, [Comp, PathPtr, ConfigCopy, TaskCompleteCopy]() {

            FNav3DPath& PathRef = *PathPtr.Get();

            Comp->ApplyPathPruning(PathRef, ConfigCopy);
            Comp->ApplyPathSmoothing(PathRef, ConfigCopy);

#if WITH_EDITOR
            Comp->RequestNavPathDebugDraw(PathRef);
#endif

            TaskCompleteCopy.Execute(PathRef.Points.Num() > 0);
		});
	}

	FORCEINLINE TStatId GetStatId() const
	{
		RETURN_QUICK_DECLARE_CYCLE_STAT(FNav3DFindPathTask, STATGROUP_ThreadPoolAsyncTasks);
	}
};