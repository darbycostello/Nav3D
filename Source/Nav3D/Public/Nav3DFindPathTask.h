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
		FNav3DPath& Path,
		const FFindPathTaskCompleteDynamicDelegate Complete) :
	
		Nav3DComponent(Nav3DComponent),
		Start(StartEdge),
		Target(TargetEdge),
		StartLocation(StartLocation),
		TargetLocation(TargetLocation),
		Config(Config),
		Path(Path),
		TaskComplete(Complete)
	{}

protected:
	UNav3DComponent* Nav3DComponent;
	FNav3DOctreeEdge Start;
	FNav3DOctreeEdge Target;
	FVector StartLocation;
	FVector TargetLocation;
	FNav3DPathFindingConfig Config;
	FNav3DPath& Path;
	FFindPathTaskCompleteDynamicDelegate TaskComplete;

	void DoWork() const {
		Nav3DComponent->ExecutePathFinding(Start, Target, StartLocation, TargetLocation, Config, Path);
		

		// Run the debug draw back on the game thread
		AsyncTask(ENamedThreads::GameThread, [=]() {
			Nav3DComponent->ApplyPathPruning(Path, Config);
			Nav3DComponent->ApplyPathSmoothing(Path, Config);
			Nav3DComponent->DebugDrawNavPath(Path);	
		});

		
		TaskComplete.Execute(Path.Points.Num() > 0);
	}

	FORCEINLINE TStatId GetStatId() const
	{
		RETURN_QUICK_DECLARE_CYCLE_STAT(FNav3DFindPathTask, STATGROUP_ThreadPoolAsyncTasks);
	}
};