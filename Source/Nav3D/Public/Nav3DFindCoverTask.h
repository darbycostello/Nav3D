#pragma once

#include "Nav3DComponent.h"
#include "Nav3DStructs.h"
#include "Async/Async.h"
#include "Async/AsyncWork.h"

class FNav3DFindCoverTask : public FNonAbandonableTask
{
	friend class FAutoDeleteAsyncTask<FNav3DFindCoverTask>;

public:
	FNav3DFindCoverTask(
		UNav3DComponent* Nav3DComponent,
		const FVector Location,
		const float Radius,
		const TArray<AActor*> Opponents,
		const TArray<TEnumAsByte<EObjectTypeQuery>> ObjectTypes,
		const ENav3DCoverSearchType SearchType,
		const bool bPerformLineTraces,
		FNav3DCoverLocation& CoverLocation,
		const FFindCoverTaskCompleteDynamicDelegate Complete) :
	
		Nav3DComponent(Nav3DComponent),
		Location(Location),
		Radius(Radius),
		Opponents(Opponents),
		ObjectTypes(ObjectTypes),
		SearchType(SearchType),
		bPerformLineTraces(bPerformLineTraces),
		CoverLocation(CoverLocation),
		TaskComplete(Complete) {}

protected:
	UNav3DComponent* Nav3DComponent;
	FVector Location;
    float Radius;
    TArray<AActor*> Opponents;
	TArray<TEnumAsByte<EObjectTypeQuery>> ObjectTypes;
	ENav3DCoverSearchType SearchType;
	bool bPerformLineTraces;
    FNav3DCoverLocation& CoverLocation;
	FFindCoverTaskCompleteDynamicDelegate TaskComplete;

	void DoWork() const {
		Nav3DComponent->ExecuteFindCover(Location, Radius, Opponents, ObjectTypes, SearchType, bPerformLineTraces, CoverLocation);

		AsyncTask(ENamedThreads::GameThread, [=, this]() {

#if WITH_EDITOR
			if (CoverLocation.Actor && CoverLocation.Location != FVector::ZeroVector) {
				Nav3DComponent->RequestNavCoverLocationDebugDraw(CoverLocation);	
			}
#endif

		});

		TaskComplete.Execute(CoverLocation.Location != FVector::ZeroVector);
	}

	FORCEINLINE TStatId GetStatId() const
	{
		RETURN_QUICK_DECLARE_CYCLE_STAT(FNav3DFindCoverTask, STATGROUP_ThreadPoolAsyncTasks);
	}
};