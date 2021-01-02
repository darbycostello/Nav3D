#pragma once

#include "Nav3DVolume.h"
#include "Async/Async.h"
#include "Async/AsyncWork.h"

class FNav3DUpdateOctreeTask : public FNonAbandonableTask
{
	friend class FAutoDeleteAsyncTask<FNav3DUpdateOctreeTask>;

public:
	FNav3DUpdateOctreeTask(
		ANav3DVolume* Volume,
		const FNav3DUpdateOctreeDelegate Complete) :
		Volume(Volume),
		TaskComplete(Complete){}

protected:
	ANav3DVolume* Volume;
	FNav3DUpdateOctreeDelegate TaskComplete;

	void DoWork() const {
		Volume->UpdateOctree();
		TaskComplete.Execute();
	}

	FORCEINLINE TStatId GetStatId() const
	{
		RETURN_QUICK_DECLARE_CYCLE_STAT(FNav3DUpdateOctreeTask, STATGROUP_ThreadPoolAsyncTasks);
	}
};