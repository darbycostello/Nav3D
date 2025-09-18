#pragma once

#include "Nav3DData.h"
#include "Nav3DTypes.h"
#include <AI/NavDataGenerator.h>
#include "TimerManager.h"

class ANav3DData;
class FNav3DDataGenerator;

struct FNav3DVolumeNavigationDataGenerator final : FNoncopyable
{
	FNav3DVolumeNavigationDataGenerator(
		FNav3DDataGenerator& NavigationDataGenerator, const FBox& VolumeBounds);

	FNav3DVolumeNavigationData GetBoundsNavigationData() const;

	bool DoWork();

private:
	FNav3DDataGenerator& ParentGenerator;
	FNav3DVolumeNavigationData BoundsNavigationData;
	FBox VolumeBounds;
	TWeakObjectPtr<UWorld> World;
	FNavDataConfig NavDataConfig;
};

FORCEINLINE FNav3DVolumeNavigationData
FNav3DVolumeNavigationDataGenerator::GetBoundsNavigationData() const
{
	return BoundsNavigationData;
}

struct NAV3D_API FNav3DBoxGeneratorWrapper : FNonAbandonableTask
{
	TSharedRef<FNav3DVolumeNavigationDataGenerator> BoxNavigationDataGenerator;

	FNav3DBoxGeneratorWrapper(
		const TSharedRef<FNav3DVolumeNavigationDataGenerator>
		& BoxNavigationGenerator)
		: BoxNavigationDataGenerator(BoxNavigationGenerator)
	{
	}

	void DoWork() const { BoxNavigationDataGenerator->DoWork(); }

	FORCEINLINE static TStatId GetStatId() { return TStatId(); }
};

using FNav3DBoxGeneratorTask = FAsyncTask<FNav3DBoxGeneratorWrapper>;

struct FPendingBoundsDataGenerationElement
{
	FBox VolumeBounds;
	float SeedDistance;

	FPendingBoundsDataGenerationElement()
		: VolumeBounds(ForceInit), SeedDistance(MAX_flt)
	{
	}

	bool operator==(const FBox& OtherBox) const
	{
		return VolumeBounds == OtherBox;
	}

	bool operator==(const FPendingBoundsDataGenerationElement& Other) const
	{
		return VolumeBounds == Other.VolumeBounds;
	}

	bool operator<(const FPendingBoundsDataGenerationElement& Other) const
	{
		return Other.SeedDistance < SeedDistance;
	}

	friend uint32
	GetTypeHash(const FPendingBoundsDataGenerationElement& Element)
	{
		return HashCombine(GetTypeHash(Element.VolumeBounds.GetCenter()),
		                   GetTypeHash(Element.VolumeBounds.GetExtent()));
	}
};

struct FRunningBoundsDataGenerationElement
{
	FRunningBoundsDataGenerationElement()
		: VolumeBounds(ForceInit), ShouldDiscard(false), AsyncTask(nullptr)
	{
	}

	FRunningBoundsDataGenerationElement(const FBox& VolumeBounds)
		: VolumeBounds(VolumeBounds), ShouldDiscard(false), AsyncTask(nullptr)
	{
	}

	bool operator==(const FRunningBoundsDataGenerationElement& Other) const
	{
		return VolumeBounds == Other.VolumeBounds;
	}

	FBox VolumeBounds;
	/** whether generated results should be discarded */
	bool ShouldDiscard;
	FNav3DBoxGeneratorTask* AsyncTask;
};

class NAV3D_API FNav3DDataGenerator final : public FNavDataGenerator, public FNoncopyable
{
public:
	explicit FNav3DDataGenerator(ANav3DData& NavigationData);

	ANav3DData* GetOwner() const;
	UWorld* GetWorld() const;
	const FNav3DDataGenerationSettings& GetGenerationSettings() const;
	TSharedRef<FNav3DVolumeNavigationDataGenerator> CreateBoxNavigationGenerator(const FBox& Box);

	// Enhanced build management
	ANav3DDataChunkActor* CreateChunkActorForVolume(
		const FBox& VolumeBounds, const FNav3DVolumeNavigationData& NavData) const;
	void BuildAdjacencyBetweenChunkActors(const TArray<ANav3DDataChunkActor*>& ChunkActors) const;
	static void BuildAdjacencyBetweenTwoChunkActors(
		ANav3DDataChunkActor* ActorA, ANav3DDataChunkActor* ActorB, float VoxelSize);

	// Single volume build support
	void SetBuildTargetVolume(const FBox& VolumeBounds);
	void RestoreAllVolumes();
	bool IsSingleVolumeBuild() const { return bIsSingleVolumeBuild; }
	
	void Init();

	virtual bool RebuildAll() override;
	virtual void EnsureBuildCompletion() override;
	virtual void CancelBuild() override;
	virtual void TickAsyncBuild(float DeltaSeconds) override;
	virtual void OnNavigationBoundsChanged() override;
	virtual void RebuildDirtyAreas(const TArray<FNavigationDirtyArea>& DirtyAreas) override;
	virtual bool IsBuildInProgressCheckDirty() const override;
	virtual int32 GetNumRemaningBuildTasks() const override;
	virtual int32 GetNumRunningBuildTasks() const override;

	// Exposed for cooperative cancellation checks in nested work if needed
	static bool ShouldCancelBuild() { return false; }
	
	// Tactical generation support
	void StartTacticalGeneration();
	void ProcessTacticalGeneration();
	void ResetTacticalGenerationFlag();
	bool IsTacticalGenerationInProgress() const { return bTacticalGenerationInProgress; }
	
	// Adjacency helper functions
	static float CalculateConnectionWeight(const ANav3DDataChunkActor* FromChunk, const ANav3DDataChunkActor* ToChunk);

	// Compact portal conversion helpers
	static FNav3DVoxelConnection CompactPortalToVoxelConnection(
		const FCompactPortal& CompactPortal,
		const FNav3DChunkAdjacency& Adjacency,
		int32 LocalVolumeIndex = 0,
		int32 RemoteVolumeIndex = 0);

	static FNav3DActorPortal CompactPortalToActorPortal(
		const FCompactPortal& CompactPortal,
		const FNav3DChunkAdjacency& Adjacency,
		ANav3DDataChunkActor* FromActor,
		ANav3DDataChunkActor* ToActor);

	TNavStatArray<FPendingBoundsDataGenerationElement> GetPendingBoundsDataGenerationElements()
	{
		return PendingBoundsDataGenerationElements;
	};

private:
	void StartChunkedBuildCompletion();
	void ProcessBuildChunk();

	static void GetSeedLocations(TArray<FVector2D>& SeedLocations,
	                             const UWorld& World);
	void SortPendingBounds();
	void UpdateNavigationBounds();
	TArray<FBox> ProcessAsyncTasks(int32 TaskToProcessCount);
	
	// Volume partitioning support
	TArray<FBox> GetOriginalNavigationBounds() const;
	static TArray<FBox> PartitionVolumeIfNeeded(const FBox& OriginalVolume);
	static void ValidatePartitionedVolumes(const TArray<FBox>& Volumes);
	
	ANav3DData& NavigationData;
	FNav3DDataGenerationSettings GenerationSettings;
	int MaximumGeneratorTaskCount;
	uint8 IsInitialized : 1;
	
	// Single volume build support
	TNavStatArray<FBox> OriginalNavigationBounds;
	bool bIsSingleVolumeBuild = false;

	/** Total bounding box that includes all volumes, in unreal units. */
	FBox TotalNavigationBounds;

	TNavStatArray<FBox> RegisteredNavigationBounds;
	TNavStatArray<FPendingBoundsDataGenerationElement> PendingBoundsDataGenerationElements;
	TNavStatArray<FRunningBoundsDataGenerationElement> RunningBoundsDataGenerationElements;

	FTimerHandle ChunkedBuildTimerHandle;
	
	// Tactical generation state
	bool bTacticalGenerationInProgress = false;
	FTimerHandle TacticalGenerationTimerHandle;
	int32 CurrentTacticalLayer = 0;
	TArray<ANav3DDataChunkActor*> TacticalChunkActors;
};

FORCEINLINE ANav3DData* FNav3DDataGenerator::GetOwner() const
{
	return &NavigationData;
}

FORCEINLINE UWorld* FNav3DDataGenerator::GetWorld() const
{
	return NavigationData.GetWorld();
}

FORCEINLINE const FNav3DDataGenerationSettings&
FNav3DDataGenerator::GetGenerationSettings() const
{
	return GenerationSettings;
}
