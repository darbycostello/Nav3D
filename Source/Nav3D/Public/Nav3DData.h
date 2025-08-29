// ReSharper disable CppRedefinitionOfDefaultArgumentInOverrideFunction
// ReSharper disable CppUEBlueprintCallableFunctionUnused
#pragma once

#include "Nav3DTypes.h"
#include "Nav3DVolumeNavigationData.h"
#include <CoreMinimal.h>
#include <NavigationData.h>
#include "Tactical/Nav3DTacticalReasoning.h"
#include "Nav3DData.generated.h"

class UNav3DDataChunk;
class UNav3DNavDataRenderingComponent;
struct FNav3DBounds;
class FNav3DTacticalReasoning;
struct FNav3DTacticalData;

DECLARE_MULTICAST_DELEGATE_OneParam(FNav3DGenerationFinishedDelegate, ANav3DData*);

UCLASS(meta=(DisplayName="Nav3D Data"))
class NAV3D_API ANav3DData final : public ANavigationData
{
	GENERATED_BODY()

public:
	ANav3DData();
	virtual ~ANav3DData() override;

	friend class FNav3DDataGenerator;

	friend class FNav3DTacticalReasoning;
	TUniquePtr<FNav3DTacticalReasoning> TacticalReasoning;

	UPROPERTY(EditAnywhere, Category = "Nav3D")
	FNav3DTacticalSettings TacticalSettings;

	UPROPERTY(EditInstanceOnly, Category="Nav3D")
	FNav3DVolumeDebugData DebugData;

	UPROPERTY(EditAnywhere, Category="Nav3D", config)
	FNav3DDataGenerationSettings GenerationSettings;

	UPROPERTY(EditAnywhere, Category="Nav3D", config, meta = (ClampMin = "0", UIMin = "0"), AdvancedDisplay)
	int32 MaxSimultaneousBoxGenerationJobsCount;

	FNav3DVolumeDebugData GetDebugData() const;
	FNav3DDataGenerationSettings GetGenerationSettings() const;
	const TArray<FNav3DVolumeNavigationData>& GetVolumeNavigationData() const;
	TArray<FNav3DVolumeNavigationData>& GetVolumeNavigationData();

	UFUNCTION(BlueprintCallable, Category = "Nav3D")
	bool FindBestLocation(
		const FVector& StartPosition,
		const TArray<FVector>& ObserverPositions,
		TArray<FPositionCandidate>& OutCandidatePositions,
		const ETacticalVisibility Visibility,
		const ETacticalDistance DistancePreference,
		const ETacticalRegion RegionPreference,
		bool bForceNewRegion,
		bool bUseRaycasting) const;
    
	void RebuildDirtyBounds(const TArray<FBox>& DirtyBounds);
	void RegisterDynamicOccluder(const AActor* Occluder);
	void UnregisterDynamicOccluder(const AActor* Occluder);
	virtual void PostInitProperties() override;
	virtual void OnRegistered() override;
	virtual void PostLoad() override;
	virtual void Serialize(FArchive& Archive) override;
	virtual void CleanUp() override;
	virtual bool NeedsRebuild() const override;
	virtual void EnsureBuildCompletion() override;
	virtual bool SupportsRuntimeGeneration() const override;
	virtual bool SupportsStreaming() const override;
	virtual FNavLocation GetRandomPoint(FSharedConstNavQueryFilter Filter, const UObject* Querier) const override;
	virtual bool GetRandomReachablePointInRadius(
		const FVector& Origin, float Radius, FNavLocation& OutResult,
		FSharedConstNavQueryFilter Filter = nullptr,
		const UObject* Querier = nullptr) const override;
	virtual bool GetRandomPointInNavigableRadius(
		const FVector& Origin, float Radius, FNavLocation& OutResult,
		FSharedConstNavQueryFilter Filter = nullptr,
		const UObject* Querier = nullptr) const override;
	virtual void BatchRaycast(TArray<FNavigationRaycastWork>& Workload,
	                          FSharedConstNavQueryFilter Filter,
	                          const UObject* Querier = nullptr) const override;
	virtual bool FindMoveAlongSurface(const FNavLocation& StartLocation,
	                     const FVector& TargetPosition, FNavLocation& OutLocation,
	                     FSharedConstNavQueryFilter Filter = nullptr,
	                     const UObject* Querier = nullptr) const override;
	virtual bool ProjectPoint(const FVector& Point, FNavLocation& OutLocation,
	                          const FVector& Extent,
	                          FSharedConstNavQueryFilter Filter = nullptr,
	                          const UObject* Querier = nullptr) const override;
	virtual void BatchProjectPoints(TArray<FNavigationProjectionWork>& Workload,
	                   const FVector& Extent,
	                   FSharedConstNavQueryFilter Filter = nullptr,
	                   const UObject* Querier = nullptr) const override;
	virtual void BatchProjectPoints(TArray<FNavigationProjectionWork>& Workload,
	                   FSharedConstNavQueryFilter Filter = nullptr,
	                   const UObject* Querier = nullptr) const override;
	virtual ENavigationQueryResult::Type CalcPathCost(const FVector& PathStart, const FVector& PathEnd,
	             FVector::FReal& OutPathCost,
	             FSharedConstNavQueryFilter Filter = nullptr,
	             const UObject* Querier = nullptr) const override;
	virtual ENavigationQueryResult::Type CalcPathLength(const FVector& PathStart, const FVector& PathEnd,
	               FVector::FReal& OutPathLength,
	               FSharedConstNavQueryFilter Filter = nullptr,
	               const UObject* Querier = nullptr) const override;
	virtual ENavigationQueryResult::Type CalcPathLengthAndCost(const FVector& PathStart, const FVector& PathEnd,
	                      FVector::FReal& OutPathLength,
	                      FVector::FReal& OutPathCost,
	                      FSharedConstNavQueryFilter Filter = nullptr,
	                      const UObject* Querier = nullptr) const override;
	virtual bool DoesNodeContainLocation(NavNodeRef NodeRef,
	                        const FVector& WorldSpaceLocation) const override;
	virtual UPrimitiveComponent* ConstructRenderingComponent() override;
	virtual void OnStreamingLevelAdded(ULevel* Level, UWorld* World) override;
	virtual void OnStreamingLevelRemoved(ULevel* Level, UWorld* World) override;
	virtual void OnNavAreaChanged() override;
	virtual void OnNavAreaAdded(const UClass* NavAreaClass,
	                            int32 AgentIndex) override;
	virtual int32 GetNewAreaID(const UClass* NavAreaClass) const override;
	virtual int32 GetMaxSupportedAreas() const override;
	virtual bool IsNodeRefValid(NavNodeRef NodeRef) const override;
	virtual void TickActor(float DeltaTime, ELevelTick TickType,
	                       FActorTickFunction& ThisTickFunction) override;

#if WITH_EDITOR
	static bool NeedsTacticalRebuild(const FPropertyChangedEvent& PropertyChangedEvent);
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
	virtual bool ShouldExport() override;
#endif

#if !UE_BUILD_SHIPPING
	virtual uint32 LogMemUsed() const override;
#endif

	virtual void ConditionalConstructGenerator() override;
	void RequestDrawingUpdate(bool Force = false);
	FBox GetBoundingBox() const;
	void RemoveDataInBounds(const FBox& Bounds);

	template <typename Allocator_Type_>
	void RemoveDataInBounds(const TArray<FBox, Allocator_Type_>& BoundsArray)
	{
		for (const auto& Bounds : BoundsArray)
		{
			RemoveDataInBounds(Bounds);
		}
	}

	void AddVolumeNavigationData(FNav3DVolumeNavigationData Data);
	const FNav3DVolumeNavigationData*
	GetVolumeNavigationDataContainingPoints(const TArray<FVector>& Points) const;
	void UpdateNavVersion();
	bool InitializeTacticalReasoning();
	void BuildTacticalData();
	const FNav3DTacticalData& GetTacticalDataAtPosition(const FVector& Position) const;
	const FNav3DVolumeNavigationData* GetVolumeNavigationDataContainingPoint(const FVector& Point) const;

	// Get the voxel extent based on the agent radius from NavConfig
	float GetVoxelExtent() const;
	int32 GetLayerCount() const;

protected:
	float TimeSinceLastUpdate;

private:
	void SerializeNav3DData(FArchive& Archive, ENav3DVersion Version);
	void
	CheckToDiscardSubLevelNavData(const UNavigationSystemBase& NavigationSystem);
	void RecreateDefaultFilter() const;
	void UpdateDrawing() const;
	void ResetGenerator(bool CancelBuild = true);
	void OnNavigationDataUpdatedInBounds(const TArray<FBox>& UpdatedBounds);
	
	UFUNCTION(CallInEditor, meta=(DisplayName="Clear", Category="Nav3D"))
	void ClearNavigationData();

	UFUNCTION(CallInEditor, meta=(DisplayName="Build", Category="Nav3D"))
	void BuildNavigationData() const;

	void InvalidateAffectedPaths(const TArray<FBox>& UpdatedBounds);
	void OnNavigationDataGenerationFinished();
	UNav3DDataChunk* GetNavigationDataChunk(ULevel* Level) const;

	static FPathFindingResult FindPath(
		const FNavAgentProperties& NavAgentProperties,
		const FPathFindingQuery& PathFindingQuery);

	static FNav3DGenerationFinishedDelegate GenerationFinishedDelegate;

	TArray<FNav3DVolumeNavigationData> VolumeNavigationData;
	ENav3DVersion Version;
    
    // Empty tactical to return when tactical reasoning is not enabled
    static const FNav3DTacticalData EmptyTacticalData;
};

FORCEINLINE const TArray<FNav3DVolumeNavigationData>& ANav3DData::GetVolumeNavigationData() const
{
	return VolumeNavigationData;
}

FORCEINLINE TArray<FNav3DVolumeNavigationData>& ANav3DData::GetVolumeNavigationData() 
{
	return VolumeNavigationData;
}

FORCEINLINE FNav3DVolumeDebugData ANav3DData::GetDebugData() const
{
	return DebugData;
}

FORCEINLINE FNav3DDataGenerationSettings ANav3DData::GetGenerationSettings() const
{
	return GenerationSettings;
}