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
class ANav3DDataChunkActor;
class ANav3DTacticalActor;
class UNav3DWorldSubsystem;
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

	// Deprecated: moved to GenerationSettings.MaxSimultaneousBoxGenerationJobsCount

	FNav3DVolumeDebugData GetDebugData() const;
	FNav3DDataGenerationSettings GetGenerationSettings() const;
	
	// Chunk actor management (single source of truth)
	void RegisterChunkActor(ANav3DDataChunkActor* ChunkActor);
	void UnregisterChunkActor(ANav3DDataChunkActor* ChunkActor);
	TArray<ANav3DDataChunkActor*> GetAllChunkActors() const;
	
	// Cleanup methods for invalid actors
	void CleanupInvalidChunkActors();
	int32 GetInvalidChunkActorCount() const;
	
	// Blueprint-callable cleanup method
	UFUNCTION(BlueprintCallable, meta=(DisplayName="Cleanup Invalid Chunk Actors", Category="Nav3D"))
	void CleanupInvalidChunkActorsBP();
	
	// Tactical actor management
	void RegisterTacticalActor(ANav3DTacticalActor* TacticalActor);
	void UnregisterTacticalActor(ANav3DTacticalActor* TacticalActor);
	void ClearAllTacticalActors();
	TArray<ANav3DTacticalActor*> GetAllTacticalActors() const;
	
	// Cleanup methods for invalid tactical actors
	void CleanupInvalidTacticalActors();
	int32 GetInvalidTacticalActorCount() const;
	
	// Blueprint-callable cleanup method for tactical actors
	UFUNCTION(BlueprintCallable, meta=(DisplayName="Cleanup Invalid Tactical Actors", Category="Nav3D"))
	void CleanupInvalidTacticalActorsBP();
	
	// Comprehensive cleanup method for both types of invalid actors
	UFUNCTION(BlueprintCallable, meta=(DisplayName="Cleanup All Invalid Actors", Category="Nav3D"))
	void CleanupAllInvalidActors();
	
	// Navigation data access (queries chunk actors)
	const FNav3DVolumeNavigationData* GetVolumeNavigationDataContainingPoint(const FVector& Point) const;
	const FNav3DVolumeNavigationData* GetVolumeNavigationDataContainingPoints(const TArray<FVector>& Points) const;
	
	// Bounds calculation (computed from chunk actors)
	FBox GetBoundingBox() const;
	
	// Navigation queries
	bool HasNavigationData() const { return ChunkActors.Num() > 0; }
	int32 GetChunkCount() const { return ChunkActors.Num(); }
	TArray<TObjectPtr<ANav3DDataChunkActor>> GetChunkActors() const { return ChunkActors; }
	
	// Volume information (read-only)
	UFUNCTION(BlueprintCallable, meta=(DisplayName="Show Partitioned Volumes", Category="Nav3D"))
	TArray<FBox> GetPartitionedVolumes() const;
	
	// Get all discoverable volumes from the world (built and unbuilt)
	TArray<FBox> GetAllDiscoverableVolumes() const;

	UFUNCTION(BlueprintCallable, meta=(DisplayName="Validate", Category="Nav3D"))
	void ValidateNavigationSystem();
	
	void ShowBuildStatus();

	UFUNCTION(BlueprintCallable, meta=(DisplayName="Analyse", Category="Nav3D"))
	void Analyse() const;

	// Build operations (moved to Build Controls section in details panel)
	void BuildNavigationData() const;
	void ClearNavigationData();
	void BuildSingleVolume(const FBox& VolumeBounds);

	// Build operations (chunk-scoped)
	void RebuildSingleChunk(const FBox& ChunkBounds);
	void RebuildSingleChunk(const class ANav3DDataChunkActor* ChunkActor);
	
	// Tactical build operations
	void RebuildTacticalData();

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
	virtual void PostLoad() override;
	virtual void OnRegistered() override;
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
	bool InitializeTacticalReasoning();
	void BuildTacticalData();
	const FNav3DTacticalData& GetTacticalDataAtPosition(const FVector& Position) const;

	// Get the voxel extent based on the agent radius from NavConfig
	float GetVoxelExtent() const;
	int32 GetLayerCount() const;
	UNav3DWorldSubsystem* GetSubsystem() const;

#if WITH_EDITORONLY_DATA
	// Editor-only: increments whenever chunks are added/removed so details panel can refresh
	UPROPERTY(Transient, VisibleAnywhere, Category="Nav3D")
	int32 ChunkRevision = 0;
#endif

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Navigation", meta=(AllowPrivateAccess="true"))
	TArray<TObjectPtr<ANav3DDataChunkActor>> ChunkActors;
	
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Tactical", meta=(AllowPrivateAccess="true"))
	TArray<TObjectPtr<ANav3DTacticalActor>> TacticalActors;
	
	// Spatial query caching (transient)
	mutable TWeakObjectPtr<UNav3DWorldSubsystem> CachedSubsystem;
	
	void CheckToDiscardSubLevelNavData(const UNavigationSystemBase& NavigationSystem);
	void RecreateDefaultFilter() const;
	void UpdateDrawing() const;
	void ResetGenerator(bool CancelBuild = true);
	void OnNavigationDataUpdatedInBounds(const TArray<FBox>& UpdatedBounds);
	void DiscoverExistingChunkActors();
	void NotifyChunksChanged();

	static void AnalyzeActualSpatialDistribution(const FBox& VolumeBounds, const TArray<FOverlapResult>& OverlappingObjects);
	static void AnalyzeSpatialClustering(const TArray<FVector>& ObjectPositions, const TArray<FBox>& ObjectBounds, const FBox& VolumeBounds, int32 NumCandidateObjects);
	static void EstimateOctreeSize(const FBox& VolumeBounds, float EmptyGridRatio, int32 MaxLayers, float LeafNodeSize);

	void InvalidateAffectedPaths(const TArray<FBox>& UpdatedBounds);
	void OnNavigationDataGenerationFinished();
	UNav3DDataChunk* GetNavigationDataChunk(ULevel* Level) const;
	static FBox CalculateLevelBounds(ULevel* Level);

	static FPathFindingResult FindPath(
		const FNavAgentProperties& NavAgentProperties,
		const FPathFindingQuery& PathFindingQuery);

	static FNav3DGenerationFinishedDelegate GenerationFinishedDelegate;

    // Empty tactical to return when tactical reasoning is not enabled
    static const FNav3DTacticalData EmptyTacticalData;

	// Track which volumes are currently loaded and their reference counts
	TMap<FBox, int32> LoadedVolumeReferenceCounts;  // Volume bounds → ref count
	mutable FCriticalSection VolumeLoadingMutex;    // Thread safety
};

FORCEINLINE FNav3DVolumeDebugData ANav3DData::GetDebugData() const
{
	return DebugData;
}

FORCEINLINE FNav3DDataGenerationSettings ANav3DData::GetGenerationSettings() const
{
	return GenerationSettings;
}