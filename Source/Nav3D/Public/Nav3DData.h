// ReSharper disable CppRedefinitionOfDefaultArgumentInOverrideFunction
// ReSharper disable CppUEBlueprintCallableFunctionUnused
#pragma once

#include "Nav3DTypes.h"
#include "Nav3DVolumeNavigationData.h"
#include <CoreMinimal.h>
#include <NavigationData.h>

#include "Tactical/Nav3DTacticalReasoning.h"
#include "Nav3DData.generated.h"
USTRUCT()
struct FRegionMapping
{
    GENERATED_BODY()
    
    uint16 VolumeID;
    uint8 LocalRegionIndex;
    TWeakObjectPtr<ANav3DDataChunkActor> ChunkActor;
    
    FRegionMapping()
        : VolumeID(0), LocalRegionIndex(0), ChunkActor(nullptr)
    {}
};

class UNav3DDataChunk;
class UNav3DNavDataRenderingComponent;
class ANav3DDataChunkActor;
class UNav3DWorldSubsystem;

DECLARE_MULTICAST_DELEGATE_OneParam(FNav3DGenerationFinishedDelegate, ANav3DData*);
DECLARE_MULTICAST_DELEGATE_TwoParams(FOnTacticalBuildCompleted, ANav3DData*, const TArray<FBox>&);

UCLASS(meta=(DisplayName="Nav3D Data"))
class NAV3D_API ANav3DData final : public ANavigationData
{
	GENERATED_BODY()

public:
	ANav3DData();
	virtual ~ANav3DData() override;

	friend class FNav3DDataGenerator;

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
	// Tactical build completion notifications
	FOnTacticalBuildCompleted OnTacticalBuildCompletedDelegate;

#if WITH_EDITORONLY_DATA
	/** Get the current chunk revision number (used by details panel to detect changes) */
	int32 GetChunkRevision() const;

	/** Increment chunk revision and notify of changes */
	void IncrementChunkRevision();

	/** Called when tactical build completes to refresh editor UI */
	void OnTacticalBuildCompleted(const TArray<FBox>& UpdatedVolumes);
#endif

	/** Rebuild tactical data for a specific volume (called from inspector) */
	void RebuildTacticalDataForVolume(const TArray<ANav3DDataChunkActor*>& VolumeChunks, const FBox& VolumeBounds);
	
	// Blueprint-callable cleanup method
	UFUNCTION(BlueprintCallable, meta=(DisplayName="Cleanup Invalid Chunk Actors", Category="Nav3D"))
	void CleanupInvalidChunkActorsBP();
	void CleanupAllInvalidActors();

	// =============================================================================
	// CONSOLIDATED TACTICAL BUILDING HELPERS  
	// =============================================================================
	
	// Consolidate regions from all loaded chunks
	void ConsolidateRegionsFromChunks(const TArray<ANav3DDataChunkActor*>& LoadedChunks);
	
	// Build cross-chunk adjacency from boundary interfaces
	void BuildCrossChunkAdjacency(const TArray<ANav3DDataChunkActor*>& LoadedChunks);
	
	// Build cross-chunk visibility using sample-based raycasting
	void BuildCrossChunkVisibility(const TArray<ANav3DDataChunkActor*>& LoadedChunks);
	
	// Build visibility sets for loaded regions, async
	void BuildVisibilitySetsForLoadedRegionsAsync(
		FConsolidatedTacticalData& ConsolidatedData, 
		TFunction<void()> OnCompleteCallback = nullptr) const;
	
	// Build adjacency between two specific chunks
	void BuildAdjacencyBetweenChunks(
		ANav3DDataChunkActor* ChunkA, 
		ANav3DDataChunkActor* ChunkB);
	
	// Filter consolidated data to only include selected regions after pruning
	void FilterConsolidatedDataToSelectedRegions(const TArray<int32>& SelectedRegionIds);

	// =============================================================================
	// NEW: CONSOLIDATED TACTICAL DATA MANAGEMENT
	// =============================================================================

	// Transient tactical data built from loaded chunks
	UPROPERTY(Transient)
	FConsolidatedTacticalData ConsolidatedTacticalData;

	// Compact tactical data built from loaded chunks (new format)
	UPROPERTY(Transient)
	FConsolidatedCompactTacticalData ConsolidatedCompactTacticalData;
	
	// Performance monitoring data
	UPROPERTY(Transient, VisibleAnywhere, Category="Nav3D Performance")
	FNav3DPerformanceStats PerformanceStats;
	
	// Loaded region tracking for tactical queries
	TSet<int32> LoadedRegionIds;
	mutable bool bConsolidatedDataDirty = true;
	
	// Consolidated tactical management methods
	const FConsolidatedTacticalData& GetConsolidatedTacticalData() const;
	void RefreshConsolidatedTacticalData();
	void RefreshConsolidatedDataFromCompact() const;
	void InvalidateConsolidatedData() const;
	void RebuildConsolidatedCompactTacticalData();
	void OnChunkActorLoaded(const ANav3DDataChunkActor* ChunkActor);
	void OnChunkActorUnloaded(ANav3DDataChunkActor* ChunkActor);

	/** Helpers for consolidated compact data */
    uint16 GetVolumeIDForGlobalRegion(uint16 GlobalRegionId) const;
    uint8 GetLocalRegionIndexForGlobalRegion(uint16 GlobalRegionId) const;

	// Compact consolidated data helper methods
	void ConsolidateCompactRegionsFromChunks(const TArray<ANav3DDataChunkActor*>& LoadedChunks);
	void BuildGlobalCompactAdjacency(const TArray<ANav3DDataChunkActor*>& LoadedChunks);
	
	// Build consolidated compact data directly from chunks' serialized compact data (no rebuild)
	void BuildConsolidatedCompactFromChunks(const TArray<ANav3DDataChunkActor*>& ChunksWithData);
	
	// Loaded region filtering for tactical queries
	void UpdateLoadedRegionIds();
	TSet<int32> GetLoadedRegionIds() const { return LoadedRegionIds; }
	bool IsRegionLoaded(int32 RegionId) const { return LoadedRegionIds.Contains(RegionId); }

	// Tactical query methods that work with both old and new formats
	bool FindBestTacticalLocation(
		const FVector& StartPosition,
		const TArray<FVector>& ObserverPositions,
		ETacticalVisibility Visibility,
		ETacticalDistance DistancePreference,
		ETacticalRegion RegionPreference,
		bool bForceNewRegion,
		bool bUseRaycasting,
		TArray<FPositionCandidate>& OutCandidatePositions) const;
	

	// Helper to get volume data containing specific points
	const FNav3DVolumeNavigationData* GetVolumeNavigationDataContainingPoints(
		const TArray<FVector>& Points) const;
	
	// Navigation data access (queries chunk actors)
	const FNav3DVolumeNavigationData* GetVolumeNavigationDataContainingPoint(const FVector& Point) const;
	
	// Bounds calculation (computed from chunk actors)
	FBox GetBoundingBox() const;
	
	// Navigation queries
	bool HasNavigationData() const { return ChunkActors.Num() > 0; }
	int32 GetChunkCount() const { return ChunkActors.Num(); }
	TArray<TObjectPtr<ANav3DDataChunkActor>> GetChunkActors() const { return ChunkActors; }

private:

    // Maps global region ID to volume/local region info (for compact data)
    UPROPERTY(Transient)
    TMap<uint16, struct FRegionMapping> GlobalToLocalRegionMapping;

    /** Timer handle for deferred tactical rebuild */
    FTimerHandle DeferredTacticalRebuildHandle;
    /** Flag indicating tactical data needs rebuilding */
    bool bNeedsTacticalRebuild = false;
	
    /** Perform deferred tactical data rebuild */
    UFUNCTION()
	void PerformDeferredTacticalRefresh();

public:
	// Volume information (read-only)
	UFUNCTION(BlueprintCallable, meta=(DisplayName="Show Partitioned Volumes", Category="Nav3D"))
	TArray<FBox> GetPartitionedVolumes() const;
	
	// Get all discoverable volumes from the world (built and unbuilt)
	TArray<FBox> GetAllDiscoverableVolumes() const;

	UFUNCTION(BlueprintCallable, meta=(DisplayName="Validate", Category="Nav3D"))
	void ValidateNavigationSystem();

	/** Convert compact tactical data to consolidated build format for debug rendering (editor/debug builds only) */
	UFUNCTION(CallInEditor, Category = "Nav3D Debug")
	void RebuildConsolidatedTacticalDataFromCompact();
	
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
	
	// Tactical build operations (build/global)
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
	static void BuildTacticalData();

	// Get the voxel extent based on the agent radius from NavConfig
	float GetVoxelExtent() const;
	int32 GetLayerCount() const;
	UNav3DWorldSubsystem* GetSubsystem() const;

	// Debug commands for chunk adjacency validation
	UFUNCTION(BlueprintCallable, CallInEditor, Category="Nav3D Debug")
	void DebugPrintChunkAdjacency();
	
	UFUNCTION(BlueprintCallable, CallInEditor, Category="Nav3D Debug")
	void ValidateAllChunkAdjacency();
	
	// Tactical data validation
	UFUNCTION(BlueprintCallable, CallInEditor, Category="Nav3D Debug")
	bool ValidateConsolidatedTacticalData() const;
	
	// Adjacency consistency validation
	void ValidateAdjacencyConsistency(const ANav3DDataChunkActor* ChunkA, const ANav3DDataChunkActor* ChunkB) const;
	int32 GetTotalRegionsInVolume() const;

	// Performance monitoring
	void UpdatePerformanceStats();
	float EstimateMemoryUsage() const;
	UFUNCTION(BlueprintCallable, CallInEditor, Category="Nav3D Debug")
	void LogPerformanceStats() const;

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
	void OnNavigationDataGenerationFinished() const;
	UNav3DDataChunk* GetNavigationDataChunk(ULevel* Level) const;
	static FBox CalculateLevelBounds(ULevel* Level);

	static FPathFindingResult FindPath(
		const FNavAgentProperties& NavAgentProperties,
		const FPathFindingQuery& PathFindingQuery);

	static FNav3DGenerationFinishedDelegate GenerationFinishedDelegate;


	// Track which volumes are currently loaded and their reference counts
	TMap<FBox, int32> LoadedVolumeReferenceCounts;  // Volume bounds → ref count
	mutable FCriticalSection VolumeLoadingMutex;    // Thread safety
	
	// Single volume build timing
	double SingleVolumeBuildStartTime = 0.0;
};

FORCEINLINE FNav3DVolumeDebugData ANav3DData::GetDebugData() const
{
	return DebugData;
}

FORCEINLINE FNav3DDataGenerationSettings ANav3DData::GetGenerationSettings() const
{
	return GenerationSettings;
}