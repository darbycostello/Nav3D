#pragma once

#include "CoreMinimal.h"
#include "Nav3DTypes.h"

class ANav3DData;
class ANav3DDataChunkActor;
class FNav3DVolumeNavigationData;

/**
 * Handles tactical data generation for Nav3D navigation system.
 * 
 * New Architecture:
 * - Phase 1: Build local tactical data per chunk (stored in chunk actors)
 * - Phase 2: Consolidate loaded chunks into runtime tactical data (managed by Nav3DData)
 */
class NAV3D_API FNav3DTacticalReasoning
{
public:
    FNav3DTacticalReasoning();
    ~FNav3DTacticalReasoning();

    // Initialize with Nav3DData reference
    void SetNavDataRef(ANav3DData* NavData);

    // Helper to filter and re-index tactical data after pruning
    static void FilterTacticalDataToSelectedRegions(
    FConsolidatedTacticalData& TacticalData, 
    const TArray<int32>& SelectedRegionIds);

    // Volume-level tactical data building (replaces per-chunk building)
    void BuildTacticalDataForVolume(
        const TArray<ANav3DDataChunkActor*>& VolumeChunks,
        const FBox& VolumeBounds);

    // Partition volume-wide tactical data into a specific chunk's compact format
    static FCompactTacticalData PartitionTacticalDataToChunk(
        const FConsolidatedTacticalData& VolumeData,
        const ANav3DDataChunkActor* TargetChunk,
        uint16 VolumeID);

    // Convert transient build structures to compact
    static FCompactTacticalData ConvertBuildToCompact(
        const TArray<FNav3DRegion>& BuildRegions,
        const TMap<int32, FRegionIdArray>& BuildAdjacency,
        uint16 VolumeID);
    
    // Helper method to check if a region is from a loaded chunk
    bool IsRegionFromLoadedChunk(int32 RegionId) const;

    // Build cross-chunk visibility using sample-based raycasting, async.
    void BuildVisibilitySetsForLoadedRegionsAsync(
        FConsolidatedTacticalData& ConsolidatedData,
        const TFunction<void()>& OnCompleteCallback = nullptr);

    /**
     * Find the best tactical positions from compact consolidated tactical data.
     * This provides the same interface but works with the new compact format.
     */
    bool FindBestLocationFromCompact(
        const FConsolidatedCompactTacticalData& CompactData,
        const FVector& StartPosition,
        const TArray<FVector>& ObserverPositions,
        ETacticalVisibility Visibility,
        ETacticalDistance DistancePreference,
        ETacticalRegion RegionPreference,
        bool bForceNewRegion,
        bool bUseRaycasting,
        TArray<FPositionCandidate>& OutCandidatePositions) const;

    // Get random point within a region
    static FVector GetRandomPointInRegion(const FNav3DRegion& Region);

    /**
     * Prune regions to a maximum limit using tactical analysis
     */
    static TArray<int32> PruneRegionsToLimit(
        const FConsolidatedTacticalData& ConsolidatedData,
        const FBox& VolumeBounds,
        int32 MaxRegions = 64);

private:
    // Extract regions from chunk's navigation data
    TArray<FNav3DRegion> ExtractRegionsFromChunk(
        ANav3DDataChunkActor* ChunkActor,
        const FNav3DVolumeNavigationData* VolumeData);
    
    // Build adjacency within a single chunk
    static void BuildRegionAdjacency(
        const TArray<FNav3DRegion>& LocalRegions,
        TMap<int32, FRegionIdArray>& OutRegionAdjacency);
    
    // Extract free voxels with coordinates for a layer
    static TArray<TPair<uint64, FIntVector>> ExtractFreeVoxelsWithCoords(
        int32 LayerIndex, 
        const FNav3DVolumeNavigationData* VolumeData);
    
    // Build box regions using greedy algorithm
    TArray<FBoxRegion> BuildBoxRegions(
        const TMap<FIntVector, bool>& VoxelGrid, 
        int32 LayerIndex);
    
    // Build voxel-level adjacency between region builders
    static void BuildVoxelLevelAdjacency(TArray<FNav3DRegionBuilder>& Regions);
    
    // Verify regions don't overlap with static geometry
    static void VerifyRegionsAgainstStaticGeometry(
        TArray<FNav3DRegion>& Regions,
        const FNav3DVolumeNavigationData* VolumeData);

    // Generate sample points for visibility testing
    TArray<FVector> GenerateSamplePoints(
        const FNav3DRegion& Region) const;

    // Check if two regions are spatially adjacent
    static bool AreRegionsAdjacent(const FNav3DRegion& RegionA, const FNav3DRegion& RegionB);
    
    // Check if region overlaps with static geometry
    static bool IsRegionInsideGeometry(
        const FNav3DRegion& Region,
        const UPrimitiveComponent* Component,
        const FNav3DVolumeNavigationData* VolumeData);

    // Build visibility sets with yielding to prevent editor freeze
    void ProcessVisibilityBuildChunk();

    // Reference to Nav3DData (weak pointer for safety)
    TWeakObjectPtr<ANav3DData> NavDataRef;
    
    // Region ID counter for chunk-local generation
    int32 NextRegionId = 0;

    // Timer-based visibility building state
    FTimerHandle VisibilityBuildTimerHandle;
    int32 CurrentVisibilityRegionIndex = 0;
    FConsolidatedTacticalData* ActiveVisibilityBuildData = nullptr;
    TFunction<void()> VisibilityBuildCompleteCallback;
    
    /**
     * Calculate pruning data for all regions
     */
    static void CalculatePruningData(
        const FConsolidatedTacticalData& ConsolidatedData,
        const FBox& VolumeBounds,
        TArray<FRegionPruningData>& OutPruningData);
    
    /**
     * Ensure spatial coverage across the volume
     */
    static void EnsureSpatialCoverage(
        const TArray<FRegionPruningData>& PruningData,
        const FBox& VolumeBounds,
        int32 SpatialBudget,
        TSet<int32>& SelectedRegions);
    
    /**
     * Select tactical extremes (high visibility, chokepoints, etc.)
     */
    static void SelectTacticalExtremes(
        const TArray<FRegionPruningData>& PruningData,
        int32 TacticalBudget,
        TSet<int32>& SelectedRegions);
    
    /**
     * Fill remaining slots with highest scoring regions
     */
    static void FillRemainingSlots(
        const TArray<FRegionPruningData>& PruningData,
        int32 MaxRegions,
        TSet<int32>& SelectedRegions);
    
    /**
     * Add top candidates from sorted data
     */
    static void AddTopCandidates(
        const TArray<FRegionPruningData>& SortedData,
        int32 Count,
        TSet<int32>& SelectedRegions);
    
    /**
     * Calculate composite tactical score for a region
     */
    static float CalculateTacticalScore(const FRegionPruningData& Data);
    
    /**
     * Check if a region is near volume boundaries
     */
    static bool IsBoundaryRegion(const FNav3DRegion& Region, const FBox& VolumeBounds);
    
    /**
     * Check if a region is a chokepoint (low connectivity)
     */
    static bool IsChokePoint(const FNav3DRegion& Region, const FConsolidatedTacticalData& Data);
    
    /**
     * Calculate distance variance for tactical uniqueness
     */
    static float CalculateDistanceVariance(const FNav3DRegion& Region, const FConsolidatedTacticalData& Data);
};

/**
 * Density-focused pruning strategy that prioritizes tactically complex areas
 */
class NAV3D_API FDensityFocusedPruningStrategy
{
public:
    /**
     * Prune regions to a maximum limit using density-focused tactical analysis
     */
    static TArray<int32> PruneRegionsToLimit(
        const FConsolidatedTacticalData& ConsolidatedData,
        const FBox& VolumeBounds,
        const TArray<ANav3DDataChunkActor*>& ChunkActors,
        int32 MaxRegions = 64);

private:
    /**
     * Calculate density-focused tactical data for all regions
     */
    static void CalculateDensityPruningData(
        const FConsolidatedTacticalData& ConsolidatedData,
        const FBox& VolumeBounds,
        const TArray<ANav3DDataChunkActor*>& ChunkActors,
        TArray<FDensityRegionPruningData>& OutPruningData);
    
    /**
     * Calculate local geometry density around a region
     */
    static float CalculateLocalGeometryDensity(
        const FNav3DRegion& Region, 
        const TArray<ANav3DDataChunkActor*>& ChunkActors);
    
    /**
     * Calculate proximity to dense geometry
     */
    static float CalculateGeometryProximity(
        const FNav3DRegion& Region,
        const TArray<ANav3DDataChunkActor*>& ChunkActors);
    
    /**
     * Calculate visibility complexity (how interesting the visibility pattern is)
     */
    static float CalculateVisibilityComplexity(
        const FNav3DRegion& Region,
        const FConsolidatedTacticalData& ConsolidatedData);
    
    /**
     * Calculate adjacency complexity
     */
    static float CalculateAdjacencyComplexity(
        const FNav3DRegion& Region,
        const FConsolidatedTacticalData& ConsolidatedData);
    
    /**
     * Calculate composite tactical complexity score
     */
    static float CalculateTacticalComplexityScore(const FDensityRegionPruningData& Data);
    
    /**
     * Select high-density tactical zones (50% of budget)
     */
    static void SelectHighDensityZones(
        const TArray<FDensityRegionPruningData>& PruningData,
        int32 DensityBudget,
        TSet<int32>& SelectedRegions);
    
    /**
     * Ensure tactical diversity (30% of budget)
     */
    static void EnsureTacticalDiversity(
        const TArray<FDensityRegionPruningData>& PruningData,
        int32 DiversityBudget,
        TSet<int32>& SelectedRegions);
    
    /**
     * Ensure basic spatial coverage for larger ships (20% of budget)
     */
    static void EnsureBasicSpatialCoverage(
        const TArray<FDensityRegionPruningData>& PruningData,
        const FBox& VolumeBounds,
        int32 SpatialBudget,
        TSet<int32>& SelectedRegions);
    
    /**
     * Fill remaining slots with highest scoring regions
     */
    static void FillRemainingSlots(
        const TArray<FDensityRegionPruningData>& PruningData,
        int32 MaxRegions,
        TSet<int32>& SelectedRegions);
    
    /**
     * Add top candidates from sorted data
     */
    static void AddTopCandidates(
        const TArray<FDensityRegionPruningData>& SortedData,
        int32 Count,
        TSet<int32>& SelectedRegions);
    
    /**
     * Check if a region is near volume boundaries
     */
    static bool IsBoundaryRegion(const FNav3DRegion& Region, const FBox& VolumeBounds);
    
    /**
     * Check if a region is a chokepoint (low connectivity)
     */
    static bool IsChokePoint(const FNav3DRegion& Region, const FConsolidatedTacticalData& Data);
};