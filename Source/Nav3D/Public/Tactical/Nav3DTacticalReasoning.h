#pragma once

#include "CoreMinimal.h"
#include "Tactical/Nav3DTacticalTypes.h"

class ANav3DData;
class ANav3DDataChunkActor;
class ANav3DTacticalActor;
class FNav3DVolumeNavigationData;

// Class that handles the creation and queries for the tactical data
class NAV3D_API FNav3DTacticalReasoning
{
public:
    FNav3DTacticalReasoning();
    ~FNav3DTacticalReasoning();

    // Initialize with an existing Nav3DData instance
    void Initialize(ANav3DData* NavData);

    void BuildTacticalData(const FBox& VolumeBounds);
    
    // Global tactical generation that processes all chunks together
    void BuildGlobalTacticalData(const TArray<ANav3DDataChunkActor*>& ChunkActors);
    
    // Create tactical actor for regions
    void CreateTacticalActorForRegions(const TArray<FNav3DRegion>& Regions, int32 LayerIndex) const;
    
    // Build tactical data for a specific layer (for async processing)
    void BuildTacticalDataForLayer(int32 LayerIndex, const TArray<ANav3DDataChunkActor*>& ChunkActors);
    
    /**
     * Finds the best positions based on tactical criteria like visibility, distance, and region size.
     * This method searches for regions that match the specified visibility requirements against all
     * provided observer positions, and ranks them according to the specified preferences.
     * @param TargetData
     * @param StartPosition The position to start the search from
     * @param ObserverPositions Array of observer positions to check visibility against
     * @param Visibility Visibility relationship to require between observers and candidate positions
     * @param DistancePreference How to weight candidates based on distance
     * @param RegionPreference How to weight candidates based on region size
     * @param bForceNewRegion If true, forces selection of a region different from the StartPosition's region
     * @param bUseRaycasting If true, performs raycasts to validate visibility requirements
     * @param OutCandidatePositions [out] Array of candidate positions sorted by score (best first)
     * @return True if suitable candidate positions were found, false otherwise
     */
    bool FindBestLocation(
        const FNav3DTacticalData& TargetData,  // Changed to const reference
        const FVector& StartPosition,
        const TArray<FVector>& ObserverPositions,
        ETacticalVisibility Visibility,
        ETacticalDistance DistancePreference,
        ETacticalRegion RegionPreference,
        bool bForceNewRegion,
        bool bUseRaycasting,
        TArray<FPositionCandidate>& OutCandidatePositions) const;
    
    // Get a random point within a region
    static FVector GetRandomPointInRegion(const FNav3DRegion& Region);

private:
    // Extract free voxels at the specified layer
    TArray<FVector> ExtractFreeVoxelsInLayer(int32 LayerIndex) const;

    TArray<FNav3DRegionBuilder> SubdivideVoxelsForLayer(
        const TArray<TPair<uint64, FIntVector>>& FreeVoxels,
        int32 LayerIndex,
        int32 MaxRegionLayer,
        const FNav3DVolumeNavigationData* VolumeData
    );

    // Build adjacency graph
    static void BuildAdjacencyGraph(FNav3DTacticalData& TargetData);
    static void BuildAdjacencyGraph(TArray<FNav3DRegion>& Regions);

    static bool IsPositionNavigable(const FNav3DTacticalData& TargetData, const FVector& Position, int32 LayerIndex);
    
    // Build visibility sets using parallel raycasting
    void BuildVisibilitySets(FNav3DTacticalData& TargetData) const;

    // Generate sample points for a region
    TArray<FVector> GenerateSamplePoints(const FNav3DTacticalData& TargetData, const FNav3DRegion& Region) const;

    static TArray<FNav3DRegionBuilder> MergeBoxRegions(const TArray<FNav3DRegionBuilder>& BoxRegions);

    // Calculate average region volume
    static float CalculateAverageRegionVolume(const FNav3DTacticalData& TargetData);

    // Find the region with the specified ID
    static const FNav3DRegion* GetRegionById(const FNav3DTacticalData& TargetData, int32 RegionId);

    static TArray<TPair<uint64, FIntVector>> ExtractFreeVoxelsWithCoords(int32 LayerIndex, const FNav3DVolumeNavigationData* VolumeData);
    TArray<FNav3DRegionBuilder> BuildInitialRegions(const TArray<TPair<uint64, FIntVector>>& FreeVoxels, int32 LayerIndex);
    TArray<FNav3DRegionBuilder> RefineRegionsToBoxes(const TArray<FNav3DRegionBuilder>& InitialRegions);
    TArray<FNav3DRegionBuilder> PartitionRegion(const FNav3DRegionBuilder& Region);
    static bool CanMergeRegions(const FNav3DRegionBuilder& RegionA, const FNav3DRegionBuilder& RegionB);
    TArray<FNav3DRegion> SplitLargeRegion(const FNav3DRegion& Region, float MaxRegionSize);
    TArray<FBoxRegion> BuildBoxRegions(const TMap<FIntVector, bool>& VoxelGrid, int32 LayerIndex);
    void ReindexRegions(TArray<FNav3DRegion>& Regions);
    void VerifyRegionsAgainstStaticGeometry(TArray<FNav3DRegion>& Regions, const FNav3DVolumeNavigationData* VolumeData);
    static bool IsRegionInsideGeometry(const FNav3DRegion& Region, UPrimitiveComponent* Component,
                                       const FNav3DVolumeNavigationData* VolumeData);

    // Build voxel-level adjacency between regions
    static void BuildVoxelLevelAdjacency(TArray<FNav3DRegionBuilder>& Regions);
    
    // Build adjacency between regions in different layers
    static void BuildCrossLayerAdjacency(TArray<FNav3DRegion>& Regions, const FNav3DVolumeNavigationData* VolumeData);
    
    // Merge two regions, preserving adjacency information
    static FNav3DRegionBuilder MergeRegions(const FNav3DRegionBuilder& RegionA, 
                                          const FNav3DRegionBuilder& RegionB);
    
    // Update adjacency references after region merging
    static void UpdateAdjacentRegionReferences(TArray<FNav3DRegionBuilder>& Regions, 
                                             int32 RegionAIndex, int32 RegionBId);

    // Reference to Nav3DData
    TWeakObjectPtr<ANav3DData> NavDataRef;

    // Next available region ID
    int32 NextRegionId;

    // Check if two regions are adjacent
    static bool AreRegionsAdjacent(const FNav3DRegion& RegionA, const FNav3DRegion& RegionB);
};