#pragma once

#include "CoreMinimal.h"
#include "Nav3DData.h"
#include "Nav3DTypes.h"

/**
 * Bidirectional converter between build-time and compact tactical data formats
 */
class NAV3D_API FNav3DTacticalDataConverter
{
public:
    // Adjacency conversion
    static TMap<uint16, uint64> AdjacencyToBitmask(
        const TMap<int32, FRegionIdArray>& BuildAdjacency);
    static TMap<int32, FRegionIdArray> BitmaskToAdjacency(
        const TMap<uint16, uint64>& CompactAdjacency);

    // Visibility conversion (best-effort; lacks strict volume mapping)
    static TMap<uint16, FVolumeRegionMatrix> VisibilityToSparseMatrix(
        const TMap<int32, FRegionIdArray>& BuildVisibility);
    static TMap<int32, FRegionIdArray> SparseMatrixToVisibility(
        const TMap<uint16, FVolumeRegionMatrix>& CompactVisibility);

    static FCompactRegion RegionToCompact(const FNav3DRegion& BuildRegion);
    
    static FNav3DRegion CompactToRegion(const FCompactRegion& CompactRegion, int32 RegionId);

    // Build → Compact (for serialization)
    static FConsolidatedCompactTacticalData BuildToCompact(
        const FConsolidatedTacticalData& BuildData);

    // Compact → Build (for debug/editor tools)
    static FConsolidatedTacticalData CompactToBuild(
        const FConsolidatedCompactTacticalData& CompactData,
        const TArray<ANav3DDataChunkActor*>& SourceChunks);

    // Conversion function for debug drawing
    static FBox CompactRegionToWorldBounds(
        const FCompactRegion& CompactRegion, 
        const FNav3DVolumeNavigationData* VolumeData);
    
    // New simplified center calculation function
    static FVector CompactRegionToWorldCenter(const FCompactRegion& CompactRegion);

private:
    static bool ValidateBuildData(const FConsolidatedTacticalData& BuildData);
    static bool ValidateCompactData(const FConsolidatedCompactTacticalData& CompactData);
};


