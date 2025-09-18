#include "Tactical/Nav3DTacticalDataConverter.h"
#include "Nav3D.h"
#include "Nav3DDataChunkActor.h"

FCompactRegion FNav3DTacticalDataConverter::RegionToCompact(const FNav3DRegion& BuildRegion)
{
    // Store center and size directly - no coordinate conversion needed!
    const FVector WorldCenter = BuildRegion.Bounds.GetCenter();
    const FVector WorldSize = BuildRegion.Bounds.GetSize();
    
    return FCompactRegion(
        static_cast<uint8>(BuildRegion.LayerIndex),
        WorldCenter,
        WorldSize
    );
}

FNav3DRegion FNav3DTacticalDataConverter::CompactToRegion(const FCompactRegion& CompactRegion, const int32 RegionId)
{
    // Perfect reconstruction using stored center and size
    const FBox Bounds = CompactRegion.GetWorldBounds();
    return FNav3DRegion(RegionId, Bounds, CompactRegion.LayerIndex);
}

bool FNav3DTacticalDataConverter::ValidateBuildData(const FConsolidatedTacticalData& BuildData)
{
    return !BuildData.IsEmpty();
}

bool FNav3DTacticalDataConverter::ValidateCompactData(const FConsolidatedCompactTacticalData& CompactData)
{
    return !CompactData.IsEmpty();
}

FConsolidatedCompactTacticalData FNav3DTacticalDataConverter::BuildToCompact(
    const FConsolidatedTacticalData& BuildData)
{
    QUICK_SCOPE_CYCLE_COUNTER(STAT_Nav3D_BuildToCompact);

    FConsolidatedCompactTacticalData CompactData = {};
    if (!ValidateBuildData(BuildData))
    {
        UE_LOG(LogNav3D, Warning, TEXT("BuildToCompact: Invalid build data"));
        return CompactData;
    }

    // Regions: pack by region id (clamped to uint16)
    const int32 MaxRegions = FMath::Min(BuildData.AllLoadedRegions.Num(), 65535);
    for (int32 i = 0; i < MaxRegions; ++i)
    {
        const FNav3DRegion& BuildRegion = BuildData.AllLoadedRegions[i];
        const uint16 GlobalId = static_cast<uint16>(FMath::Clamp(BuildRegion.Id, 0, 65535));
        CompactData.AllLoadedRegions.Add(GlobalId, RegionToCompact(BuildRegion));
    }

    // Adjacency → bitmask
    CompactData.GlobalRegionAdjacency = AdjacencyToBitmask(BuildData.RegionAdjacency);

    // Visibility → sparse matrix (best-effort single-matrix bucket)
    CompactData.VolumeVisibilityData = VisibilityToSparseMatrix(BuildData.RegionVisibility);

    // Source chunks passthrough
    CompactData.SourceChunks = BuildData.SourceChunks.Array();

    UE_LOG(LogNav3D, Log, TEXT("BuildToCompact: %d regions, %d adjacency, %d visibility matrices"),
        CompactData.AllLoadedRegions.Num(), CompactData.GlobalRegionAdjacency.Num(), CompactData.VolumeVisibilityData.Num());

    return CompactData;
}

FConsolidatedTacticalData FNav3DTacticalDataConverter::CompactToBuild(
    const FConsolidatedCompactTacticalData& CompactData,
    const TArray<ANav3DDataChunkActor*>& SourceChunks)
{
    QUICK_SCOPE_CYCLE_COUNTER(STAT_Nav3D_CompactToBuild);

    FConsolidatedTacticalData BuildData;
    if (!ValidateCompactData(CompactData))
    {
        UE_LOG(LogNav3D, Warning, TEXT("CompactToBuild: Invalid compact data"));
        return BuildData;
    }
    
    // Regions: perfect reconstruction using stored center + size
    BuildData.AllLoadedRegions.Reserve(CompactData.AllLoadedRegions.Num());
    for (const auto& Pair : CompactData.AllLoadedRegions)
    {
        const uint16 GlobalId = Pair.Key;
        const FCompactRegion& CR = Pair.Value;
        
        // Perfect bounds reconstruction - no VolumeData needed!
        BuildData.AllLoadedRegions.Add(CompactToRegion(CR, GlobalId));
    }

    // Adjacency → build lists
    BuildData.RegionAdjacency = BitmaskToAdjacency(CompactData.GlobalRegionAdjacency);

    // Visibility → build lists (no World needed, uses the sparse matrix directly)
    BuildData.RegionVisibility = SparseMatrixToVisibility(CompactData.VolumeVisibilityData);

    // Source chunks passthrough
    for (ANav3DDataChunkActor* ChunkActor : SourceChunks)
    {
        BuildData.SourceChunks.Add(ChunkActor);
    }

    return BuildData;
}

TMap<uint16, uint64> FNav3DTacticalDataConverter::AdjacencyToBitmask(
    const TMap<int32, FRegionIdArray>& BuildAdjacency)
{
    TMap<uint16, uint64> Out;
    for (const auto& Pair : BuildAdjacency)
    {
        const int32 RegionId = Pair.Key;
        if (RegionId < 0 || RegionId > 65535) { continue; }
        uint64 Mask = 0;
        for (const int32 Adj : Pair.Value.RegionIds)
        {
            if (Adj >= 0 && Adj < 64)
            {
                Mask |= (1ULL << Adj);
            }
        }
        if (Mask != 0)
        {
            Out.Add(static_cast<uint16>(RegionId), Mask);
        }
    }
    return Out;
}

TMap<int32, FRegionIdArray> FNav3DTacticalDataConverter::BitmaskToAdjacency(
    const TMap<uint16, uint64>& CompactAdjacency)
{
    TMap<int32, FRegionIdArray> Out;
    for (const auto& Pair : CompactAdjacency)
    {
        const uint16 RegionId = Pair.Key;
        const uint64 Mask = Pair.Value;
        FRegionIdArray Adj;
        for (int32 Bit = 0; Bit < 64; ++Bit)
        {
            if (Mask & (1ULL << Bit))
            {
                Adj.Add(Bit);
            }
        }
        if (Adj.Num() > 0)
        {
            Out.Add(static_cast<int32>(RegionId), Adj);
        }
    }
    return Out;
}

TMap<uint16, FVolumeRegionMatrix> FNav3DTacticalDataConverter::VisibilityToSparseMatrix(
    const TMap<int32, FRegionIdArray>& BuildVisibility)
{
    UE_LOG(LogNav3D, Warning, TEXT("=== SAVE DEBUG: VisibilityToSparseMatrix ==="));
    UE_LOG(LogNav3D, Warning, TEXT("SAVE CONVERSION: Using synthetic VolumeID = 0 for all entries"));
    UE_LOG(LogNav3D, Verbose, TEXT("SAVE CONVERSION: Processing %d viewer regions"), BuildVisibility.Num());

    // Single volume (ID=0) containing all renumbered regions 0-63
    TMap<uint16, FVolumeRegionMatrix> Out;
    FVolumeRegionMatrix& M = Out.FindOrAdd(0);
    
    UE_LOG(LogNav3D, Verbose, TEXT("VisibilityToSparseMatrix: Processing %d viewer regions"), BuildVisibility.Num());
    
    for (const auto& Pair : BuildVisibility)
    {
        const int32 Viewer = Pair.Key;
        const uint8 LocalViewer = static_cast<uint8>(Viewer & 0x3F);
        
        // Validate viewer region ID is in expected range
        if (Viewer < 0 || Viewer >= 64)
        {
            UE_LOG(LogNav3D, Warning, TEXT("VisibilityToSparseMatrix: Viewer region %d out of range [0-63], skipping"), Viewer);
            continue;
        }
        
        uint64 Mask = 0;
        for (const int32 Target : Pair.Value.RegionIds)
        {
            // Validate target region ID is in expected range
            if (Target < 0 || Target >= 64)
            {
                UE_LOG(LogNav3D, Warning, TEXT("VisibilityToSparseMatrix: Target region %d out of range [0-63], skipping"), Target);
                continue;
            }
            
            Mask |= (1ULL << Target);
        }
        
        if (Mask != 0)
        {
            // Store all visibility as intra-volume references (TargetVolumeID=0)
            // This matches the 0-63 renumbering scheme
            M.SetReferenceMask(LocalViewer, /*TargetVolumeID*/0, Mask);
            
            UE_LOG(LogNav3D, VeryVerbose, TEXT("VisibilityToSparseMatrix: Region %d sees %d regions (mask=0x%llX)"), 
                   LocalViewer, FPlatformMath::CountBits(Mask), Mask);
        }
        else
        {
            UE_LOG(LogNav3D, VeryVerbose, TEXT("VisibilityToSparseMatrix: Region %d has no valid targets"), Viewer);
        }
    }
    
    UE_LOG(LogNav3D, Verbose, TEXT("VisibilityToSparseMatrix: Stored %d entries in volume matrix"), 
           M.SparseReferences.Num());
    
    // Debug: Log the final sparse references to verify they're stored correctly
    for (const auto& RefPair : M.SparseReferences)
    {
        uint8 DecodedViewer;
        uint16 DecodedTargetVolume;
        FVolumeRegionMatrix::DecodeKey(RefPair.Key, DecodedViewer, DecodedTargetVolume);
        UE_LOG(LogNav3D, VeryVerbose, TEXT("  Key=%d -> Viewer=%d, TargetVolume=%d, Mask=0x%llX (%d targets)"), 
               RefPair.Key, DecodedViewer, DecodedTargetVolume, RefPair.Value, FPlatformMath::CountBits(RefPair.Value));
    }
    
    return Out;
}

TMap<int32, FRegionIdArray> FNav3DTacticalDataConverter::SparseMatrixToVisibility(
    const TMap<uint16, FVolumeRegionMatrix>& CompactVisibility)
{
    TMap<int32, FRegionIdArray> Out;
    for (const auto& VolPair : CompactVisibility)
    {
        const FVolumeRegionMatrix& Matrix = VolPair.Value;

        // Iterate over the sparse references in this volume's matrix
        for (const TPair<uint16, uint64>& RefPair : Matrix.SparseReferences)
        {
            // Decode the key using the known layout: (TargetVolumeID << 6) | LocalRegionID
            const uint8 LocalRegionA = static_cast<uint8>(RefPair.Key & 0x3F);
            
            const uint64 Mask = RefPair.Value;
            
            // Since FilterConsolidatedDataToSelectedRegions() renumbers regions to 0-63 per volume,
            // the visibility data should also use this same 0-63 indexing scheme
            const int32 ViewerGlobalID = LocalRegionA;
            
            FRegionIdArray& Visible = Out.FindOrAdd(ViewerGlobalID);
            
            // Extract target region IDs from the bitmask
            for (int32 Bit = 0; Bit < 64; ++Bit)
            {
                if (Mask & (1ULL << Bit))
                {
                    const int32 TargetGlobalID = Bit;
                    Visible.Add(TargetGlobalID);
                }
            }
        }
    }
    
    return Out;
}

FBox FNav3DTacticalDataConverter::CompactRegionToWorldBounds(
    const FCompactRegion& CompactRegion, 
    const FNav3DVolumeNavigationData* VolumeData)
{
    // Perfect bounds reconstruction using stored center and size
    return CompactRegion.GetWorldBounds();
}

// New simplified center calculation function
FVector FNav3DTacticalDataConverter::CompactRegionToWorldCenter(const FCompactRegion& CompactRegion)
{
    // Simple and reliable - just return the stored center
    return CompactRegion.GetWorldCenter();
}
