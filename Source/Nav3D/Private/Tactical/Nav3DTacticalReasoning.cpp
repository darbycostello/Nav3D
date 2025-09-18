#include "Tactical/Nav3DTacticalReasoning.h"

#include "EngineUtils.h"
#include "Nav3DData.h"
#include "Nav3DDataChunkActor.h"
#include "Nav3DUtils.h"
#include "Raycasting/Nav3DRaycaster.h"
#include "Nav3D.h"
#include "LandscapeHeightfieldCollisionComponent.h"
#include "Nav3DSettings.h"
#include "Nav3DVolumeIDSystem.h"
#include "Tactical/Nav3DTacticalDataConverter.h"

FNav3DTacticalReasoning::FNav3DTacticalReasoning()
{
}

FNav3DTacticalReasoning::~FNav3DTacticalReasoning()
{
    // Clean up any active async operations
    if (VisibilityBuildTimerHandle.IsValid() && NavDataRef.IsValid())
    {
        if (UWorld* World = NavDataRef->GetWorld())
        {
            World->GetTimerManager().ClearTimer(VisibilityBuildTimerHandle);
            UE_LOG(LogNav3D, Warning, TEXT("Cleaned up active visibility build timer in FNav3DTacticalReasoning destructor"));
        }
    }
    
    // Clean up state
    ActiveVisibilityBuildData = nullptr;
    CurrentVisibilityRegionIndex = 0;
    VisibilityBuildCompleteCallback = nullptr;
}

void FNav3DTacticalReasoning::SetNavDataRef(ANav3DData* NavData)
{
    NavDataRef = NavData;
}

void FNav3DTacticalReasoning::FilterTacticalDataToSelectedRegions(
    FConsolidatedTacticalData& TacticalData, 
    const TArray<int32>& SelectedRegionIds)
{
    if (SelectedRegionIds.Num() == 0)
    {
        return;
    }
    
    // Create a set for fast lookup and mapping from old IDs to new sequential IDs
    TSet SelectedSet(SelectedRegionIds);
    TMap<int32, int32> OldToNewIdMapping;
    
    // Create mapping from old region IDs to new sequential IDs (0 to SelectedRegionIds.Num()-1)
    for (int32 i = 0; i < SelectedRegionIds.Num(); ++i)
    {
        OldToNewIdMapping.Add(SelectedRegionIds[i], i);
    }
    
    // Filter and renumber regions
    TArray<FNav3DRegion> FilteredRegions;
    FilteredRegions.Reserve(SelectedRegionIds.Num());
    for (const FNav3DRegion& Region : TacticalData.AllLoadedRegions)
    {
        if (SelectedSet.Contains(Region.Id))
        {
            FNav3DRegion RenumberedRegion = Region;
            RenumberedRegion.Id = OldToNewIdMapping[Region.Id];
            FilteredRegions.Add(RenumberedRegion);
        }
    }
    
    // Filter and renumber adjacency data
    TMap<int32, FRegionIdArray> FilteredAdjacency;
    for (const auto& AdjPair : TacticalData.RegionAdjacency)
    {
        int32 OldRegionId = AdjPair.Key;
        if (SelectedSet.Contains(OldRegionId))
        {
            int32 NewRegionId = OldToNewIdMapping[OldRegionId];
            FRegionIdArray FilteredAdjacentIds;
            for (int32 OldAdjacentId : AdjPair.Value.GetArray())
            {
                if (SelectedSet.Contains(OldAdjacentId))
                {
                    int32 NewAdjacentId = OldToNewIdMapping[OldAdjacentId];
                    FilteredAdjacentIds.Add(NewAdjacentId);
                }
            }
            if (FilteredAdjacentIds.Num() > 0)
            {
                FilteredAdjacency.Add(NewRegionId, FilteredAdjacentIds);
            }
        }
    }
    
    // Filter and renumber visibility data
    TMap<int32, FRegionIdArray> FilteredVisibility;
    for (const auto& VisPair : TacticalData.RegionVisibility)
    {
        int32 OldRegionId = VisPair.Key;
        if (SelectedSet.Contains(OldRegionId))
        {
            int32 NewRegionId = OldToNewIdMapping[OldRegionId];
            FRegionIdArray FilteredVisibleIds;
            for (int32 OldVisibleId : VisPair.Value.GetArray())
            {
                if (SelectedSet.Contains(OldVisibleId))
                {
                    int32 NewVisibleId = OldToNewIdMapping[OldVisibleId];
                    FilteredVisibleIds.Add(NewVisibleId);
                }
            }
            if (FilteredVisibleIds.Num() > 0)
            {
                FilteredVisibility.Add(NewRegionId, FilteredVisibleIds);
            }
        }
    }
    
    // Update tactical data
    TacticalData.AllLoadedRegions = MoveTemp(FilteredRegions);
    TacticalData.RegionAdjacency = MoveTemp(FilteredAdjacency);
    TacticalData.RegionVisibility = MoveTemp(FilteredVisibility);
    
    UE_LOG(LogNav3D, Verbose, TEXT("Filtered and renumbered tactical data: %d regions (IDs 0-%d), %d adjacency entries, %d visibility entries"),
           TacticalData.AllLoadedRegions.Num(), TacticalData.AllLoadedRegions.Num() - 1, 
           TacticalData.RegionAdjacency.Num(), TacticalData.RegionVisibility.Num());
}

void FNav3DTacticalReasoning::BuildTacticalDataForVolume(
    const TArray<ANav3DDataChunkActor*>& VolumeChunks,
    const FBox& VolumeBounds)
{
    NextRegionId = 0;
    
    if (!NavDataRef.IsValid() || VolumeChunks.Num() == 0)
    {
        return;
    }

    UE_LOG(LogNav3D, Log, TEXT("Building cross-chunk tactical data for %d chunks in volume [%s]"),
        VolumeChunks.Num(), *VolumeBounds.ToString());

    // Step 1: Extract ALL free regions from ALL chunks (no bounds filtering)
    TArray<FNav3DRegion> AllVolumeRegions;
    for (ANav3DDataChunkActor* ChunkActor : VolumeChunks)
    {
        if (!ChunkActor) continue;
        for (UNav3DDataChunk* Chunk : ChunkActor->Nav3DChunks)
        {
            if (!Chunk) continue;
            if (const FNav3DVolumeNavigationData* VolumeData = Chunk->GetVolumeNavigationData())
            {
                TArray<FNav3DRegion> ChunkRegions = ExtractRegionsFromChunk(ChunkActor, VolumeData);
                AllVolumeRegions.Append(ChunkRegions);
            }
        }
    }

    if (AllVolumeRegions.Num() == 0)
    {
        UE_LOG(LogNav3D, Warning, TEXT("No regions extracted from volume"));
        return;
    }
    
    // Step 2: Build region adjacency
    TMap<int32, FRegionIdArray> RegionAdjacency;
    BuildRegionAdjacency(AllVolumeRegions, RegionAdjacency);
    
    // Store in heap to persist across async callback
    FConsolidatedTacticalData* TacticalDataTemp = new FConsolidatedTacticalData();
    TacticalDataTemp->AllLoadedRegions = AllVolumeRegions;
    TacticalDataTemp->RegionAdjacency = RegionAdjacency;
    
    // Capture all needed variables for the callback
    TArray<ANav3DDataChunkActor*> CapturedChunks = VolumeChunks;
    FBox CapturedBounds = VolumeBounds;

    // Step 3: Build cross-chunk visibility sets across ALL regions (ASYNC)
    BuildVisibilitySetsForLoadedRegionsAsync(
        *TacticalDataTemp, [this, TacticalDataTemp, CapturedBounds, CapturedChunks]()
    {
        // This callback runs when visibility building is complete
        UE_LOG(LogNav3D, Log, TEXT("Visibility building completed, proceeding with pruning and partitioning"));
        
        // Step 4: Apply pruning BEFORE partitioning
        TArray<int32> SelectedRegionIds = FDensityFocusedPruningStrategy::PruneRegionsToLimit(
            *TacticalDataTemp, CapturedBounds, CapturedChunks, 64);

        // Step 5: Re-index the regions in the range [0...63], including visibility pairs
        FilterTacticalDataToSelectedRegions(*TacticalDataTemp, SelectedRegionIds);
        
        UE_LOG(LogNav3D, Log, TEXT("Built and pruned tactical data: %d regions -> %d regions"),
            TacticalDataTemp->AllLoadedRegions.Num(), SelectedRegionIds.Num());

        // Step 6: Get the proper VolumeID from the Nav3DBoundsVolume that contains these chunks
        uint16 VolumeID = 0;
        
        // Find the Nav3DBoundsVolume that contains the first chunk
        if (CapturedChunks.Num() > 0 && CapturedChunks[0])
        {
            if (UWorld* World = CapturedChunks[0]->GetWorld())
            {
                // Get all loaded volume IDs mapped to their GUIDs
                TMap<uint16, FGuid> LoadedVolumeIDs = FNav3DVolumeIDSystem::GetLoadedVolumeIDs(World);
                
                // Find which Nav3DBoundsVolume contains the first chunk by checking bounds overlap
                const FVector ChunkCenter = CapturedChunks[0]->DataChunkActorBounds.GetCenter();
                
                for (TActorIterator<ANav3DBoundsVolume> It(World); It; ++It)
                {
                    if (ANav3DBoundsVolume* BoundsVolume = *It)
                    {
                        // Check if the chunk center is inside this Nav3DBoundsVolume
                        if (const FBox BoundsBox = BoundsVolume->GetComponentsBoundingBox(); BoundsBox.IsInside(ChunkCenter))
                        {
                            // Find the VolumeID for this volume's GUID
                            const FGuid VolumeGUID = BoundsVolume->VolumeGUID;
                            
                            // Search for this GUID in the loaded volume IDs
                            for (const auto& Pair : LoadedVolumeIDs)
                            {
                                if (Pair.Value == VolumeGUID)
                                {
                                    VolumeID = Pair.Key;
                                    UE_LOG(LogNav3D, Warning, TEXT("Found Nav3DBoundsVolume %s with GUID %s -> VolumeID %d for chunks"), 
                                           *BoundsVolume->GetName(), *VolumeGUID.ToString(), VolumeID);
                                    break;
                                }
                            }
                            
                            if (VolumeID != 0) 
                            {
                                break; // Found the volume and its ID
                            }
                            else
                            {
                                UE_LOG(LogNav3D, Warning, TEXT("Found Nav3DBoundsVolume %s with GUID %s but no corresponding VolumeID in system"), 
                                       *BoundsVolume->GetName(), *VolumeGUID.ToString());
                            }
                        }
                    }
                }
            }
        }
        
        // Fallback to hash if we couldn't find the proper volume
        if (VolumeID == 0)
        {
            VolumeID = static_cast<uint16>(GetTypeHash(CapturedBounds) % 1024);
            UE_LOG(LogNav3D, Warning, TEXT("Could not find Nav3DBoundsVolume for chunks, using fallback VolumeID %d"), VolumeID);
        }

        // Step 7: Partition pruned data into chunks - ALL chunks get the SAME VolumeID
        for (ANav3DDataChunkActor* ChunkActor : CapturedChunks)
        {
            if (!ChunkActor) continue;
            ChunkActor->CompactTacticalData = PartitionTacticalDataToChunk(*TacticalDataTemp, ChunkActor, VolumeID);
            UE_LOG(LogNav3D, Verbose, TEXT("Partitioned %d regions to chunk %s with VolumeID %d"),
                ChunkActor->CompactTacticalData.Regions.Num(), *ChunkActor->GetName(), VolumeID);
        }
        
        // Clean up heap-allocated temp data
        // IMPORTANT: Invalidate consolidated data after tactical build completes
        if (NavDataRef.IsValid())
        {
            NavDataRef->InvalidateConsolidatedData();
            UE_LOG(LogNav3D, Log, TEXT("Tactical build complete - consolidated data invalidated for refresh"));
        }

        delete TacticalDataTemp;
        
        UE_LOG(LogNav3D, Log, TEXT("Completed tactical data building for volume"));
    });
}

FCompactTacticalData FNav3DTacticalReasoning::PartitionTacticalDataToChunk(
    const FConsolidatedTacticalData& VolumeData,
    const ANav3DDataChunkActor* TargetChunk,
    const uint16 VolumeID)
{
    UE_LOG(LogNav3D, Warning, TEXT("=== SAVE DEBUG: PartitionTacticalDataToChunk ==="));
    UE_LOG(LogNav3D, Warning, TEXT("SAVE: Using VolumeID = %d for chunk %s"), VolumeID, TargetChunk ?
        *TargetChunk->GetName() : TEXT("NULL"));

    FCompactTacticalData ChunkData;
    ChunkData.VolumeID = VolumeID;

    if (!TargetChunk)
    {
        return ChunkData;
    }

    TMap<int32, uint8> GlobalToLocalIdMap;
    uint8 LocalIndex = 0;

    // Step 1: Find regions whose centers are in this chunk
    for (const FNav3DRegion& Region : VolumeData.AllLoadedRegions)
    {
        if (TargetChunk->DataChunkActorBounds.IsInside(Region.Bounds.GetCenter()))
        {
            GlobalToLocalIdMap.Add(Region.Id, LocalIndex);
            ChunkData.Regions.Add(FNav3DTacticalDataConverter::RegionToCompact(Region));
            LocalIndex++;
            if (LocalIndex == 64) break; // Should never happen, due to pruning and re-indexing.
        }
    }

    // Step 2: Convert adjacency data using local-to-local indices
    for (const auto& AdjPair : VolumeData.RegionAdjacency)
    {
        const int32 GlobalFromId = AdjPair.Key;
        if (const uint8* LocalFromId = GlobalToLocalIdMap.Find(GlobalFromId))
        {
            uint64 LocalAdjMask = 0;
            for (const int32 GlobalToId : AdjPair.Value.RegionIds)
            {
                if (const uint8* LocalToId = GlobalToLocalIdMap.Find(GlobalToId))
                {
                    LocalAdjMask |= (1ULL << *LocalToId);
                }
            }

            if (LocalAdjMask != 0)
            {
                ChunkData.RegionAdjacency.Add(*LocalFromId, LocalAdjMask);
            }
        }
    }

    // Step 3: Convert visibility data - FIXED TO STORE ALL VISIBILITY DATA IN ALL CHUNKS
    int32 VisibilityEntriesSaved = 0;
    
    // IMPORTANT: Store ALL visibility relationships in ALL chunks, not just chunk-local ones.
    // This is because visibility queries can happen from any chunk and need access to the full data.
    // There's no data overhead for this because we still store the full uint64 for the sparse matrix.
    for (const auto& VisPair : VolumeData.RegionVisibility)
    {
        const int32 GlobalFromId = VisPair.Key;
        
        // Convert global viewer ID to local index for storage key
        const uint8 LocalViewerId = static_cast<uint8>(GlobalFromId & 0x3F);
        
        // Build bitmask of all visible regions (using global IDs as bit positions)
        uint64 LocalVisMask = 0;
        for (const int32 GlobalToId : VisPair.Value.RegionIds)
        {
            if (GlobalToId >= 0 && GlobalToId < 64)
            {
                LocalVisMask |= (1ULL << GlobalToId);
            }
        }
        
        if (LocalVisMask != 0)
        {
            // Encode key: (VolumeID << 6) | LocalViewerId
            const uint16 Key = (VolumeID << 6) | LocalViewerId;
            ChunkData.VisibilityMatrix.SparseReferences.Add(Key, LocalVisMask);
            VisibilityEntriesSaved++;
        }
    }

    UE_LOG(LogNav3D, Warning, TEXT("SAVE: Stored %d visibility entries for chunk %s with VolumeID %d"), 
           VisibilityEntriesSaved, TargetChunk ? *TargetChunk->GetName() : TEXT("NULL"), VolumeID);
    
    UE_LOG(LogNav3D, Warning, TEXT("SAVE: Chunk has %d regions, %d adjacency entries, %d visibility entries"),
           ChunkData.Regions.Num(), ChunkData.RegionAdjacency.Num(), ChunkData.VisibilityMatrix.SparseReferences.Num());

    return ChunkData;
}

bool FNav3DTacticalReasoning::IsRegionFromLoadedChunk(const int32 RegionId) const
{
    if (!NavDataRef.IsValid())
    {
        return false;
    }
    
    return NavDataRef->IsRegionLoaded(RegionId);
}

void FNav3DTacticalReasoning::BuildVisibilitySetsForLoadedRegionsAsync(
    FConsolidatedTacticalData& ConsolidatedData,
    const TFunction<void()>& OnCompleteCallback)
{
    if (!NavDataRef.IsValid() || ConsolidatedData.IsEmpty())
    {
        if (OnCompleteCallback)
        {
            OnCompleteCallback();
        }
        return;
    }
    
    // Clear existing visibility data
    ConsolidatedData.RegionVisibility.Empty();

    const UWorld* World = NavDataRef->GetWorld();
    if (!World)
    {
        UE_LOG(LogNav3D, Warning, TEXT("BuildVisibilitySetsForLoadedRegionsAsync: No world available"));
        if (OnCompleteCallback)
        {
            OnCompleteCallback();
        }
        return;
    }
    
    // Stop any existing visibility build
    if (VisibilityBuildTimerHandle.IsValid())
    {
        World->GetTimerManager().ClearTimer(VisibilityBuildTimerHandle);
    }
    
    // Setup async build state
    ActiveVisibilityBuildData = &ConsolidatedData;
    VisibilityBuildCompleteCallback = OnCompleteCallback;
    CurrentVisibilityRegionIndex = 0;
    
    UE_LOG(LogNav3D, Log, TEXT("Starting async visibility build for %d regions"), 
           ConsolidatedData.AllLoadedRegions.Num());
    
    // Start timer-based processing
    FTimerDelegate Delegate;
    Delegate.BindRaw(this, &FNav3DTacticalReasoning::ProcessVisibilityBuildChunk);
    World->GetTimerManager().SetTimer(VisibilityBuildTimerHandle, Delegate, 0.01f, true);
}

void FNav3DTacticalReasoning::ProcessVisibilityBuildChunk()
{
    // Add safety check at the beginning
    if (!NavDataRef.IsValid())
    {
        UE_LOG(LogNav3D, Warning, TEXT("NavDataRef became invalid during async visibility build, stopping"));
        
        // Clean up timer
        if (VisibilityBuildTimerHandle.IsValid())
        {
            if (UWorld* World = GWorld) // Fallback world access
            {
                World->GetTimerManager().ClearTimer(VisibilityBuildTimerHandle);
            }
        }
        
        // Clean up state
        ActiveVisibilityBuildData = nullptr;
        CurrentVisibilityRegionIndex = 0;
        VisibilityBuildCompleteCallback = nullptr;
        return;
    }

    if (!ActiveVisibilityBuildData)
    {
        UE_LOG(LogNav3D, Warning, TEXT("ActiveVisibilityBuildData is null during async visibility build, stopping"));
        
        // Clean up and complete
        if (VisibilityBuildTimerHandle.IsValid())
        {
            NavDataRef->GetWorld()->GetTimerManager().ClearTimer(VisibilityBuildTimerHandle);
        }
        if (VisibilityBuildCompleteCallback)
        {
            VisibilityBuildCompleteCallback();
            VisibilityBuildCompleteCallback = nullptr;
        }
        return;
    }
    
    // Process visibility for a limited number of regions per chunk to prevent freezing
    static constexpr float MaxChunkTimeSeconds = 0.03f; // 30ms chunks
    static constexpr int32 MinRegionsPerChunk = 1; // Always process at least 1 region
    const double StartTime = FPlatformTime::Seconds();
    
    const UWorld* World = NavDataRef->GetWorld();
    if (!World)
    {
        UE_LOG(LogNav3D, Error, TEXT("World became invalid during visibility build"));
        return;
    }
    
    // Setup collision parameters for visibility raycasts
    FCollisionQueryParams CollisionParams;
    CollisionParams.bTraceComplex = true;
    CollisionParams.bReturnPhysicalMaterial = false;
    CollisionParams.TraceTag = FName(TEXT("TacticalVisibility"));
    constexpr ECollisionChannel VisibilityChannel = ECC_Visibility;
    
    int32 ProcessedInThisChunk = 0;
    bool bHasTimeRemaining = true;
    
    while (bHasTimeRemaining && 
           CurrentVisibilityRegionIndex < ActiveVisibilityBuildData->AllLoadedRegions.Num())
    {
        const int32 i = CurrentVisibilityRegionIndex;
        const FNav3DRegion& ViewerRegion = ActiveVisibilityBuildData->AllLoadedRegions[i];
        TArray<FVector> ViewerSamples = GenerateSamplePoints(ViewerRegion);
        
        // Build visibility for this region - MATCH THE ORIGINAL LOGIC EXACTLY
        FRegionIdArray& VisibleRegions = ActiveVisibilityBuildData->RegionVisibility.FindOrAdd(ViewerRegion.Id);
        
        for (int32 j = 0; j < ActiveVisibilityBuildData->AllLoadedRegions.Num(); ++j)
        {
            if (i == j)
            {
                VisibleRegions.Add(ViewerRegion.Id); // Self-visible - SAME AS ORIGINAL
                continue;
            }
            
            const FNav3DRegion& TargetRegion = ActiveVisibilityBuildData->AllLoadedRegions[j];
            TArray<FVector> TargetSamples = GenerateSamplePoints(TargetRegion);
            
            // Test visibility between sample pairs using physics raycasts - SAME AS ORIGINAL
            int32 VisiblePairs = 0;
            int32 TestedPairs = 0;  // Track actual tests performed (excluding invalid samples)
            
            for (const FVector& ViewerPos : ViewerSamples)
            {
                for (const FVector& TargetPos : TargetSamples)
                {
                    // Skip if positions are too close (degenerate case) - SAME AS ORIGINAL
                    const float Distance = FVector::Dist(ViewerPos, TargetPos);
                    if (Distance < 10.0f)  // 10cm minimum distance
                    {
                        continue;
                    }
                    
                    TestedPairs++;
                    
                    // Perform physics raycast for line-of-sight check - SAME AS ORIGINAL
                    FHitResult HitResult;
                    const bool bHit = World->LineTraceSingleByChannel(
                        HitResult,
                        ViewerPos,
                        TargetPos,
                        VisibilityChannel,
                        CollisionParams
                    );
                    
                    // Line of sight is clear if no blocking hit occurred - SAME AS ORIGINAL
                    if (!bHit)
                    {
                        VisiblePairs++;
                    }
                }
            }
            
            // Calculate visibility ratio based on tested pairs - SAME AS ORIGINAL
            const float VisibilityRatio = TestedPairs > 0 ? 
                static_cast<float>(VisiblePairs) / static_cast<float>(TestedPairs) : 0.0f;
            
            UE_LOG(LogNav3D, VeryVerbose, TEXT("Tactical Visibility: Region %d -> Region %d: %d/%d visible pairs (%.2f%%) vs threshold %.2f%%"), 
                   ViewerRegion.Id, TargetRegion.Id, VisiblePairs, TestedPairs, 
                   VisibilityRatio * 100.0f, NavDataRef->TacticalSettings.VisibilityScoreThreshold * 100.0f);
            
            if (VisibilityRatio > NavDataRef->TacticalSettings.VisibilityScoreThreshold)
            {
                VisibleRegions.Add(TargetRegion.Id);
                UE_LOG(LogNav3D, Verbose, TEXT("Tactical Visibility: Region %d marked as visible from Region %d"), 
                       TargetRegion.Id, ViewerRegion.Id);
            }
            else
            {
                UE_LOG(LogNav3D, Verbose, TEXT("Tactical Visibility: Region %d marked as NOT visible from Region %d (below threshold)"), 
                       TargetRegion.Id, ViewerRegion.Id);
            }
        }
        
        CurrentVisibilityRegionIndex++;
        ProcessedInThisChunk++;
        
        // Check if we should yield (but always process at least one region)
        if (ProcessedInThisChunk >= MinRegionsPerChunk)
        {
            const double CurrentTime = FPlatformTime::Seconds();
            bHasTimeRemaining = (CurrentTime - StartTime) < MaxChunkTimeSeconds;
        }
    }
    
    if (CurrentVisibilityRegionIndex % 10 == 0 || CurrentVisibilityRegionIndex >= ActiveVisibilityBuildData->AllLoadedRegions.Num())
    {
        UE_LOG(LogNav3D, Log, TEXT("Visibility build progress: %d/%d regions (%.1f%%)"), 
               CurrentVisibilityRegionIndex, 
               ActiveVisibilityBuildData->AllLoadedRegions.Num(),
               100.0f * CurrentVisibilityRegionIndex / ActiveVisibilityBuildData->AllLoadedRegions.Num());
    }
    
    // Check if build is complete
    if (CurrentVisibilityRegionIndex >= ActiveVisibilityBuildData->AllLoadedRegions.Num())
    {
        UE_LOG(LogNav3D, Log, TEXT("Async visibility build completed for %d regions"), 
               ActiveVisibilityBuildData->AllLoadedRegions.Num());
        
        // Clean up timer
        if (VisibilityBuildTimerHandle.IsValid())
        {
            World->GetTimerManager().ClearTimer(VisibilityBuildTimerHandle);
        }
        
        // Reset state
        ActiveVisibilityBuildData = nullptr;
        CurrentVisibilityRegionIndex = 0;
        
        // Call completion callback
        if (VisibilityBuildCompleteCallback)
        {
            VisibilityBuildCompleteCallback();
            VisibilityBuildCompleteCallback = nullptr;
        }
    }
}

FCompactTacticalData FNav3DTacticalReasoning::ConvertBuildToCompact(
    const TArray<FNav3DRegion>& BuildRegions,
    const TMap<int32, FRegionIdArray>& BuildAdjacency,
    const uint16 VolumeID)
{
    FCompactTacticalData CompactData;
    CompactData.Reset();
    CompactData.VolumeID = VolumeID;
    
    const int32 MaxRegions = FMath::Min(BuildRegions.Num(), 64);
    CompactData.Regions.Reserve(MaxRegions);
    for (int32 i = 0; i < MaxRegions; ++i)
    {
        // Create a placeholder compact region - this function is deprecated
        // in favor of building compact regions directly from voxel coordinates
        FCompactRegion PlaceholderRegion;
        PlaceholderRegion.LayerIndex = static_cast<uint8>(BuildRegions[i].LayerIndex);
        CompactData.Regions.Add(PlaceholderRegion);
    }
    
    for (const auto& Pair : BuildAdjacency)
    {
        const int32 RegionId = Pair.Key;
        const FRegionIdArray& Adj = Pair.Value;
        if (RegionId >= 0 && RegionId < 64)
        {
            uint64 Mask = 0;
            for (const int32 A : Adj.RegionIds)
            {
                if (A >= 0 && A < 64) { Mask |= (1ULL << A); }
            }
            if (Mask != 0)
            {
                CompactData.RegionAdjacency.Add(static_cast<uint8>(RegionId), Mask);
            }
        }
    }
    
    return CompactData;
}

TArray<FNav3DRegion> FNav3DTacticalReasoning::ExtractRegionsFromChunk(
    ANav3DDataChunkActor* ChunkActor,
    const FNav3DVolumeNavigationData* VolumeData)
{
    TArray<FNav3DRegion> ExtractedRegions;
    TArray<FCompactRegion> CompactRegions; // Build these in parallel
    
    if (!ChunkActor || !VolumeData || !NavDataRef.IsValid())
    {
        return ExtractedRegions;
    }
    
    const FNav3DTacticalSettings& TacticalSettings = NavDataRef->TacticalSettings;
    const FNav3DData& OctreeData = VolumeData->GetData();
    const int32 LayerCount = OctreeData.GetLayerCount();
    const int32 MinRegionLayer = TacticalSettings.MinRegioningLayer;
    const int32 MaxRegionLayer = FMath::Min(TacticalSettings.MaxRegioningLayer, LayerCount - 1);
    
    // Process each layer within the chunk
    for (int32 LayerIdx = MinRegionLayer; LayerIdx <= MaxRegionLayer; ++LayerIdx)
    {
        // Extract free voxels for this layer within the chunk bounds
        TArray<TPair<uint64, FIntVector>> LayerFreeVoxels = ExtractFreeVoxelsWithCoords(LayerIdx, VolumeData);
        
        if (LayerFreeVoxels.Num() == 0)
        {
            continue;
        }
        
        // Filter voxels to only include those within this chunk's bounds
        TArray<TPair<uint64, FIntVector>> ChunkFreeVoxels;
        for (const auto& VoxelPair : LayerFreeVoxels)
        {
            FVector VoxelWorldPos;
            if (LayerIdx == 0)
            {
                VoxelWorldPos = VolumeData->GetLeafNodePositionFromMortonCode(VoxelPair.Key);
            }
            else
            {
                VoxelWorldPos = VolumeData->GetNodePositionFromLayerAndMortonCode(LayerIdx, VoxelPair.Key);
            }
            
            if (ChunkActor->DataChunkActorBounds.IsInside(VoxelWorldPos))
            {
                ChunkFreeVoxels.Add(VoxelPair);
            }
        }
        
        if (ChunkFreeVoxels.Num() == 0)
        {
            continue;
        }
        
        UE_LOG(LogNav3D, Verbose, TEXT("Layer %d: Found %d free voxels in chunk bounds"), LayerIdx, ChunkFreeVoxels.Num());
        
        // Build regions for this layer
        TArray<FNav3DRegionBuilder> RegionBuilders;
        
        if (LayerIdx < MaxRegionLayer)
        {
            // Use box-building algorithm for lower layers
            TMap<FIntVector, bool> VoxelGrid;
            for (const auto& VoxelPair : ChunkFreeVoxels)
            {
                VoxelGrid.Add(VoxelPair.Value, true);
            }
            
            TArray<FBoxRegion> BoxRegions = BuildBoxRegions(VoxelGrid, LayerIdx);
            
            for (const FBoxRegion& BoxRegion : BoxRegions)
            {
                RegionBuilders.Add(BoxRegion.ToRegionBuilder(ChunkFreeVoxels));
            }
            
            // Build voxel-level adjacency
            BuildVoxelLevelAdjacency(RegionBuilders);
        }
        else
        {
            // Create individual voxel regions for highest layer
            for (const auto& VoxelPair : ChunkFreeVoxels)
            {
                FNav3DRegionBuilder Builder(NextRegionId++, LayerIdx);
                Builder.MinCoord = VoxelPair.Value;
                Builder.MaxCoord = VoxelPair.Value;
                Builder.MortonCodes.Add(VoxelPair.Key);
                RegionBuilders.Add(Builder);
            }
            
            BuildVoxelLevelAdjacency(RegionBuilders);
        }
        
        // Convert builders to compact regions
        TArray<FNav3DRegion> LayerRegions;
        for (const FNav3DRegionBuilder& Builder : RegionBuilders)
        {
            if (Builder.IsValid())
            {
                // Create Build region for compatibility
                FNav3DRegion BuildRegion = Builder.ToRegion(VolumeData);
                LayerRegions.Add(BuildRegion);
        
                // Create compact region from the Build region's bounds
                CompactRegions.Add(FNav3DTacticalDataConverter::RegionToCompact(BuildRegion));
            }
        }
        
        // Verify regions against static geometry
        VerifyRegionsAgainstStaticGeometry(LayerRegions, VolumeData);
        
        // Add to extracted regions
        ExtractedRegions.Append(LayerRegions);
        
        UE_LOG(LogNav3D, Verbose, TEXT("Layer %d: Created %d verified regions"), LayerIdx, LayerRegions.Num());
    }
    
    // Store compact regions in chunk actor
    ChunkActor->CompactRegions = CompactRegions;
    
    // If no regions were extracted at any layer, treat the entire chunk as a single free region
    if (ExtractedRegions.Num() == 0)
    {
        const FBox& ChunkBounds = ChunkActor->DataChunkActorBounds;
        FNav3DRegion WholeChunkRegion(NextRegionId++, ChunkBounds, /*LayerIndex*/ 0);
        ExtractedRegions.Add(WholeChunkRegion);
        CompactRegions.Add(FNav3DTacticalDataConverter::RegionToCompact(WholeChunkRegion));
        
        // Update compact regions on the chunk actor to include the synthetic region
        ChunkActor->CompactRegions = CompactRegions;
        
        UE_LOG(LogNav3D, Verbose, TEXT("ExtractRegionsFromChunk: No free-voxel regions found; created whole-chunk region %d for %s"),
               WholeChunkRegion.Id, *ChunkActor->GetName());
    }
    
    return ExtractedRegions;
}

void FNav3DTacticalReasoning::BuildRegionAdjacency(
    const TArray<FNav3DRegion>& LocalRegions,
    TMap<int32, FRegionIdArray>& OutRegionAdjacency)
{
    OutRegionAdjacency.Empty();
    
    // Build adjacency between all region pairs within the chunk
    for (int32 i = 0; i < LocalRegions.Num(); ++i)
    {
        // Create adjacency list for this region (always create new to avoid TMap reference issues)
        FRegionIdArray RegionAdjacencyList;
        
        for (int32 j = i + 1; j < LocalRegions.Num(); ++j)
        {
            if (AreRegionsAdjacent(LocalRegions[i], LocalRegions[j]))
            {
                RegionAdjacencyList.Add(j);
            }
        }
        
        // Add the adjacency list to the output map (only if it has entries)
        if (RegionAdjacencyList.Num() > 0)
        {
            OutRegionAdjacency.Add(i, RegionAdjacencyList);
        }
    }
    
    // Now build the reverse adjacency (bidirectional)
    for (int32 i = 0; i < LocalRegions.Num(); ++i)
    {
        if (FRegionIdArray* AdjacencyList = OutRegionAdjacency.Find(LocalRegions[i].Id))
        {
            // For each region in this adjacency list, add this region to their adjacency list
            for (int32 AdjacentId : AdjacencyList->GetArray())
            {
                if (FRegionIdArray* ReverseAdjacencyList = OutRegionAdjacency.Find(AdjacentId))
                {
                    ReverseAdjacencyList->Add(LocalRegions[i].Id);
                }
                else
                {
                    // Create new adjacency list for the adjacent region
                    FRegionIdArray NewAdjacencyList;
                    NewAdjacencyList.Add(LocalRegions[i].Id);
                    OutRegionAdjacency.Add(AdjacentId, NewAdjacencyList);
                }
            }
        }
    }
    
    UE_LOG(LogNav3D, Verbose, TEXT("Built region adjacency for %d regions"), LocalRegions.Num());
}

// =============================================================================
// REGION BUILDING CORE FUNCTIONS (Restored from original implementation)
// =============================================================================

TArray<TPair<uint64, FIntVector>> FNav3DTacticalReasoning::ExtractFreeVoxelsWithCoords(
    const int32 LayerIndex, 
    const FNav3DVolumeNavigationData* VolumeData)
{
    TArray<TPair<uint64, FIntVector>> FreeVoxels;
    
    if (!VolumeData)
    {
        UE_LOG(LogNav3D, Warning, TEXT("ExtractFreeVoxelsWithCoords: No volume data provided"));
        return FreeVoxels;
    }
    
    const FNav3DData& OctreeData = VolumeData->GetData();
    
    if (!OctreeData.IsValid())
    {
        UE_LOG(LogNav3D, Warning, TEXT("ExtractFreeVoxelsWithCoords: Octree data is not valid"));
        return FreeVoxels;
    }
    
    if (LayerIndex >= OctreeData.GetLayerCount())
    {
        UE_LOG(LogNav3D, Warning, TEXT("ExtractFreeVoxelsWithCoords: Layer index %d out of bounds (max: %d)"), LayerIndex, OctreeData.GetLayerCount());
        return FreeVoxels;
    }
    
    const FNav3DLayer& Layer = OctreeData.GetLayer(LayerIndex);
    
    for (const FNav3DNode& Node : Layer.GetNodes())
    {
        // Nodes with children are occupied
        if (!Node.HasChildren())
        {
            uint64 MortonCode = Node.MortonCode;
            FIntVector Coord = FIntVector(FNav3DUtils::GetVectorFromMortonCode(MortonCode));
            
            // Only include voxels within the volume bounds
            FVector NodePosition;
            if (LayerIndex == 0)
            {
                NodePosition = VolumeData->GetLeafNodePositionFromMortonCode(MortonCode);
            }
            else
            {
                NodePosition = VolumeData->GetNodePositionFromLayerAndMortonCode(LayerIndex, MortonCode);
            }
            
            if (VolumeData->GetVolumeBounds().IsInside(NodePosition))
            {
                FreeVoxels.Add(TPair<uint64, FIntVector>(MortonCode, Coord));
            }
        }
    }
    
    return FreeVoxels;
}

TArray<FBoxRegion> FNav3DTacticalReasoning::BuildBoxRegions(
    const TMap<FIntVector, bool>& VoxelGrid, 
    int32 LayerIndex)
{
    TSet<FIntVector> AvailableVoxels;
    for (auto It = VoxelGrid.CreateConstIterator(); It; ++It)
    {
        AvailableVoxels.Add(It.Key());
    }
    
    TArray<FBoxRegion> Results;
    
    while (AvailableVoxels.Num() > 0)
    {
        // Get deterministic seed voxel
        TArray<FIntVector> AvailableArray = AvailableVoxels.Array();
        AvailableArray.Sort([](const FIntVector& A, const FIntVector& B) {
            if (A.Z != B.Z) return A.Z < B.Z;
            if (A.Y != B.Y) return A.Y < B.Y;
            return A.X < B.X;
        });
        
        FIntVector SeedCoord = AvailableArray[0];
        FBoxRegion CurrentBox(NextRegionId++, SeedCoord, SeedCoord, LayerIndex);
        AvailableVoxels.Remove(SeedCoord);
        
        // Expand greedily in all directions
        bool bContinueExpanding = true;
        while (bContinueExpanding)
        {
            bContinueExpanding = false;
            
            // Try each direction
            const TArray<FIntVector> Directions = {
                FIntVector(1, 0, 0), FIntVector(-1, 0, 0),  // ±X
                FIntVector(0, 1, 0), FIntVector(0, -1, 0),  // ±Y
                FIntVector(0, 0, 1), FIntVector(0, 0, -1)   // ±Z
            };
            
            for (const FIntVector& Dir : Directions)
            {
                // Check if we can expand in this direction
                bool bCanExpand = true;
                FIntVector NewMin = CurrentBox.Min;
                FIntVector NewMax = CurrentBox.Max;
                
                if (Dir.X > 0) NewMax.X += 1;
                else if (Dir.X < 0) NewMin.X -= 1;
                else if (Dir.Y > 0) NewMax.Y += 1;
                else if (Dir.Y < 0) NewMin.Y -= 1;
                else if (Dir.Z > 0) NewMax.Z += 1;
                else if (Dir.Z < 0) NewMin.Z -= 1;
                
                // Check if all voxels in the expansion are available
                for (int32 X = NewMin.X; X <= NewMax.X && bCanExpand; ++X)
                {
                    for (int32 Y = NewMin.Y; Y <= NewMax.Y && bCanExpand; ++Y)
                    {
                        for (int32 Z = NewMin.Z; Z <= NewMax.Z && bCanExpand; ++Z)
                        {
                            FIntVector TestCoord(X, Y, Z);
                            // Skip voxels already in the current box
                            if (X >= CurrentBox.Min.X && X <= CurrentBox.Max.X &&
                                Y >= CurrentBox.Min.Y && Y <= CurrentBox.Max.Y &&
                                Z >= CurrentBox.Min.Z && Z <= CurrentBox.Max.Z)
                            {
                                continue;
                            }
                            
                            if (!AvailableVoxels.Contains(TestCoord))
                            {
                                bCanExpand = false;
                            }
                        }
                    }
                }
                
                if (bCanExpand)
                {
                    // Remove newly claimed voxels from available set
                    for (int32 X = NewMin.X; X <= NewMax.X; ++X)
                    {
                        for (int32 Y = NewMin.Y; Y <= NewMax.Y; ++Y)
                        {
                            for (int32 Z = NewMin.Z; Z <= NewMax.Z; ++Z)
                            {
                                FIntVector TestCoord(X, Y, Z);
                                if (!(X >= CurrentBox.Min.X && X <= CurrentBox.Max.X &&
                                      Y >= CurrentBox.Min.Y && Y <= CurrentBox.Max.Y &&
                                      Z >= CurrentBox.Min.Z && Z <= CurrentBox.Max.Z))
                                {
                                    AvailableVoxels.Remove(TestCoord);
                                }
                            }
                        }
                    }
                    
                    CurrentBox.Min = NewMin;
                    CurrentBox.Max = NewMax;
                    bContinueExpanding = true;
                    break; // Try expansion again from the beginning
                }
            }
        }
        
        Results.Add(CurrentBox);
    }
    
    return Results;
}

void FNav3DTacticalReasoning::BuildVoxelLevelAdjacency(TArray<FNav3DRegionBuilder>& Regions)
{
    // Create coordinate to region mapping
    TMap<FIntVector, int32> CoordToRegionIndex;
    for (int32 i = 0; i < Regions.Num(); ++i)
    {
        for (const uint64 MortonCode : Regions[i].MortonCodes)
        {
            FIntVector Coord = FIntVector(FNav3DUtils::GetVectorFromMortonCode(MortonCode));
            CoordToRegionIndex.Add(Coord, i);
        }
    }
    
    const TArray<FIntVector> Directions = {
        FIntVector(1, 0, 0), FIntVector(-1, 0, 0),
        FIntVector(0, 1, 0), FIntVector(0, -1, 0),
        FIntVector(0, 0, 1), FIntVector(0, 0, -1)
    };
    
    // Check adjacency for each region
    for (int32 i = 0; i < Regions.Num(); ++i)
    {
        FNav3DRegionBuilder& Region = Regions[i];
        
        for (const uint64 MortonCode : Region.MortonCodes)
        {
            FIntVector Coord = FIntVector(FNav3DUtils::GetVectorFromMortonCode(MortonCode));
            
            for (const FIntVector& Dir : Directions)
            {
                FIntVector NeighborCoord = Coord + Dir;
                if (const int32* NeighborRegionIndex = CoordToRegionIndex.Find(NeighborCoord))
                {
                    if (*NeighborRegionIndex != i)
                    {
                        Region.AdjacentRegionIds.Add(Regions[*NeighborRegionIndex].Id);
                        Regions[*NeighborRegionIndex].AdjacentRegionIds.Add(Region.Id);
                    }
                }
            }
        }
    }
}

void FNav3DTacticalReasoning::VerifyRegionsAgainstStaticGeometry(
    TArray<FNav3DRegion>& Regions,
    const FNav3DVolumeNavigationData* VolumeData)
{
    if (!VolumeData || !VolumeData->Settings.World || Regions.Num() == 0)
    {
        return;
    }
    
    const FCollisionQueryParams& QueryParams = VolumeData->Settings.GenerationSettings.CollisionQueryParameters;
    const ECollisionChannel CollisionChannel = VolumeData->Settings.GenerationSettings.CollisionChannel;
    
    TArray<int32> RegionsToRemove;
    
    for (int32 RegionIndex = 0; RegionIndex < Regions.Num(); ++RegionIndex)
    {
        const FNav3DRegion& Region = Regions[RegionIndex];
        FVector BoxCenter = Region.Bounds.GetCenter();
        FVector BoxExtent = Region.Bounds.GetExtent() * 0.9f;
        
        TArray<FOverlapResult> Overlaps;
        const bool bHasOverlap = VolumeData->Settings.World->OverlapMultiByChannel(
            Overlaps, BoxCenter, FQuat::Identity, CollisionChannel,
            FCollisionShape::MakeBox(BoxExtent), QueryParams);
        
        if (bHasOverlap)
        {
            for (const FOverlapResult& Overlap : Overlaps)
            {
                UPrimitiveComponent* Component = Overlap.GetComponent();
                if (Component && Component->CanEverAffectNavigation())
                {
                    if (Cast<UStaticMeshComponent>(Component) || 
                        Cast<ULandscapeHeightfieldCollisionComponent>(Component))
                    {
                        if (IsRegionInsideGeometry(Region, Component, VolumeData))
                        {
                            RegionsToRemove.Add(RegionIndex);
                            break;
                        }
                    }
                }
            }
        }
    }
    
    // Remove invalid regions (in reverse order)
    if (RegionsToRemove.Num() > 0)
    {
        RegionsToRemove.Sort([](const int32 A, const int32 B) { return A > B; });
        for (const int32 IndexToRemove : RegionsToRemove)
        {
            if (IndexToRemove < Regions.Num())
            {
                Regions.RemoveAt(IndexToRemove);
            }
        }
        
        // Reindex regions
        for (int32 i = 0; i < Regions.Num(); ++i)
        {
            Regions[i].Id = i;
        }
    }
}

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================

FVector FNav3DTacticalReasoning::GetRandomPointInRegion(const FNav3DRegion& Region)
{
    const FVector Center = Region.Bounds.GetCenter();
    const FVector Extent = Region.Bounds.GetExtent() * 0.8f;
    
    return Center + FVector(
        FMath::FRandRange(-Extent.X, Extent.X),
        FMath::FRandRange(-Extent.Y, Extent.Y),
        FMath::FRandRange(-Extent.Z, Extent.Z));
}

bool FNav3DTacticalReasoning::AreRegionsAdjacent(const FNav3DRegion& RegionA, const FNav3DRegion& RegionB)
{
    const FBox& BoxA = RegionA.Bounds;
    const FBox& BoxB = RegionB.Bounds;
    constexpr float Epsilon = 50.0f; // adjustable tolerance

    // Check overlap amounts on each axis
    const float XOverlap = FMath::Max(0.0f, FMath::Min(BoxA.Max.X, BoxB.Max.X) - FMath::Max(BoxA.Min.X, BoxB.Min.X));
    const float YOverlap = FMath::Max(0.0f, FMath::Min(BoxA.Max.Y, BoxB.Max.Y) - FMath::Max(BoxA.Min.Y, BoxB.Min.Y));
    const float ZOverlap = FMath::Max(0.0f, FMath::Min(BoxA.Max.Z, BoxB.Max.Z) - FMath::Max(BoxA.Min.Z, BoxB.Min.Z));

    // Check adjacency along X axis (faces touching)
    const bool AdjacentX = 
        (FMath::IsNearlyEqual(BoxA.Max.X, BoxB.Min.X, Epsilon) || 
         FMath::IsNearlyEqual(BoxA.Min.X, BoxB.Max.X, Epsilon)) &&
        (YOverlap > 0 && ZOverlap > 0);

    // Check adjacency along Y axis
    const bool AdjacentY = 
        (FMath::IsNearlyEqual(BoxA.Max.Y, BoxB.Min.Y, Epsilon) || 
         FMath::IsNearlyEqual(BoxA.Min.Y, BoxB.Max.Y, Epsilon)) &&
        (XOverlap > 0 && ZOverlap > 0);

    // Check adjacency along Z axis
    const bool AdjacentZ = 
        (FMath::IsNearlyEqual(BoxA.Max.Z, BoxB.Min.Z, Epsilon) || 
         FMath::IsNearlyEqual(BoxA.Min.Z, BoxB.Max.Z, Epsilon)) &&
        (XOverlap > 0 && YOverlap > 0);

    return AdjacentX || AdjacentY || AdjacentZ;
}


bool FNav3DTacticalReasoning::IsRegionInsideGeometry(
    const FNav3DRegion& Region,
    const UPrimitiveComponent* Component,
    const FNav3DVolumeNavigationData* VolumeData)
{
    if (!Component || !VolumeData)
    {
        return false;
    }
    
    // Simple overlap test - can be improved
    const FVector RegionCenter = Region.Bounds.GetCenter();
    TArray<FOverlapResult> Overlaps;

    const bool bHasOverlap = VolumeData->Settings.World->OverlapMultiByChannel(
        Overlaps, RegionCenter, FQuat::Identity, 
        VolumeData->Settings.GenerationSettings.CollisionChannel,
        FCollisionShape::MakeSphere(10.0f),
        VolumeData->Settings.GenerationSettings.CollisionQueryParameters);
    
    if (bHasOverlap)
    {
        for (const FOverlapResult& Overlap : Overlaps)
        {
            if (Overlap.GetComponent() == Component)
            {
                return true;
            }
        }
    }
    
    return false;
}



TArray<FVector> FNav3DTacticalReasoning::GenerateSamplePoints(
    const FNav3DRegion& Region) const
{
    TArray<FVector> Samples;
    
    if (!NavDataRef.IsValid())
    {
        return Samples;
    }
    
    const int32 SampleCount = FMath::Clamp(
        NavDataRef->TacticalSettings.MinSamplesPerRegion,
        1,
        NavDataRef->TacticalSettings.MaxSamplesPerRegion);
    
    const FVector Center = Region.Bounds.GetCenter();
    
    // Special case: if only 1 sample requested, use region center only
    if (SampleCount == 1)
    {
        Samples.Add(Center);
        UE_LOG(LogNav3D, VeryVerbose, TEXT("Tactical Sample: Using region center only: %s"), *Center.ToString());
        return Samples;
    }
    
    // For multiple samples, distribute within region bounds
    const FVector Extent = Region.Bounds.GetExtent() * 0.8f;  // Stay within 80% of bounds
    
    // First sample is always the center
    Samples.Add(Center);
    
    // Generate additional samples
    for (int32 i = 1; i < SampleCount; ++i)
    {
        FVector SamplePos = Center + FVector(
            FMath::FRandRange(-Extent.X, Extent.X),
            FMath::FRandRange(-Extent.Y, Extent.Y),
            FMath::FRandRange(-Extent.Z, Extent.Z));
        
        // Validate that the sample point is in navigable space (optional octree check)
        if (NavDataRef.IsValid())
        {
            if (const FNav3DVolumeNavigationData* NavData = 
                NavDataRef->GetVolumeNavigationDataContainingPoints({SamplePos}))
            {
                // Check if the sample position is actually free (not occluded)
                const float VoxelExtent = NavData->GetData().GetLeafNodes().GetLeafSubNodeExtent();
                if (NavData->IsPositionOccluded(SamplePos, VoxelExtent))
                {
                    UE_LOG(LogNav3D, VeryVerbose, TEXT("Tactical Sample: Generated sample point %s is occluded, skipping"), 
                           *SamplePos.ToString());
                    continue; // Skip this sample as it's in occluded space
                }
            }
        }
        
        UE_LOG(LogNav3D, VeryVerbose, TEXT("Tactical Sample: Generated valid sample point %s"), *SamplePos.ToString());
        Samples.Add(SamplePos);
    }
    
    // Ensure we have at least one sample (the center)
    if (Samples.Num() == 0)
    {
        Samples.Add(Center);
        UE_LOG(LogNav3D, Warning, TEXT("Tactical Sample: All generated samples were invalid, falling back to region center"));
    }
    
    return Samples;
}

// =============================================================================
// REGION PRUNING IMPLEMENTATION
// =============================================================================

TArray<int32> FNav3DTacticalReasoning::PruneRegionsToLimit(
    const FConsolidatedTacticalData& ConsolidatedData,
    const FBox& VolumeBounds,
    const int32 MaxRegions)
{
    const TArray<FNav3DRegion>& AllRegions = ConsolidatedData.AllLoadedRegions;
    
    if (AllRegions.Num() <= MaxRegions)
    {
        // No pruning needed
        TArray<int32> AllIds;
        for (const auto& Region : AllRegions)
        {
            AllIds.Add(Region.Id);
        }
        UE_LOG(LogNav3D, Log, TEXT("Pruning %d regions %d not required for volume %s (%d regions)"), 
           AllRegions.Num(), MaxRegions, *VolumeBounds.ToString(), AllRegions.Num());
        return AllIds;
    }
    
    UE_LOG(LogNav3D, Log, TEXT("Pruning %d regions down to %d for volume %s"), 
           AllRegions.Num(), MaxRegions, *VolumeBounds.ToString());
    
    // Step 1: Calculate tactical data for all regions
    TArray<FRegionPruningData> PruningData;
    CalculatePruningData(ConsolidatedData, VolumeBounds, PruningData);
    
    // Step 2: Multi-phase selection
    TSet<int32> SelectedRegions;
    
    // Phase 1: Spatial Coverage (20% of budget)
    EnsureSpatialCoverage(PruningData, VolumeBounds, MaxRegions * 0.2f, SelectedRegions);
    
    // Phase 2: Tactical Extremes (30% of budget)  
    SelectTacticalExtremes(PruningData, MaxRegions * 0.3f, SelectedRegions);
    
    // Phase 3: Fill remaining slots with highest scoring regions
    FillRemainingSlots(PruningData, MaxRegions, SelectedRegions);
    
    UE_LOG(LogNav3D, Log, TEXT("Region pruning complete: selected %d regions"), SelectedRegions.Num());
    
    return SelectedRegions.Array();
}

void FNav3DTacticalReasoning::CalculatePruningData(
    const FConsolidatedTacticalData& ConsolidatedData,
    const FBox& VolumeBounds,
    TArray<FRegionPruningData>& OutPruningData)
{
    OutPruningData.Reset();
    
    // Calculate elevation range for normalization
    float MinZ = MAX_flt, MaxZ = -MAX_flt;
    for (const auto& Region : ConsolidatedData.AllLoadedRegions)
    {
        const float RegionZ = Region.Bounds.GetCenter().Z;
        MinZ = FMath::Min(MinZ, RegionZ);
        MaxZ = FMath::Max(MaxZ, RegionZ);
    }
    const float ZRange = FMath::Max(1.0f, MaxZ - MinZ);
    
    for (const auto& Region : ConsolidatedData.AllLoadedRegions)
    {
        FRegionPruningData Data;
        Data.RegionId = Region.Id;
        Data.Position = Region.Bounds.GetCenter();
        Data.Volume = Region.Bounds.GetVolume();
        Data.ElevationRank = (Data.Position.Z - MinZ) / ZRange;
        Data.bIsBoundaryRegion = IsBoundaryRegion(Region, VolumeBounds);
        
        // Count visibility relationships
        if (const FRegionIdArray* VisibleIds = ConsolidatedData.RegionVisibility.Find(Region.Id))
        {
            Data.VisibilityCount = VisibleIds->Num();
        }
        
        // Count reverse visibility (how many can see this region)
        Data.VisibleFromCount = 0;
        for (const auto& VisibilityPair : ConsolidatedData.RegionVisibility)
        {
            if (VisibilityPair.Value.Contains(Region.Id))
            {
                Data.VisibleFromCount++;
            }
        }
        
        // Count adjacencies
        if (const FRegionIdArray* AdjacentIds = ConsolidatedData.RegionAdjacency.Find(Region.Id))
        {
            Data.AdjacencyCount = AdjacentIds->Num();
        }
        
        // Calculate if this is a chokepoint (low adjacency relative to neighbors)
        Data.bIsChokepoint = IsChokePoint(Region, ConsolidatedData);
        
        // Calculate distance variance (how unique this region's distance profile is)
        Data.DistanceVariance = CalculateDistanceVariance(Region, ConsolidatedData);
        
        // Calculate composite tactical score
        Data.TacticalScore = CalculateTacticalScore(Data);
        
        OutPruningData.Add(Data);
    }
}

void FNav3DTacticalReasoning::EnsureSpatialCoverage(
    const TArray<FRegionPruningData>& PruningData,
    const FBox& VolumeBounds,
    const int32 SpatialBudget,
    TSet<int32>& SelectedRegions)
{
    // Divide volume into spatial grid
    const int32 GridSize = FMath::CeilToInt(FMath::Pow(SpatialBudget, 1.0f/3.0f)); // Cube root
    const FVector CellSize = VolumeBounds.GetSize() / GridSize;
    
    UE_LOG(LogNav3D, Verbose, TEXT("Spatial coverage: using %dx%dx%d grid"), GridSize, GridSize, GridSize);
    
    // For each grid cell, find the best representative region
    for (int32 X = 0; X < GridSize; X++)
    {
        for (int32 Y = 0; Y < GridSize; Y++)
        {
            for (int32 Z = 0; Z < GridSize; Z++)
            {
                FVector CellCenter = VolumeBounds.Min + FVector(
                    (X + 0.5f) * CellSize.X,
                    (Y + 0.5f) * CellSize.Y,
                    (Z + 0.5f) * CellSize.Z
                );
                
                // Find closest region to this cell center
                float BestDistance = MAX_flt;
                int32 BestRegionId = -1;
                
                for (const auto& Data : PruningData)
                {
                    const float Distance = FVector::Dist(Data.Position, CellCenter);
                    if (Distance < BestDistance && !SelectedRegions.Contains(Data.RegionId))
                    {
                        BestDistance = Distance;
                        BestRegionId = Data.RegionId;
                    }
                }
                
                if (BestRegionId != -1)
                {
                    SelectedRegions.Add(BestRegionId);
                    if (SelectedRegions.Num() >= SpatialBudget) return;
                }
            }
        }
    }
}

void FNav3DTacticalReasoning::SelectTacticalExtremes(
    const TArray<FRegionPruningData>& PruningData,
    const int32 TacticalBudget,
    TSet<int32>& SelectedRegions)
{
    // Sort by different tactical criteria and pick extremes
    auto SortedData = PruningData;
    
    // Highest visibility (sniper positions)
    SortedData.Sort([](const FRegionPruningData& A, const FRegionPruningData& B) {
        return A.VisibilityCount > B.VisibilityCount;
    });
    AddTopCandidates(SortedData, TacticalBudget * 0.2f, SelectedRegions);
    
    // Lowest visibility (cover positions)
    SortedData.Sort([](const FRegionPruningData& A, const FRegionPruningData& B) {
        return A.VisibleFromCount < B.VisibleFromCount;
    });
    AddTopCandidates(SortedData, TacticalBudget * 0.2f, SelectedRegions);
    
    // Chokepoints and boundary regions
    for (const auto& Data : PruningData)
    {
        if ((Data.bIsChokepoint || Data.bIsBoundaryRegion) && 
            !SelectedRegions.Contains(Data.RegionId))
        {
            SelectedRegions.Add(Data.RegionId);
            if (SelectedRegions.Num() >= TacticalBudget) return;
        }
    }
    
    // Elevation extremes
    SortedData.Sort([](const FRegionPruningData& A, const FRegionPruningData& B) {
        return A.ElevationRank > B.ElevationRank;
    });
    AddTopCandidates(SortedData, TacticalBudget * 0.1f, SelectedRegions);
    
    SortedData.Sort([](const FRegionPruningData& A, const FRegionPruningData& B) {
        return A.ElevationRank < B.ElevationRank;
    });
    AddTopCandidates(SortedData, TacticalBudget * 0.1f, SelectedRegions);
}

void FNav3DTacticalReasoning::FillRemainingSlots(
    const TArray<FRegionPruningData>& PruningData,
    const int32 MaxRegions,
    TSet<int32>& SelectedRegions)
{
    // Sort by tactical score and fill remaining slots
    auto SortedData = PruningData;
    SortedData.Sort([](const FRegionPruningData& A, const FRegionPruningData& B) {
        return A.TacticalScore > B.TacticalScore;
    });
    
    for (const auto& Data : SortedData)
    {
        if (!SelectedRegions.Contains(Data.RegionId))
        {
            SelectedRegions.Add(Data.RegionId);
            if (SelectedRegions.Num() >= MaxRegions) break;
        }
    }
}

void FNav3DTacticalReasoning::AddTopCandidates(
    const TArray<FRegionPruningData>& SortedData,
    const int32 Count,
    TSet<int32>& SelectedRegions)
{
    int32 Added = 0;
    for (const auto& Data : SortedData)
    {
        if (!SelectedRegions.Contains(Data.RegionId))
        {
            SelectedRegions.Add(Data.RegionId);
            Added++;
            if (Added >= Count) break;
        }
    }
}

float FNav3DTacticalReasoning::CalculateTacticalScore(const FRegionPruningData& Data)
{
    float Score = 0.0f;
    
    // Visibility importance (both seeing and being seen)
    Score += FMath::Sqrt(static_cast<float>(Data.VisibilityCount)) * 10.0f;
    Score += Data.VisibleFromCount > 0 ? 100.0f / Data.VisibleFromCount : 50.0f; // Cover value
    
    // Connectivity importance
    Score += FMath::Sqrt(static_cast<float>(Data.AdjacencyCount)) * 5.0f;
    
    // Size diversity
    Score += FMath::Loge(FMath::Max(1.0f, Data.Volume)) * 2.0f;
    
    // Special tactical features
    if (Data.bIsChokepoint) Score += 20.0f;
    if (Data.bIsBoundaryRegion) Score += 15.0f;
    
    // Distance uniqueness
    Score += Data.DistanceVariance * 10.0f;
    
    return Score;
}

bool FNav3DTacticalReasoning::IsBoundaryRegion(const FNav3DRegion& Region, const FBox& VolumeBounds)
{
    constexpr float Tolerance = 100.0f;
    const FVector Center = Region.Bounds.GetCenter();
    
    return Center.X - VolumeBounds.Min.X < Tolerance ||
           VolumeBounds.Max.X - Center.X < Tolerance ||
           Center.Y - VolumeBounds.Min.Y < Tolerance ||
           VolumeBounds.Max.Y - Center.Y < Tolerance ||
           Center.Z - VolumeBounds.Min.Z < Tolerance ||
           VolumeBounds.Max.Z - Center.Z < Tolerance;
}

bool FNav3DTacticalReasoning::IsChokePoint(const FNav3DRegion& Region, const FConsolidatedTacticalData& Data)
{
    // A region is a choke-point if it has significantly fewer adjacencies than the average of its neighbors
    const FRegionIdArray* AdjacentIds = Data.RegionAdjacency.Find(Region.Id);
    if (!AdjacentIds || AdjacentIds->Num() == 0) return false;
    
    float NeighborAdjacencySum = 0.0f;
    int32 ValidNeighbors = 0;
    
    for (const int32 AdjacentId : AdjacentIds->GetArray())
    {
        if (const FRegionIdArray* NeighborAdjacencies = Data.RegionAdjacency.Find(AdjacentId))
        {
            NeighborAdjacencySum += NeighborAdjacencies->Num();
            ValidNeighbors++;
        }
    }
    
    if (ValidNeighbors == 0) return false;

    const float AvgNeighborAdjacency = NeighborAdjacencySum / ValidNeighbors;
    return AdjacentIds->Num() < AvgNeighborAdjacency * 0.6f; // 40% below average
}

float FNav3DTacticalReasoning::CalculateDistanceVariance(const FNav3DRegion& Region, const FConsolidatedTacticalData& Data)
{
    // Calculate how unique this region's distance profile is
    // Higher variance = more unique tactical positioning
    TArray<float> Distances;
    
    for (const auto& OtherRegion : Data.AllLoadedRegions)
    {
        if (OtherRegion.Id != Region.Id)
        {
            float Distance = FVector::Dist(Region.Bounds.GetCenter(), OtherRegion.Bounds.GetCenter());
            Distances.Add(Distance);
        }
    }
    
    if (Distances.Num() < 2) return 0.0f;
    
    // Calculate variance
    float Mean = 0.0f;
    for (const float Dist : Distances) Mean += Dist;
    Mean /= Distances.Num();
    
    float Variance = 0.0f;
    for (const float Dist : Distances) 
    {
        Variance += FMath::Square(Dist - Mean);
    }
    Variance /= Distances.Num();
    
    return FMath::Sqrt(Variance);
}

// =============================================================================
// DENSITY-FOCUSED PRUNING STRATEGY IMPLEMENTATION
// =============================================================================

TArray<int32> FDensityFocusedPruningStrategy::PruneRegionsToLimit(
    const FConsolidatedTacticalData& ConsolidatedData,
    const FBox& VolumeBounds,
    const TArray<ANav3DDataChunkActor*>& ChunkActors,
    const int32 MaxRegions)
{
    const TArray<FNav3DRegion>& AllRegions = ConsolidatedData.AllLoadedRegions;
    
    if (AllRegions.Num() <= MaxRegions)
    {
        TArray<int32> AllIds;
        for (const auto& Region : AllRegions)
        {
            AllIds.Add(Region.Id);
        }
        UE_LOG(LogNav3D, Log, TEXT("Density-focused pruning: %d regions <= %d limit, no pruning needed"), 
               AllRegions.Num(), MaxRegions);
        return AllIds;
    }
    
    UE_LOG(LogNav3D, Log, TEXT("Density-focused pruning: %d regions -> %d for volume %s"), 
           AllRegions.Num(), MaxRegions, *VolumeBounds.ToString());
    
    // Calculate density-focused tactical data
    TArray<FDensityRegionPruningData> PruningData;
    CalculateDensityPruningData(ConsolidatedData, VolumeBounds, ChunkActors, PruningData);
    
    // Multi-phase selection prioritizing tactical complexity
    TSet<int32> SelectedRegions;
    
    // Phase 1: High-density tactical zones (50% of budget)
    SelectHighDensityZones(PruningData, MaxRegions * 0.5f, SelectedRegions);
    
    // Phase 2: Tactical diversity (ensure variety) (30% of budget)
    EnsureTacticalDiversity(PruningData, MaxRegions * 0.3f, SelectedRegions);
    
    // Phase 3: Spatial coverage for larger ships (20% of budget)
    EnsureBasicSpatialCoverage(PruningData, VolumeBounds, MaxRegions * 0.2f, SelectedRegions);
    
    // Fill any remaining slots with highest scoring regions
    FillRemainingSlots(PruningData, MaxRegions, SelectedRegions);
    
    UE_LOG(LogNav3D, Log, TEXT("Density-focused pruning complete: selected %d regions"), SelectedRegions.Num());
    return SelectedRegions.Array();
}

void FDensityFocusedPruningStrategy::CalculateDensityPruningData(
    const FConsolidatedTacticalData& ConsolidatedData,
    const FBox& VolumeBounds,
    const TArray<ANav3DDataChunkActor*>& ChunkActors,
    TArray<FDensityRegionPruningData>& OutPruningData)
{
    OutPruningData.Reset();
    
    // Calculate elevation range for normalization
    float MinZ = MAX_flt, MaxZ = -MAX_flt;
    for (const auto& Region : ConsolidatedData.AllLoadedRegions)
    {
        const float RegionZ = Region.Bounds.GetCenter().Z;
        MinZ = FMath::Min(MinZ, RegionZ);
        MaxZ = FMath::Max(MaxZ, RegionZ);
    }
    const float ZRange = FMath::Max(1.0f, MaxZ - MinZ);
    
    for (const auto& Region : ConsolidatedData.AllLoadedRegions)
    {
        FDensityRegionPruningData Data;
        Data.RegionId = Region.Id;
        Data.Position = Region.Bounds.GetCenter();
        Data.Volume = Region.Bounds.GetVolume();
        Data.ElevationRank = (Data.Position.Z - MinZ) / ZRange;
        Data.bIsBoundaryRegion = IsBoundaryRegion(Region, VolumeBounds);
        
        // Calculate geometry density around this region
        Data.LocalGeometryDensity = CalculateLocalGeometryDensity(Region, ChunkActors);
        Data.GeometryProximity = CalculateGeometryProximity(Region, ChunkActors);
        
        // Calculate visibility complexity (how interesting the visibility pattern is)
        Data.VisibilityComplexity = CalculateVisibilityComplexity(Region, ConsolidatedData);
        
        // Calculate adjacency complexity
        Data.AdjacencyComplexity = CalculateAdjacencyComplexity(Region, ConsolidatedData);
        
        // Traditional metrics (for fallback)
        if (const FRegionIdArray* VisibleIds = ConsolidatedData.RegionVisibility.Find(Region.Id))
        {
            Data.VisibilityCount = VisibleIds->Num();
        }
        
        Data.VisibleFromCount = 0;
        for (const auto& VisibilityPair : ConsolidatedData.RegionVisibility)
        {
            if (VisibilityPair.Value.Contains(Region.Id))
            {
                Data.VisibleFromCount++;
            }
        }
        
        if (const FRegionIdArray* AdjacentIds = ConsolidatedData.RegionAdjacency.Find(Region.Id))
        {
            Data.AdjacencyCount = AdjacentIds->Num();
        }
        
        Data.bIsChokepoint = IsChokePoint(Region, ConsolidatedData);
        
        // Calculate composite tactical complexity score
        Data.TacticalComplexityScore = CalculateTacticalComplexityScore(Data);
        
        OutPruningData.Add(Data);
    }
    
    UE_LOG(LogNav3D, Verbose, TEXT("Calculated density data for %d regions"), OutPruningData.Num());
}

float FDensityFocusedPruningStrategy::CalculateLocalGeometryDensity(
    const FNav3DRegion& Region, 
    const TArray<ANav3DDataChunkActor*>& ChunkActors)
{
    // Find the chunk containing this region
    ANav3DDataChunkActor* ContainingChunk = nullptr;
    for (ANav3DDataChunkActor* Chunk : ChunkActors)
    {
        if (Chunk && Chunk->DataChunkActorBounds.IsInside(Region.Bounds.GetCenter()))
        {
            ContainingChunk = Chunk;
            break;
        }
    }
    
    if (!ContainingChunk)
    {
        return 0.0f;
    }
    
    // Sample the area around this region to determine geometry density
    const FVector RegionCenter = Region.Bounds.GetCenter();
    const float SampleRadius = FMath::Max(Region.Bounds.GetExtent().GetMax() * 2.0f, 500.0f);
    
    int32 OccludedSamples = 0;
    int32 ValidSamples = 0;
    
    for (int32 X = -1; X <= 1; X++)
    {
        for (int32 Y = -1; Y <= 1; Y++)
        {
            for (int32 Z = -1; Z <= 1; Z++)
            {
                FVector SamplePos = RegionCenter + FVector(
                    X * SampleRadius / 3.0f,
                    Y * SampleRadius / 3.0f, 
                    Z * SampleRadius / 3.0f
                );
                
                if (ContainingChunk->DataChunkActorBounds.IsInside(SamplePos))
                {
                    ValidSamples++;
                    
                    // Check if this sample point is in occluded space
                    // This requires access to the navigation data
                    if (ContainingChunk->Nav3DChunks.Num() > 0)
                    {
                        if (const FNav3DVolumeNavigationData* NavData = ContainingChunk->Nav3DChunks[0]->GetVolumeNavigationData())
                        {
                            if (const float VoxelExtent = NavData->GetData().GetLeafNodes().GetLeafSubNodeExtent();
                                NavData->IsPositionOccluded(SamplePos, VoxelExtent))
                            {
                                OccludedSamples++;
                            }
                        }
                    }
                }
            }
        }
    }
    
    return ValidSamples > 0 ? static_cast<float>(OccludedSamples) / ValidSamples : 0.0f;
}

float FDensityFocusedPruningStrategy::CalculateGeometryProximity(
    const FNav3DRegion& Region,
    const TArray<ANav3DDataChunkActor*>& ChunkActors)
{
    // Find the minimum distance to significant geometry (occluded voxels)
    const FVector RegionCenter = Region.Bounds.GetCenter();
    float MinDistanceToGeometry = MAX_flt;
    
    // Sample in expanding rings around the region
    for (const TArray SampleRadii = {100.0f, 250.0f, 500.0f, 1000.0f}; const float Radius : SampleRadii)
    {
        constexpr int32 SamplesPerRing = 16;
        for (int32 i = 0; i < SamplesPerRing; i++)
        {
            const float Angle = (2.0f * PI * i) / SamplesPerRing;
            FVector SamplePos = RegionCenter + FVector(
                FMath::Cos(Angle) * Radius,
                FMath::Sin(Angle) * Radius,
                0.0f // Keep at same elevation
            );
            
            // Find chunk containing this sample
            for (ANav3DDataChunkActor* Chunk : ChunkActors)
            {
                if (Chunk && Chunk->DataChunkActorBounds.IsInside(SamplePos) && Chunk->Nav3DChunks.Num() > 0)
                {
                    if (const FNav3DVolumeNavigationData* NavData = Chunk->Nav3DChunks[0]->GetVolumeNavigationData())
                    {
                        if (const float VoxelExtent = NavData->GetData().GetLeafNodes().GetLeafSubNodeExtent();
                            NavData->IsPositionOccluded(SamplePos, VoxelExtent))
                        {
                            MinDistanceToGeometry = FMath::Min(MinDistanceToGeometry, Radius);
                            break;
                        }
                    }
                }
            }
        }
        
        // If we found geometry at this radius, no need to sample further
        if (MinDistanceToGeometry < MAX_flt)
        {
            break;
        }
    }
    
    // Return inverted proximity (closer = higher score)
    return MinDistanceToGeometry < MAX_flt ? (2000.0f - MinDistanceToGeometry) / 2000.0f : 0.0f;
}

float FDensityFocusedPruningStrategy::CalculateVisibilityComplexity(
    const FNav3DRegion& Region,
    const FConsolidatedTacticalData& ConsolidatedData)
{
    // Measure how "interesting" the visibility pattern is
    const FRegionIdArray* VisibleIds = ConsolidatedData.RegionVisibility.Find(Region.Id);
    if (!VisibleIds || VisibleIds->Num() == 0)
    {
        return 0.0f;
    }
    
    // Calculate variance in distances to visible regions
    TArray<float> VisibleDistances;
    const FVector RegionCenter = Region.Bounds.GetCenter();
    
    for (const int32 VisibleId : VisibleIds->GetArray())
    {
        for (const FNav3DRegion& OtherRegion : ConsolidatedData.AllLoadedRegions)
        {
            if (OtherRegion.Id == VisibleId)
            {
                float Distance = FVector::Dist(RegionCenter, OtherRegion.Bounds.GetCenter());
                VisibleDistances.Add(Distance);
                break;
            }
        }
    }
    
    if (VisibleDistances.Num() < 2)
    {
        return static_cast<float>(VisibleDistances.Num());
    }
    
    // Calculate coefficient of variation (std dev / mean)
    float Mean = 0.0f;
    for (const float Dist : VisibleDistances) Mean += Dist;
    Mean /= VisibleDistances.Num();
    
    float Variance = 0.0f;
    for (const float Dist : VisibleDistances)
    {
        Variance += FMath::Square(Dist - Mean);
    }
    Variance /= VisibleDistances.Num();

    const float StdDev = FMath::Sqrt(Variance);
    const float CoefficientOfVariation = Mean > 0.0f ? StdDev / Mean : 0.0f;
    
    // Higher variation = more interesting visibility pattern
    return CoefficientOfVariation * VisibleDistances.Num();
}

float FDensityFocusedPruningStrategy::CalculateAdjacencyComplexity(
    const FNav3DRegion& Region,
    const FConsolidatedTacticalData& ConsolidatedData)
{
    const FRegionIdArray* AdjacentIds = ConsolidatedData.RegionAdjacency.Find(Region.Id);
    if (!AdjacentIds)
    {
        return 0.0f;
    }
    
    // Regions with moderate adjacency counts are often most tactically interesting
    const int32 AdjCount = AdjacentIds->Num();
    
    // Optimal adjacency is around 4-8 connections (not too isolated, not too open)
    constexpr float OptimalAdjacency = 6.0f;
    const float AdjacencyScore = 1.0f - FMath::Abs(AdjCount - OptimalAdjacency) / OptimalAdjacency;
    
    return FMath::Max(0.0f, AdjacencyScore) * AdjCount;
}

float FDensityFocusedPruningStrategy::CalculateTacticalComplexityScore(const FDensityRegionPruningData& Data)
{
    float Score = 0.0f;
    
    // Primary factors: geometry density and proximity (60% of score)
    Score += Data.LocalGeometryDensity * 300.0f;        // High weight for local density
    Score += Data.GeometryProximity * 200.0f;           // High weight for proximity to geometry
    
    // Secondary factors: visibility and adjacency complexity (30% of score)
    Score += Data.VisibilityComplexity * 50.0f;         // Interesting visibility patterns
    Score += Data.AdjacencyComplexity * 30.0f;          // Moderate adjacency complexity
    
    // Tertiary factors: traditional metrics (10% of score)
    Score += FMath::Sqrt(static_cast<float>(Data.VisibilityCount)) * 5.0f;
    Score += Data.VisibleFromCount > 0 ? 20.0f / Data.VisibleFromCount : 10.0f;
    
    // Special bonuses
    if (Data.bIsChokepoint) Score += 40.0f;             // Tactical bottlenecks
    if (Data.bIsBoundaryRegion) Score += 20.0f;         // Access points
    
    return Score;
}

void FDensityFocusedPruningStrategy::SelectHighDensityZones(
    const TArray<FDensityRegionPruningData>& PruningData,
    const int32 DensityBudget,
    TSet<int32>& SelectedRegions)
{
    // Sort by tactical complexity score and take the top regions
    auto SortedData = PruningData;
    SortedData.Sort([](const FDensityRegionPruningData& A, const FDensityRegionPruningData& B) {
        return A.TacticalComplexityScore > B.TacticalComplexityScore;
    });
    
    UE_LOG(LogNav3D, Verbose, TEXT("Selecting top %d high-density tactical zones"), DensityBudget);
    
    int32 Added = 0;
    for (const auto& Data : SortedData)
    {
        if (!SelectedRegions.Contains(Data.RegionId))
        {
            SelectedRegions.Add(Data.RegionId);
            Added++;
            
            UE_LOG(LogNav3D, VeryVerbose, TEXT("  Selected region %d (score %.1f, density %.2f, proximity %.2f)"), 
                   Data.RegionId, Data.TacticalComplexityScore, Data.LocalGeometryDensity, Data.GeometryProximity);
            
            if (Added >= DensityBudget) break;
        }
    }
}

void FDensityFocusedPruningStrategy::EnsureTacticalDiversity(
    const TArray<FDensityRegionPruningData>& PruningData,
    const int32 DiversityBudget,
    TSet<int32>& SelectedRegions)
{
    // Ensure we have a good mix of tactical types
    auto SortedData = PruningData;
    
    // Add some high-visibility regions
    SortedData.Sort([](const FDensityRegionPruningData& A, const FDensityRegionPruningData& B) {
        return A.VisibilityCount > B.VisibilityCount;
    });
    AddTopCandidates(SortedData, DiversityBudget * 0.3f, SelectedRegions);
    
    // Add some low-visibility regions (cover positions)
    SortedData.Sort([](const FDensityRegionPruningData& A, const FDensityRegionPruningData& B) {
        return A.VisibleFromCount < B.VisibleFromCount && A.VisibleFromCount > 0;
    });
    AddTopCandidates(SortedData, DiversityBudget * 0.3f, SelectedRegions);
    
    // Add elevation extremes for vertical tactics
    SortedData.Sort([](const FDensityRegionPruningData& A, const FDensityRegionPruningData& B) {
        return A.ElevationRank > B.ElevationRank;
    });
    AddTopCandidates(SortedData, DiversityBudget * 0.2f, SelectedRegions);
    
    SortedData.Sort([](const FDensityRegionPruningData& A, const FDensityRegionPruningData& B) {
        return A.ElevationRank < B.ElevationRank;
    });
    AddTopCandidates(SortedData, DiversityBudget * 0.2f, SelectedRegions);
}

void FDensityFocusedPruningStrategy::EnsureBasicSpatialCoverage(
    const TArray<FDensityRegionPruningData>& PruningData,
    const FBox& VolumeBounds,
    const int32 SpatialBudget,
    TSet<int32>& SelectedRegions)
{
    // Add larger regions for big ship navigation
    auto SortedData = PruningData;
    SortedData.Sort([](const FDensityRegionPruningData& A, const FDensityRegionPruningData& B) {
        return A.Volume > B.Volume;
    });
    
    AddTopCandidates(SortedData, SpatialBudget, SelectedRegions);
}

void FDensityFocusedPruningStrategy::FillRemainingSlots(
    const TArray<FDensityRegionPruningData>& PruningData,
    const int32 MaxRegions,
    TSet<int32>& SelectedRegions)
{
    auto SortedData = PruningData;
    SortedData.Sort([](const FDensityRegionPruningData& A, const FDensityRegionPruningData& B) {
        return A.TacticalComplexityScore > B.TacticalComplexityScore;
    });
    
    for (const auto& Data : SortedData)
    {
        if (!SelectedRegions.Contains(Data.RegionId))
        {
            SelectedRegions.Add(Data.RegionId);
            if (SelectedRegions.Num() >= MaxRegions) break;
        }
    }
}

void FDensityFocusedPruningStrategy::AddTopCandidates(
    const TArray<FDensityRegionPruningData>& SortedData,
    const int32 Count,
    TSet<int32>& SelectedRegions)
{
    int32 Added = 0;
    for (const auto& Data : SortedData)
    {
        if (!SelectedRegions.Contains(Data.RegionId))
        {
            SelectedRegions.Add(Data.RegionId);
            Added++;
            if (Added >= Count) break;
        }
    }
}

bool FDensityFocusedPruningStrategy::IsBoundaryRegion(const FNav3DRegion& Region, const FBox& VolumeBounds)
{
    constexpr float Tolerance = 100.0f;
    const FVector Center = Region.Bounds.GetCenter();
    
    return Center.X - VolumeBounds.Min.X < Tolerance ||
           VolumeBounds.Max.X - Center.X < Tolerance ||
           Center.Y - VolumeBounds.Min.Y < Tolerance ||
           VolumeBounds.Max.Y - Center.Y < Tolerance ||
           Center.Z - VolumeBounds.Min.Z < Tolerance ||
           VolumeBounds.Max.Z - Center.Z < Tolerance;
}

bool FDensityFocusedPruningStrategy::IsChokePoint(const FNav3DRegion& Region, const FConsolidatedTacticalData& Data)
{
    const FRegionIdArray* AdjacentIds = Data.RegionAdjacency.Find(Region.Id);
    if (!AdjacentIds || AdjacentIds->Num() == 0) return false;
    
    float NeighborAdjacencySum = 0.0f;
    int32 ValidNeighbors = 0;
    
    for (const int32 AdjacentId : AdjacentIds->GetArray())
    {
        if (const FRegionIdArray* NeighborAdjacencies = Data.RegionAdjacency.Find(AdjacentId))
        {
            NeighborAdjacencySum += NeighborAdjacencies->Num();
            ValidNeighbors++;
        }
    }
    
    if (ValidNeighbors == 0) return false;
    
    const float AvgNeighborAdjacency = NeighborAdjacencySum / ValidNeighbors;
    return AdjacentIds->Num() < AvgNeighborAdjacency * 0.6f;
}

// =============================================================================

bool FNav3DTacticalReasoning::FindBestLocationFromCompact(
    const FConsolidatedCompactTacticalData& CompactData,
    const FVector& StartPosition,
    const TArray<FVector>& ObserverPositions,
    ETacticalVisibility Visibility,
    ETacticalDistance DistancePreference,
    ETacticalRegion RegionPreference,
    bool bForceNewRegion,
    bool bUseRaycasting,
    TArray<FPositionCandidate>& OutCandidatePositions) const
{
    QUICK_SCOPE_CYCLE_COUNTER(STAT_Nav3D_FindBestLocationCompact);

    OutCandidatePositions.Reset();

    if (!NavDataRef.IsValid() || CompactData.IsEmpty() || ObserverPositions.Num() == 0)
    {
        return false;
    }

    auto ExtractVolumeIdFromRegion = [](const uint16 GlobalRegionId) -> uint16
    {
        return GlobalRegionId >> 8; // Upper 8 bits = volume ID
    };

    auto ExtractLocalRegionId = [](const uint16 GlobalRegionId) -> uint8
    {
        return static_cast<uint8>(GlobalRegionId & 0xFF); // Lower 8 bits = local region ID
    };

    auto RegionsMeetVisibility = [&](const uint16 CandidateRegionId, const TArray<uint16>& ObserverRegionIds) -> bool
    {
        const FCompactRegion* CandidateRegion = CompactData.AllLoadedRegions.Find(CandidateRegionId);
        if (!CandidateRegion)
        {
            return false;
        }

        const uint16 CandidateVolumeId = ExtractVolumeIdFromRegion(CandidateRegionId);
        const uint8 CandidateLocalId = ExtractLocalRegionId(CandidateRegionId);

        for (const uint16 ObserverRegionId : ObserverRegionIds)
        {
            const FCompactRegion* ObserverRegion = CompactData.AllLoadedRegions.Find(ObserverRegionId);
            if (!ObserverRegion)
            {
                continue;
            }

            const uint16 ObserverVolumeId = ExtractVolumeIdFromRegion(ObserverRegionId);
            const uint8 ObserverLocalId = ExtractLocalRegionId(ObserverRegionId);

            bool bRegionsVisible;

            if (CandidateVolumeId == ObserverVolumeId)
            {
                // === INTRA-VOLUME VISIBILITY ===
                // Same volume - regions are typically visible to each other unless blocked
                // Use distance heuristic as fallback for intra-volume
                const float Distance = FVector::Dist(CandidateRegion->Center, ObserverRegion->Center);
                
                // Get max intra-volume distance from navigation settings
                const UNav3DSettings* Nav3DSettings = GetDefault<UNav3DSettings>();
                const float MaxIntraVolumeDistance = Nav3DSettings ? Nav3DSettings->MaxVolumePartitionSize : 5000.0f;
                
                bRegionsVisible = (Distance <= MaxIntraVolumeDistance);
            }
            else
            {
                // === CROSS-VOLUME VISIBILITY ===
                // Use the compact volume visibility matrix
                if (const FVolumeRegionMatrix* VisibilityMatrix = CompactData.VolumeVisibilityData.Find(ObserverVolumeId))
                {
                    bRegionsVisible = VisibilityMatrix->HasReference(ObserverLocalId, CandidateVolumeId, CandidateLocalId);
                }
                else
                {
                    // No visibility data available - conservative approach
                    // Assume not visible for cross-volume unless explicitly defined
                    bRegionsVisible = false;
                }
            }

            // Apply visibility filter based on tactical requirements
            bool bMeetsVisibilityRequirement;
            switch (Visibility)
            {
                case ETacticalVisibility::TargetVisible:
                case ETacticalVisibility::MutuallyVisible:
                    bMeetsVisibilityRequirement = bRegionsVisible;
                    break;
                
                case ETacticalVisibility::TargetOccluded:
                case ETacticalVisibility::MutuallyOccluded:
                    bMeetsVisibilityRequirement = !bRegionsVisible;
                    break;
                
                default:
                    bMeetsVisibilityRequirement = true; // No visibility requirement
                    break;
            }

            if (!bMeetsVisibilityRequirement)
            {
                return false; // Failed visibility check for this observer
            }
        }

        return true; // Passed visibility check for all observers
    };

    auto EstimateRegionRadius = [](const FCompactRegion& Region) -> float
    {
        return Region.GetEstimatedRadius();
    };

    auto FindContainingRegionId = [&](const FVector& P) -> uint16
    {
        uint16 BestId = 0;
        float BestDist = MAX_flt;
        for (const auto& Pair : CompactData.AllLoadedRegions)
        {
            const uint16 Rid = Pair.Key;
            const FCompactRegion& R = Pair.Value;
            
            // === FILTER: Only consider regions from loaded chunks ===
            // Already filtered by AllLoadedRegions; do not apply additional filtering.
            
            const float Dist = FVector::Dist(P, R.Center);
            const float Radius = EstimateRegionRadius(R);
            if (Dist <= Radius && Dist < BestDist)
            {
                BestDist = Dist;
                BestId = Rid;
            }
        }
        return BestId;
    };

    auto GetAdjacentRegions = [&](const uint16 RegionId, TArray<uint16>& Out) -> void
    {
        Out.Reset();
        if (const uint64* Mask = CompactData.GlobalRegionAdjacency.Find(RegionId))
        {
            for (int32 Bit = 0; Bit < 64; ++Bit)
            {
                if ((*Mask) & (1ULL << Bit))
                {
                    const uint16 AdjacentId = static_cast<uint16>(Bit + 1);
                    
                    // === FILTER: Only include regions from loaded chunks ===
                    if (CompactData.AllLoadedRegions.Contains(AdjacentId))
                    {
                        Out.Add(AdjacentId);
                    }
                }
            }
        }
    };

    auto GenerateSamples = [&](const uint16 RegionId) -> TArray<FVector>
    {
        TArray<FVector> Samples;
        const FCompactRegion* R = CompactData.AllLoadedRegions.Find(RegionId);
        if (!R) { return Samples; }
        const float Radius = EstimateRegionRadius(*R);
        const FVector C = R->Center;
        Samples.Add(C); // center
        // 6 axial points inside bounds
        const float s = Radius * 0.6f;
        Samples.Add(C + FVector(+s, 0, 0));
        Samples.Add(C + FVector(-s, 0, 0));
        Samples.Add(C + FVector(0, +s, 0));
        Samples.Add(C + FVector(0, -s, 0));
        Samples.Add(C + FVector(0, 0, +s));
        Samples.Add(C + FVector(0, 0, -s));
        return Samples;
    };

    // 1) Locate start and observer regions (filtered by loaded chunks)
    UE_LOG(LogNav3D, Verbose, TEXT("FindBestLocationFromCompact: Looking for start position %s in %d regions"), 
        *StartPosition.ToString(), CompactData.AllLoadedRegions.Num());
    
    const uint16 StartRegionId = FindContainingRegionId(StartPosition);
    if (StartRegionId == 0)
    {
        UE_LOG(LogNav3D, Warning, TEXT("FindBestLocationFromCompact: Start position not in any loaded region"));
        UE_LOG(LogNav3D, Verbose, TEXT("Available regions:"));
        for (const auto& Pair : CompactData.AllLoadedRegions)
        {
            const uint16 Rid = Pair.Key;
            const FCompactRegion& R = Pair.Value;
            const float Dist = FVector::Dist(StartPosition, R.Center);
            const float Radius = EstimateRegionRadius(R);
            UE_LOG(LogNav3D, Verbose, TEXT("  Region %d: center %s, radius %.2f, dist %.2f, in range: %s"), 
                Rid, *R.Center.ToString(), Radius, Dist, (Dist <= Radius) ? TEXT("YES") : TEXT("NO"));
        }
        return false;
    }

    TArray<uint16> ObserverRegionIds;
    for (const FVector& ObsPos : ObserverPositions)
    {
        const uint16 ObsId = FindContainingRegionId(ObsPos);
        if (ObsId != 0)
        {
            ObserverRegionIds.AddUnique(ObsId);
        }
    }
    
    if (ObserverRegionIds.Num() == 0)
    {
        UE_LOG(LogNav3D, Warning, TEXT("FindBestLocationFromCompact: No observer positions in loaded regions"));
        return false;
    }

    UE_LOG(LogNav3D, Verbose, TEXT("FindBestLocationFromCompact: Start region %d, %d observer regions from loaded chunks"), 
           StartRegionId, ObserverRegionIds.Num());

    // 2) BFS over compact adjacency (filtered by loaded chunks)
    struct FNode { uint16 Id; float PathDist; };
    TArray<FNode> Queue;
    TSet<uint16> Visited;
    Queue.Add({ StartRegionId, 0.0f });
    Visited.Add(StartRegionId);

    const float MaxSearchDistance = NavDataRef->TacticalSettings.MaxCoverSearchDistance;

    TArray<FPositionCandidate> AllCandidates;

    while (Queue.Num() > 0)
    {
        const FNode Current = Queue[0];
        Queue.RemoveAt(0);

        if (Current.PathDist > MaxSearchDistance)
        {
            continue;
        }

        if (!(bForceNewRegion && Current.Id == StartRegionId))
        {
            // === USE PROPER COMPACT VISIBILITY CHECK ===
            if (RegionsMeetVisibility(Current.Id, ObserverRegionIds))
            {
                // Generate candidates
                for (const FVector& Pos : GenerateSamples(Current.Id))
                {
                    FPositionCandidate Cand;
                    Cand.RegionId = static_cast<int32>(Current.Id);
                    Cand.Position = Pos;
                    Cand.PathDistance = Current.PathDist;
                    Cand.DirectDistance = FVector::Dist(StartPosition, Pos);
                    if (const FCompactRegion* R = CompactData.AllLoadedRegions.Find(Current.Id))
                    {
                        Cand.RegionSize = R->GetWorldVolume();
                    }
                    Cand.Score = 1.0f;
                    AllCandidates.Add(Cand);
                }
            }
        }

        // Explore neighbors (filtered by loaded chunks)
        TArray<uint16> Adj;
        GetAdjacentRegions(Current.Id, Adj);
        for (uint16 Neighbor : Adj)
        {
            if (Visited.Contains(Neighbor)) { continue; }
            float Edge = 0.0f;
            if (const FCompactRegion* A = CompactData.AllLoadedRegions.Find(Current.Id))
            if (const FCompactRegion* B = CompactData.AllLoadedRegions.Find(Neighbor))
            {
                Edge = FVector::Dist(A->Center, B->Center);
            }
            const float NewDist = Current.PathDist + Edge;
            Queue.Add({ Neighbor, NewDist });
            Visited.Add(Neighbor);
        }
    }

    if (AllCandidates.Num() == 0)
    {
        UE_LOG(LogNav3D, Warning, TEXT("FindBestLocationFromCompact: No valid candidates found in loaded regions"));
        return false;
    }

    // 3) Optional raycast validation using existing raycaster
    if (bUseRaycasting)
    {
        for (FPositionCandidate& Cand : AllCandidates)
        {
            int32 Passed = 0;
            for (const FVector& ObsPos : ObserverPositions)
            {
                if (const FNav3DVolumeNavigationData* Nav = NavDataRef->GetVolumeNavigationDataContainingPoints({ ObsPos, Cand.Position }))
                {
                    if (const UNav3DRaycaster* Ray = NewObject<UNav3DRaycaster>())
                    {
                        FNav3DRaycastHit Hit; 
                        const bool bHit = Ray->Trace(*Nav, ObsPos, Cand.Position, Hit);
                        bool bMatches = (Visibility == ETacticalVisibility::TargetVisible || Visibility == ETacticalVisibility::MutuallyVisible) ? !bHit : bHit;
                        if (bMatches) { Passed++; }
                    }
                }
            }
            const float Ratio = static_cast<float>(Passed) / ObserverPositions.Num();
            Cand.Score *= Ratio;
        }
    }

    // 4) Score and rank
    float MinDist = FLT_MAX, MaxDist = 0.0f;
    float MinVol = FLT_MAX, MaxVol = 0.0f;
    for (const FPositionCandidate& C : AllCandidates)
    {
        MinDist = FMath::Min(MinDist, C.DirectDistance);
        MaxDist = FMath::Max(MaxDist, C.DirectDistance);
        MinVol = FMath::Min(MinVol, C.RegionSize);
        MaxVol = FMath::Max(MaxVol, C.RegionSize);
    }
    const float DistRange = FMath::Max(1.0f, MaxDist - MinDist);
    const float VolRange = FMath::Max(1.0f, MaxVol - MinVol);

    for (FPositionCandidate& C : AllCandidates)
    {
        float DistScore = 1.0f;
        switch (DistancePreference)
        {
            case ETacticalDistance::Closest:
                DistScore = 1.0f - (C.DirectDistance - MinDist) / DistRange; break;
            case ETacticalDistance::Furthest:
                DistScore = (C.DirectDistance - MinDist) / DistRange; break;
            case ETacticalDistance::Median:
            default:
                DistScore = 1.0f; break;
        }

        float VolScore = 1.0f;
        switch (RegionPreference)
        {
            case ETacticalRegion::Largest:
                VolScore = (C.RegionSize - MinVol) / VolRange; break;
            case ETacticalRegion::Smallest:
                VolScore = 1.0f - (C.RegionSize - MinVol) / VolRange; break;
            case ETacticalRegion::Median:
            default:
                VolScore = 1.0f; break;
        }

        C.Score *= DistScore * VolScore;
    }

    AllCandidates.Sort([](const FPositionCandidate& A, const FPositionCandidate& B)
    {
        return A.Score > B.Score;
    });

    const int32 MaxOut = FMath::Min(AllCandidates.Num(), 10);
    OutCandidatePositions.Reserve(MaxOut);
    for (int32 i = 0; i < MaxOut; ++i)
    {
        OutCandidatePositions.Add(AllCandidates[i]);
    }

    UE_LOG(LogNav3D, Verbose, TEXT("FindBestLocationFromCompact: Returning %d candidates"), OutCandidatePositions.Num());
    return OutCandidatePositions.Num() > 0;
}