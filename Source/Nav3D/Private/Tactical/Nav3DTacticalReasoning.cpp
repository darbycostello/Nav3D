#include "Tactical/Nav3DTacticalReasoning.h"

#include "LandscapeHeightfieldCollisionComponent.h"
#include "Nav3DData.h"
#include "Nav3DSettings.h"
#include "Nav3DUtils.h"
#include "Raycasting/Nav3DRaycaster.h"
#include "Tactical/Nav3DTacticalTypes.h"

FNav3DTacticalReasoning::FNav3DTacticalReasoning()
    : NextRegionId(0)
{
}

FNav3DTacticalReasoning::~FNav3DTacticalReasoning()
{
}

void FNav3DTacticalReasoning::Initialize(ANav3DData* NavData)
{
    NavDataRef = NavData;
    NextRegionId = 0;
}

void FNav3DTacticalReasoning::BuildTacticalData(const FBox& VolumeBounds)
{
    if (!NavDataRef.IsValid())
    {
        return;
    }
    const UNav3DSettings* Settings = UNav3DSettings::Get();

    // Find the volume containing these bounds
    FNav3DVolumeNavigationData* VolumeData = nullptr;
    TArray<FNav3DVolumeNavigationData>& VolumeNavData = NavDataRef->GetVolumeNavigationData();
    for (FNav3DVolumeNavigationData& Volume : VolumeNavData)
    {
        if (Volume.GetVolumeBounds().Intersect(VolumeBounds))
        {
            VolumeData = &Volume;
            break;
        }
    }

    if (!VolumeData)
    {
        return;
    }

    // Clear existing tactical data in the volume
    VolumeData->TacticalData.Regions.Empty();

    // Reset ID counter
    NextRegionId = 0;
    
    // Get settings
    const FNav3DData& OctreeData = VolumeData->GetData();
    const int32 LayerCount = OctreeData.GetLayerCount();
    const int32 MinRegionLayer = NavDataRef->TacticalSettings.MinRegioningLayer;
    const int32 MaxRegionLayer = NavDataRef->TacticalSettings.MaxRegioningLayer;
    
    UE_LOG(LogNav3D, Log, TEXT("Building tactical data for volume: MinRegionLayer=%d, MaxRegioningLayer=%d"),
          MinRegionLayer, MaxRegionLayer);
    
    // Process each layer
    for (int32 LayerIdx = MinRegionLayer; LayerIdx < LayerCount; ++LayerIdx)
    {
        UE_LOG(LogNav3D, Log, TEXT("Processing layer %d for tactical regions"), LayerIdx);
        
        // Extract free voxels with their grid coordinates
        TArray<TPair<uint64, FIntVector>> FreeVoxels = ExtractFreeVoxelsWithCoords(LayerIdx, VolumeData);
        
        if (FreeVoxels.Num() > 0)
        {
            TArray<FNav3DRegionBuilder> RegionBuilders;

            // Handle different layer cases based on MaxRegionLayer setting
            if (LayerIdx < MaxRegionLayer)
            {
                // Original processing for layers below MaxRegionLayer - merge as normal
                UE_LOG(LogNav3D, Verbose, TEXT("Layer %d: Standard region merging (below MaxRegionLayer)"), LayerIdx);
                
                // Step 1: Create a 3D grid representation for more efficient operations
                TMap<FIntVector, bool> VoxelGrid;
                
                // Populate the grid and find bounds
                for (const auto& VoxelPair : FreeVoxels)
                {
                    const FIntVector& Coord = VoxelPair.Value;
                    VoxelGrid.Add(Coord, true);
                }
                
                // Step 2: Build box regions using a greedy algorithm
                TArray<FBoxRegion> BoxRegions = BuildBoxRegions(VoxelGrid, LayerIdx);
                
                UE_LOG(LogNav3D, Verbose, TEXT("Built %d box regions in layer %d"), BoxRegions.Num(), LayerIdx);
                
                // Step 3: Convert to region builders
                for (const FBoxRegion& Box : BoxRegions)
                {
                    FNav3DRegionBuilder Builder = Box.ToRegionBuilder(FreeVoxels);
                    RegionBuilders.Add(Builder);
                }

                BuildVoxelLevelAdjacency(RegionBuilders);
            }
            else if (LayerIdx == MaxRegionLayer)
            {
                // For layer at MaxRegionLayer, create one region per voxel (no merging)
                UE_LOG(LogNav3D, Verbose, TEXT("Layer %d: Creating individual voxel regions (at MaxRegionLayer)"), LayerIdx);
                
                for (const auto& VoxelPair : FreeVoxels)
                {
                    FNav3DRegionBuilder Builder;
                    Builder.Id = NextRegionId++;
                    Builder.LayerIndex = LayerIdx;
                    Builder.MinCoord = VoxelPair.Value;
                    Builder.MaxCoord = VoxelPair.Value;
                    Builder.MortonCodes.Add(VoxelPair.Key);
                    
                    RegionBuilders.Add(Builder);
                }
                
                // Build voxel-level adjacency for individual regions
                BuildVoxelLevelAdjacency(RegionBuilders);
                
                UE_LOG(LogNav3D, Verbose, TEXT("Created %d individual voxel regions in layer %d (MaxRegionLayer)"), 
                      RegionBuilders.Num(), LayerIdx);
            }
            else // LayerIdx > MaxRegionLayer
            {
                // For layers above MaxRegionLayer, subdivide voxels
                UE_LOG(LogNav3D, Verbose, TEXT("Layer %d: Subdividing voxels (above MaxRegionLayer)"), LayerIdx);
                
                RegionBuilders = SubdivideVoxelsForLayer(FreeVoxels, LayerIdx, MaxRegionLayer, VolumeData);
                
                // Build voxel-level adjacency for subdivided regions
                BuildVoxelLevelAdjacency(RegionBuilders);
                
                UE_LOG(LogNav3D, Verbose, TEXT("Created %d subdivided regions in layer %d (above MaxRegionLayer)"), 
                      RegionBuilders.Num(), LayerIdx);
            }
            
            // Create layer regions from builders
            TArray<FNav3DRegion> LayerRegions;
            for (const FNav3DRegionBuilder& Builder : RegionBuilders)
            {
                LayerRegions.Add(Builder.ToRegion(VolumeData));
            }
            
            // NEW STEP: Verify regions against static geometry
            UE_LOG(LogNav3D, Log, TEXT("Verifying %d layer %d regions against static geometry"), 
                LayerRegions.Num(), LayerIdx);
            VerifyRegionsAgainstStaticGeometry(LayerRegions, VolumeData);
            
            // Add the verified regions directly to volume's tactical data
            VolumeData->TacticalData.Regions.Append(LayerRegions);
            
            UE_LOG(LogNav3D, Log, TEXT("Layer %d processing complete: %d free voxels → %d final verified regions"),
                   LayerIdx, FreeVoxels.Num(), LayerRegions.Num());
        }
        else
        {
            UE_LOG(LogNav3D, Log, TEXT("Layer %d has no free voxels, skipping"), LayerIdx);
        }
    }

    if (VolumeData->TacticalData.Regions.Num() > Settings->MaxRegions)
    {
        UE_LOG(LogNav3D, Warning, TEXT("Max regions exceeded. (%d regions / %d max) Cannot build tactical data."),
            VolumeData->TacticalData.Regions.Num(), Settings->MaxRegions);
        return;   
    }
    
    // Build adjacency graph and visibility sets for remaining regions
    UE_LOG(LogNav3D, Log, TEXT("Building adjacency graph and visibility sets for %d verified regions"), 
        VolumeData->TacticalData.Regions.Num());
    
    // Build adjacency graph directly on VolumeData's regions
    BuildAdjacencyGraph(VolumeData->TacticalData.Regions);
    
    // Build visibility sets directly on VolumeData's tactical data
    BuildVisibilitySets(VolumeData->TacticalData);
    
    NavDataRef->RequestDrawingUpdate();
    UE_LOG(LogNav3D, Log, TEXT("Tactical data built: %d regions"), VolumeData->TacticalData.Regions.Num());
}

TArray<FBoxRegion> FNav3DTacticalReasoning::BuildBoxRegions(const TMap<FIntVector, bool>& VoxelGrid, int32 LayerIndex)
{
    // Create a map to track which voxels are still available (not yet assigned to a region)
    TSet<FIntVector> AvailableVoxels;
    
    // Populate available voxels from the input grid
    for (auto It = VoxelGrid.CreateConstIterator(); It; ++It)
    {
        AvailableVoxels.Add(It.Key());
    }
    
    // Create array to store our results
    TArray<FBoxRegion> Results;
    
    // Keep processing until all voxels are assigned to regions
    while (AvailableVoxels.Num() > 0)
    {
        // Use deterministic ordering for seeds to ensure consistency
        TArray<FIntVector> AvailableArray = AvailableVoxels.Array();
        AvailableArray.Sort([](const FIntVector& A, const FIntVector& B) {
            // Sort by Z, then Y, then X
            if (A.Z != B.Z) return A.Z < B.Z;
            if (A.Y != B.Y) return A.Y < B.Y;
            return A.X < B.X;
        });
        
        // Get the first voxel as our seed
        FIntVector SeedCoord = AvailableArray[0];
        
        // Start with a 1x1x1 box
        FBoxRegion CurrentBox(NextRegionId++, SeedCoord, SeedCoord, LayerIndex);
        
        // We've now claimed this voxel
        AvailableVoxels.Remove(SeedCoord);
        
        // Function to check if a voxel at position is available
        auto IsVoxelAvailable = [&AvailableVoxels](const FIntVector& Pos) -> bool {
            return AvailableVoxels.Contains(Pos);
        };
        
        // Try to expand the box in all directions as much as possible
        bool bContinueExpanding = true;
        
        while (bContinueExpanding)
        {
            bContinueExpanding = false;
            
            // Try to expand in +X direction
            bool bCanExpandX = true;
            for (int32 Y = CurrentBox.Min.Y; Y <= CurrentBox.Max.Y && bCanExpandX; ++Y)
            {
                for (int32 Z = CurrentBox.Min.Z; Z <= CurrentBox.Max.Z && bCanExpandX; ++Z)
                {
                    FIntVector TestCoord(CurrentBox.Max.X + 1, Y, Z);
                    if (!IsVoxelAvailable(TestCoord))
                    {
                        bCanExpandX = false;
                    }
                }
            }
            
            if (bCanExpandX)
            {
                // Remove these voxels from the available set
                for (int32 Y = CurrentBox.Min.Y; Y <= CurrentBox.Max.Y; ++Y)
                {
                    for (int32 Z = CurrentBox.Min.Z; Z <= CurrentBox.Max.Z; ++Z)
                    {
                        AvailableVoxels.Remove(FIntVector(CurrentBox.Max.X + 1, Y, Z));
                    }
                }
                
                // Expand the box
                CurrentBox.Max.X += 1;
                bContinueExpanding = true;
            }
            
            // Try to expand in +Y direction
            bool bCanExpandY = true;
            for (int32 X = CurrentBox.Min.X; X <= CurrentBox.Max.X && bCanExpandY; ++X)
            {
                for (int32 Z = CurrentBox.Min.Z; Z <= CurrentBox.Max.Z && bCanExpandY; ++Z)
                {
                    FIntVector TestCoord(X, CurrentBox.Max.Y + 1, Z);
                    if (!IsVoxelAvailable(TestCoord))
                    {
                        bCanExpandY = false;
                    }
                }
            }
            
            if (bCanExpandY)
            {
                // Remove these voxels from the available set
                for (int32 X = CurrentBox.Min.X; X <= CurrentBox.Max.X; ++X)
                {
                    for (int32 Z = CurrentBox.Min.Z; Z <= CurrentBox.Max.Z; ++Z)
                    {
                        AvailableVoxels.Remove(FIntVector(X, CurrentBox.Max.Y + 1, Z));
                    }
                }
                
                // Expand the box
                CurrentBox.Max.Y += 1;
                bContinueExpanding = true;
            }
            
            // Try to expand in +Z direction
            bool bCanExpandZ = true;
            for (int32 X = CurrentBox.Min.X; X <= CurrentBox.Max.X && bCanExpandZ; ++X)
            {
                for (int32 Y = CurrentBox.Min.Y; Y <= CurrentBox.Max.Y && bCanExpandZ; ++Y)
                {
                    FIntVector TestCoord(X, Y, CurrentBox.Max.Z + 1);
                    if (!IsVoxelAvailable(TestCoord))
                    {
                        bCanExpandZ = false;
                    }
                }
            }
            
            if (bCanExpandZ)
            {
                // Remove these voxels from the available set
                for (int32 X = CurrentBox.Min.X; X <= CurrentBox.Max.X; ++X)
                {
                    for (int32 Y = CurrentBox.Min.Y; Y <= CurrentBox.Max.Y; ++Y)
                    {
                        AvailableVoxels.Remove(FIntVector(X, Y, CurrentBox.Max.Z + 1));
                    }
                }
                
                // Expand the box
                CurrentBox.Max.Z += 1;
                bContinueExpanding = true;
            }
            
            // -X direction
            bool bCanExpandNegX = true;
            for (int32 Y = CurrentBox.Min.Y; Y <= CurrentBox.Max.Y && bCanExpandNegX; ++Y)
            {
                for (int32 Z = CurrentBox.Min.Z; Z <= CurrentBox.Max.Z && bCanExpandNegX; ++Z)
                {
                    FIntVector TestCoord(CurrentBox.Min.X - 1, Y, Z);
                    if (!IsVoxelAvailable(TestCoord))
                    {
                        bCanExpandNegX = false;
                    }
                }
            }
            
            if (bCanExpandNegX)
            {
                // Remove these voxels from the available set
                for (int32 Y = CurrentBox.Min.Y; Y <= CurrentBox.Max.Y; ++Y)
                {
                    for (int32 Z = CurrentBox.Min.Z; Z <= CurrentBox.Max.Z; ++Z)
                    {
                        AvailableVoxels.Remove(FIntVector(CurrentBox.Min.X - 1, Y, Z));
                    }
                }
                
                // Expand the box
                CurrentBox.Min.X -= 1;
                bContinueExpanding = true;
            }
            
            // -Y direction
            bool bCanExpandNegY = true;
            for (int32 X = CurrentBox.Min.X; X <= CurrentBox.Max.X && bCanExpandNegY; ++X)
            {
                for (int32 Z = CurrentBox.Min.Z; Z <= CurrentBox.Max.Z && bCanExpandNegY; ++Z)
                {
                    FIntVector TestCoord(X, CurrentBox.Min.Y - 1, Z);
                    if (!IsVoxelAvailable(TestCoord))
                    {
                        bCanExpandNegY = false;
                    }
                }
            }
            
            if (bCanExpandNegY)
            {
                // Remove these voxels from the available set
                for (int32 X = CurrentBox.Min.X; X <= CurrentBox.Max.X; ++X)
                {
                    for (int32 Z = CurrentBox.Min.Z; Z <= CurrentBox.Max.Z; ++Z)
                    {
                        AvailableVoxels.Remove(FIntVector(X, CurrentBox.Min.Y - 1, Z));
                    }
                }
                
                // Expand the box
                CurrentBox.Min.Y -= 1;
                bContinueExpanding = true;
            }
            
            // -Z direction
            bool bCanExpandNegZ = true;
            for (int32 X = CurrentBox.Min.X; X <= CurrentBox.Max.X && bCanExpandNegZ; ++X)
            {
                for (int32 Y = CurrentBox.Min.Y; Y <= CurrentBox.Max.Y && bCanExpandNegZ; ++Y)
                {
                    FIntVector TestCoord(X, Y, CurrentBox.Min.Z - 1);
                    if (!IsVoxelAvailable(TestCoord))
                    {
                        bCanExpandNegZ = false;
                    }
                }
            }
            
            if (bCanExpandNegZ)
            {
                // Remove these voxels from the available set
                for (int32 X = CurrentBox.Min.X; X <= CurrentBox.Max.X; ++X)
                {
                    for (int32 Y = CurrentBox.Min.Y; Y <= CurrentBox.Max.Y; ++Y)
                    {
                        AvailableVoxels.Remove(FIntVector(X, Y, CurrentBox.Min.Z - 1));
                    }
                }
                
                // Expand the box
                CurrentBox.Min.Z -= 1;
                bContinueExpanding = true;
            }
        }
        
        // Add this box to our results
        Results.Add(CurrentBox);
        
        // Debug log for larger regions
        if (CurrentBox.GetVolume() > 10)
        {
            UE_LOG(LogNav3D, Verbose, TEXT("Created box region with volume %d: (%d,%d,%d) to (%d,%d,%d)"),
                  CurrentBox.GetVolume(),
                  CurrentBox.Min.X, CurrentBox.Min.Y, CurrentBox.Min.Z,
                  CurrentBox.Max.X, CurrentBox.Max.Y, CurrentBox.Max.Z);
        }
    }
    
    return Results;
}

TArray<FVector> FNav3DTacticalReasoning::ExtractFreeVoxelsInLayer(const int32 LayerIndex) const
{
    TArray<FVector> FreeVoxels;

    if (!NavDataRef.IsValid())
    {
        return FreeVoxels;
    }

    const TArray<FNav3DVolumeNavigationData>& VolumeNavData = NavDataRef->GetVolumeNavigationData();
    
    for (const FNav3DVolumeNavigationData& NavVolume : VolumeNavData)
    {
        const FNav3DData& OctreeData = NavVolume.GetData();
        
        if (LayerIndex >= OctreeData.GetLayerCount())
        {
            continue;
        }

        const FNav3DLayer& Layer = OctreeData.GetLayer(LayerIndex);

        for (const TArray<FNav3DNode>& Nodes = Layer.GetNodes(); const FNav3DNode& Node : Nodes)
        {
            // Nodes with children are occupied
            if (!Node.HasChildren())
            {
                FVector NodePosition;
                
                if (LayerIndex == 0)
                {
                    // For leaf layer
                    NodePosition = NavVolume.GetLeafNodePositionFromMortonCode(Node.MortonCode);
                }
                else
                {
                    // For other layers
                    NodePosition = NavVolume.GetNodePositionFromLayerAndMortonCode(LayerIndex, Node.MortonCode);
                }

                FreeVoxels.Add(NodePosition);
            }
        }
    }

    return FreeVoxels;
}

bool FNav3DTacticalReasoning::IsPositionNavigable(const FNav3DTacticalData& TargetData, const FVector& Position, const int32 LayerIndex)
{
    // Simple heuristic: If the position is inside a region, consider it navigable
    for (const FNav3DRegion& Region : TargetData.Regions)
    {
        if (Region.LayerIndex == LayerIndex && Region.Bounds.IsInside(Position))
        {
            return true;
        }
    }
    
    // Not in any region, so not navigable
    return false;
}

void FNav3DTacticalReasoning::BuildVisibilitySets(FNav3DTacticalData& TargetData) const
{
    // Prepare region IDs array for parallelization
    TArray<int32> RegionIds;
    for (const FNav3DRegion& Region : TargetData.Regions)
    {
        RegionIds.Add(Region.Id);
    }
    
    // Thread-safe result collection
    FCriticalSection ResultLock;
    TMap<int32, TArray<int32>> VisibilityResults;
    
    // Parallel processing - evaluate visibility from each region
    ParallelFor(RegionIds.Num(), [&](const int32 IndexA) {
        const int32 ViewerRegionId = RegionIds[IndexA];
        const FNav3DRegion* ViewerRegion = GetRegionById(TargetData, ViewerRegionId);
        
        if (!ViewerRegion) return;
        
        // Generate samples for viewer region
        TArray<FVector> ViewerSamples = GenerateSamplePoints(TargetData, *ViewerRegion);
        
        // Local results for this region
        TArray<int32> LocalVisibleRegions;
        
        // Self-visibility (always visible to self)
        LocalVisibleRegions.Add(ViewerRegionId);
        
        // Process all other regions
        for (int32 IndexB = 0; IndexB < RegionIds.Num(); ++IndexB) {
            if (IndexA == IndexB) continue;

            const int32 TargetRegionId = RegionIds[IndexB];
            const FNav3DRegion* TargetRegion = GetRegionById(TargetData, TargetRegionId);
            
            if (!TargetRegion) continue;
            
            // Generate samples for target region
            TArray<FVector> TargetSamples = GenerateSamplePoints(TargetData, *TargetRegion);
            
            // Count visible samples
            int32 VisiblePairs = 0;
            const int32 TotalPairs = ViewerSamples.Num() * TargetSamples.Num();
            
            // Check each sample pair for visibility
            for (const FVector& ViewerPos : ViewerSamples) {
                for (const FVector& TargetPos : TargetSamples) {
                    
                    bool bHasLineOfSight = false;
                    
                    if (NavDataRef.IsValid())
                    {
                        if (const auto* Raycaster = NewObject<UNav3DRaycaster>())
                        {
                            // Find the volume containing both points
                            if (const auto* NavData = NavDataRef->GetVolumeNavigationDataContainingPoints({ViewerPos, TargetPos}))
                            {
                                bool bHit;
                                FNav3DRaycastHit Hit;

                                // If more than one occlusion is required for a successful hit then use a counting trace.
                                if (NavDataRef->TacticalSettings.MinOcclusions > 1)
                                {
                                    // Perform the full occlusion-counting raycast
                                    bHit = Raycaster->TraceCountingOccludedVoxels(*NavData, ViewerPos, TargetPos, Hit);
                                    bHasLineOfSight = !bHit || Hit.OccludedVoxelCount < NavDataRef->TacticalSettings.MinOcclusions;
                                }
                                else
                                {
                                    // Perform a simple raycast which returns as soon as it hits something
                                    bHit = Raycaster->Trace(*NavData, ViewerPos, TargetPos, Hit);
                                    bHasLineOfSight = !bHit;
                                }
                                
                                UE_LOG(LogNav3D, VeryVerbose, TEXT("Raycast hit: %s, Occlusions: %d, MinRequired: %d, HasLineOfSight: %s"),
                                       bHit ? TEXT("true") : TEXT("false"),
                                       Hit.OccludedVoxelCount, 
                                       NavDataRef->TacticalSettings.MinOcclusions,
                                       bHasLineOfSight ? TEXT("true") : TEXT("false"));
                            }
                        }
                    }
                    
                    if (bHasLineOfSight) {
                        VisiblePairs++;
                    }
                }
            }

            const float RawScore = static_cast<float>(VisiblePairs) / FMath::Max(1, TotalPairs);
            constexpr float MinVisibility = 0.8f; // Visibility scores are only useful above 0.8
            constexpr float MaxVisibility = 1.0f; // Upper bound will always be 1.0
            const float NormalizedValue = (RawScore - MinVisibility) / (MaxVisibility - MinVisibility);

            // Cubic remapping for higher sensitivity near 1.0
            const float RemappedScore = FMath::Pow(NormalizedValue, 3.0f);
            const float VisibilityScore = FMath::Clamp(RemappedScore, 0.0f, 1.0f);
            const float VisibilityThreshold = NavDataRef->TacticalSettings.VisibilityScoreThreshold;
            
            // Store if visibility exceeds threshold
            if (VisibilityScore > VisibilityThreshold) {
                LocalVisibleRegions.Add(TargetRegionId);
            }
        }
        
        // Thread-safe update of results
        {
            FScopeLock Lock(&ResultLock);
            VisibilityResults.Add(ViewerRegionId, LocalVisibleRegions);
        }
    });
    
    // Store all results in tactical data, region by region
    for (auto& Pair : VisibilityResults)
    {
        const int32 ViewerRegionId = Pair.Key;
        const TArray<int32>& VisibleRegionIds = Pair.Value;
        
        // Add each visible region to the viewer's visibility set
        for (const int32 VisibleRegionId : VisibleRegionIds) {
            TargetData.AddToVisibilitySet(ViewerRegionId, VisibleRegionId);
        }
    }
    
    UE_LOG(LogNav3D, Verbose, TEXT("Completed visibility calculation: %d regions processed"), VisibilityResults.Num());
}

FVector FNav3DTacticalReasoning::GetRandomPointInRegion(const FNav3DRegion& Region)
{
    const FVector Center = Region.Bounds.GetCenter();
    const FVector Extent = (Region.Bounds.Max - Region.Bounds.Min) * 0.45f; // Slight inset from edges
    
    return FVector(
        Center.X + FMath::FRandRange(-Extent.X, Extent.X),
        Center.Y + FMath::FRandRange(-Extent.Y, Extent.Y),
        Center.Z + FMath::FRandRange(-Extent.Z, Extent.Z)
    );
}

const FNav3DRegion* FNav3DTacticalReasoning::GetRegionById(const FNav3DTacticalData& TargetData, const int32 RegionId)
{
    for (const FNav3DRegion& Region : TargetData.Regions)
    {
        if (Region.Id == RegionId)
        {
            return &Region;
        }
    }
    return nullptr;
}

TArray<FVector> FNav3DTacticalReasoning::GenerateSamplePoints(const FNav3DTacticalData& TargetData, const FNav3DRegion& Region) const
{
    TArray<FVector> Samples;
    
    if (!NavDataRef.IsValid())
    {
        return Samples;
    }
    
    // Base count from settings
    const int32 BaseCount = NavDataRef->TacticalSettings.MinSamplesPerRegion;
    const int32 MaxCount = NavDataRef->TacticalSettings.MaxSamplesPerRegion;
    
    // Calculate volume metrics
    const float Volume = Region.Bounds.GetVolume();
    const float DensityFactor = NavDataRef->TacticalSettings.RegionSampleDensityFactor;
    
    // Scale sample count with region size
    const float AverageVolume = CalculateAverageRegionVolume(TargetData);
    int32 SampleCount = FMath::Max(4, FMath::RoundToInt(BaseCount * FMath::Pow(Volume / AverageVolume, DensityFactor)));
    
    // Cap to maximum
    SampleCount = FMath::Min(SampleCount, MaxCount);
    
    // Calculate region dimensions
    const FVector Center = Region.Bounds.GetCenter();
    const FVector Extent = (Region.Bounds.Max - Region.Bounds.Min) * 0.45f; // Slight inset
    
    // Generate stratified random samples within the region
    for (int32 i = 0; i < SampleCount; ++i)
    {
        // Uniform random distribution within region
        FVector Offset(
            FMath::FRandRange(-Extent.X, Extent.X),
            FMath::FRandRange(-Extent.Y, Extent.Y),
            FMath::FRandRange(-Extent.Z, Extent.Z)
        );

        // Optional: Validate sample is actually navigable
        if (FVector SamplePos = Center + Offset; IsPositionNavigable(TargetData, SamplePos, Region.LayerIndex))
        {
            Samples.Add(SamplePos);
        }
    }
    
    // Ensure we have at least one sample by adding the center if needed
    if (Samples.Num() == 0)
    {
        Samples.Add(Center);
    }
    
    return Samples;
}

float FNav3DTacticalReasoning::CalculateAverageRegionVolume(const FNav3DTacticalData& TargetData)
{
    if (TargetData.Regions.Num() == 0)
        return 100000.0f; // Default if no regions
        
    float TotalVolume = 0.0f;
    for (const FNav3DRegion& Region : TargetData.Regions)
    {
        TotalVolume += Region.Bounds.GetVolume();
    }
    
    return TotalVolume / TargetData.Regions.Num();
}

TArray<TPair<uint64, FIntVector>> FNav3DTacticalReasoning::ExtractFreeVoxelsWithCoords(const int32 LayerIndex, const FNav3DVolumeNavigationData* VolumeData)
{
    TArray<TPair<uint64, FIntVector>> FreeVoxels;
    
    if (!VolumeData)
    {
        return FreeVoxels;
    }
    
    const FNav3DData& OctreeData = VolumeData->GetData();
    
    if (LayerIndex >= OctreeData.GetLayerCount())
    {
        return FreeVoxels;
    }
    
    const FNav3DLayer& Layer = OctreeData.GetLayer(LayerIndex);
    
    for (const FNav3DNode& Node : Layer.GetNodes())
    {
        // Nodes with children are occupied
        if (!Node.HasChildren())
        {
            MortonCode MortonCode = Node.MortonCode;
            FIntVector Coord = FIntVector(FNav3DUtils::GetVectorFromMortonCode(MortonCode));
            
            // Only include voxels within the volume bounds if we have bounds
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
    
    UE_LOG(LogNav3D, Verbose, TEXT("Extracted %d free voxels from layer %d"), FreeVoxels.Num(), LayerIndex);
    return FreeVoxels;
}

TArray<FNav3DRegionBuilder> FNav3DTacticalReasoning::BuildInitialRegions(const TArray<TPair<uint64, FIntVector>>& FreeVoxels, int32 LayerIndex)
{
    TArray<FNav3DRegionBuilder> Regions;
    TSet<uint64> VisitedCodes;
    
    // For each unvisited voxel, start a new region
    for (const auto& VoxelPair : FreeVoxels)
    {
        if (VisitedCodes.Contains(VoxelPair.Key))
            continue;
            
        // Start a new region
        FNav3DRegionBuilder Region;
        Region.Id = NextRegionId++;
        Region.LayerIndex = LayerIndex;
        Region.MinCoord = VoxelPair.Value;
        Region.MaxCoord = VoxelPair.Value;
        
        // Use a queue for flood fill
        TQueue<uint64> Queue;
        Queue.Enqueue(VoxelPair.Key);
        VisitedCodes.Add(VoxelPair.Key);
        
        // Create map for quick lookup during BFS
        TMap<uint64, FIntVector> MortonToCoord;
        for (const auto& Voxel : FreeVoxels)
        {
            MortonToCoord.Add(Voxel.Key, Voxel.Value);
        }
        
        while (!Queue.IsEmpty())
        {
            uint64 CurrentCode;
            Queue.Dequeue(CurrentCode);
            Region.MortonCodes.Add(CurrentCode);
            
            // Get the grid coordinates
            FIntVector CurrentCoord = MortonToCoord[CurrentCode];
            
            // Update region bounds
            Region.MinCoord = FIntVector(
                FMath::Min(Region.MinCoord.X, CurrentCoord.X),
                FMath::Min(Region.MinCoord.Y, CurrentCoord.Y),
                FMath::Min(Region.MinCoord.Z, CurrentCoord.Z)
            );
            
            Region.MaxCoord = FIntVector(
                FMath::Max(Region.MaxCoord.X, CurrentCoord.X),
                FMath::Max(Region.MaxCoord.Y, CurrentCoord.Y),
                FMath::Max(Region.MaxCoord.Z, CurrentCoord.Z)
            );
            
            // Check all 6 neighbors (orthogonal directions)
            for (int32 Dir = 0; Dir < 6; Dir++)
            {
                FIntVector NeighborCoord = CurrentCoord + GNeighbourDirections[Dir];
                uint64 NeighborCode = FNav3DUtils::GetMortonCodeFromVector(NeighborCoord);
                
                // Check if this neighbor exists in our free voxels and hasn't been visited
                if (MortonToCoord.Contains(NeighborCode) && !VisitedCodes.Contains(NeighborCode))
                {
                    Queue.Enqueue(NeighborCode);
                    VisitedCodes.Add(NeighborCode);
                }
            }
        }
        
        // Only add non-empty regions
        if (Region.MortonCodes.Num() > 0)
        {
            Regions.Add(Region);
        }
    }
    
    UE_LOG(LogNav3D, Verbose, TEXT("Created %d initial connected regions in layer %d"), Regions.Num(), LayerIndex);
    return Regions;
} 

TArray<FNav3DRegionBuilder> FNav3DTacticalReasoning::RefineRegionsToBoxes(const TArray<FNav3DRegionBuilder>& InitialRegions)
{
    TArray<FNav3DRegionBuilder> BoxRegions;
    
    for (const FNav3DRegionBuilder& Region : InitialRegions)
    {
        // Calculate the region's bounding box in grid coordinates
        FIntVector MinCoord = Region.MinCoord;
        FIntVector MaxCoord = Region.MaxCoord;
        const FIntVector Size = MaxCoord - MinCoord + FIntVector(1, 1, 1);
        
        // Check if the box is already filled completely with voxels
        const int32 ExpectedVoxelCount = Size.X * Size.Y * Size.Z;
        
        if (Region.MortonCodes.Num() == ExpectedVoxelCount)
        {
            // Perfect box - add as is
            UE_LOG(LogNav3D, VeryVerbose, TEXT("Region %d is already a perfect box (%d,%d,%d) to (%d,%d,%d), keeping as is"), 
                   Region.Id, MinCoord.X, MinCoord.Y, MinCoord.Z, MaxCoord.X, MaxCoord.Y, MaxCoord.Z);
            BoxRegions.Add(Region);
            continue;
        }
        
        // Not a perfect box - use recursive binary space partitioning
        UE_LOG(LogNav3D, VeryVerbose, TEXT("Region %d needs refinement (%d voxels of expected %d), running partition"), 
               Region.Id, Region.MortonCodes.Num(), ExpectedVoxelCount);
        TArray<FNav3DRegionBuilder> PartitionedRegions = PartitionRegion(Region);
        BoxRegions.Append(PartitionedRegions);
    }
    
    UE_LOG(LogNav3D, Verbose, TEXT("Refined %d initial regions into %d box regions"), 
           InitialRegions.Num(), BoxRegions.Num());
    return BoxRegions;
}

TArray<FNav3DRegionBuilder> FNav3DTacticalReasoning::PartitionRegion(const FNav3DRegionBuilder& Region)
{
    TArray<FNav3DRegionBuilder> Results;
    
    // Get dimensions
    FIntVector MinCoord = Region.MinCoord;
    FIntVector MaxCoord = Region.MaxCoord;
    FIntVector Size = MaxCoord - MinCoord + FIntVector(1, 1, 1);
    
    // If region is small enough or efficiently filled, don't partition further
    if (Size.X <= 2 && Size.Y <= 2 && Size.Z <= 2)
    {
        // Create individual box regions for each voxel
        for (const uint64 Code : Region.MortonCodes)
        {
            FNav3DRegionBuilder VoxelRegion;
            VoxelRegion.Id = NextRegionId++;
            VoxelRegion.LayerIndex = Region.LayerIndex;
            
            FIntVector Coord = FIntVector(FNav3DUtils::GetVectorFromMortonCode(Code));
            VoxelRegion.MinCoord = Coord;
            VoxelRegion.MaxCoord = Coord;
            VoxelRegion.MortonCodes.Add(Code);
            
            Results.Add(VoxelRegion);
        }
        
        UE_LOG(LogNav3D, VeryVerbose, TEXT("Partition: Region too small, created %d individual voxel regions"), 
               Results.Num());
        return Results;
    }
    
    // Also avoid partitioning if the region is already well-filled (>80% of expected voxels)
    int32 ExpectedVoxelCount = Size.X * Size.Y * Size.Z;
    float FillRatio = static_cast<float>(Region.MortonCodes.Num()) / static_cast<float>(ExpectedVoxelCount);
    
    if (FillRatio >= 0.8f)
    {
        // The region is mostly filled, keep it as a single box
        FNav3DRegionBuilder BoxRegion = Region;
        
        UE_LOG(LogNav3D, VeryVerbose, TEXT("Partition: Region well-filled (%.1f%%), keeping as single box"), 
               FillRatio * 100.0f);
        Results.Add(BoxRegion);
        return Results;
    }
    
    // Find largest dimension
    int32 LargestDim = 0;
    int32 LargestSize = Size.X;
    
    if (Size.Y > LargestSize)
    {
        LargestDim = 1;
        LargestSize = Size.Y;
    }
    
    if (Size.Z > LargestSize)
    {
        LargestDim = 2;
        LargestSize = Size.Z;
    }
    
    // Calculate split position - try to split at midpoint
    int32 SplitPos = MinCoord[LargestDim] + (LargestSize / 2);
    
    // Create two new regions
    FNav3DRegionBuilder LowerRegion;
    LowerRegion.Id = NextRegionId++;
    LowerRegion.LayerIndex = Region.LayerIndex;
    LowerRegion.MinCoord = MinCoord;
    LowerRegion.MaxCoord = MaxCoord;
    LowerRegion.MaxCoord[LargestDim] = SplitPos - 1;
    
    FNav3DRegionBuilder UpperRegion;
    UpperRegion.Id = NextRegionId++;
    UpperRegion.LayerIndex = Region.LayerIndex;
    UpperRegion.MinCoord = MinCoord;
    UpperRegion.MinCoord[LargestDim] = SplitPos;
    UpperRegion.MaxCoord = MaxCoord;
    
    // Distribute voxels to the appropriate region
    for (const uint64 Code : Region.MortonCodes)
    {
        if (FIntVector Coord = FIntVector(FNav3DUtils::GetVectorFromMortonCode(Code)); Coord[LargestDim] < SplitPos)
        {
            LowerRegion.MortonCodes.Add(Code);
        }
        else
        {
            UpperRegion.MortonCodes.Add(Code);
        }
    }
    
    UE_LOG(LogNav3D, VeryVerbose, TEXT("Partition: Split along dim %d at pos %d, lower has %d voxels, upper has %d voxels"), 
           LargestDim, SplitPos, LowerRegion.MortonCodes.Num(), UpperRegion.MortonCodes.Num());
    
    // Recursively partition each half if they have voxels
    if (LowerRegion.MortonCodes.Num() > 0)
    {
        Results.Append(PartitionRegion(LowerRegion));
    }
    
    if (UpperRegion.MortonCodes.Num() > 0)
    {
        Results.Append(PartitionRegion(UpperRegion));
    }
    
    return Results;
}

TArray<FNav3DRegionBuilder> FNav3DTacticalReasoning::MergeBoxRegions(const TArray<FNav3DRegionBuilder>& BoxRegions)
{
    TArray<FNav3DRegionBuilder> MergedRegions = BoxRegions;
    bool bMergedAny;
    int32 PassCount = 0;
    constexpr int32 MaxPasses = 5; // Limit the number of passes to avoid infinite loops
    
    do
    {
        PassCount++;
        bMergedAny = false;
        int32 MergeCount = 0;
        
        for (int32 i = 0; i < MergedRegions.Num(); i++)
        {
            for (int32 j = i + 1; j < MergedRegions.Num(); j++)
            {
                if (CanMergeRegions(MergedRegions[i], MergedRegions[j]))
                {
                    // Merge regions using the new function
                    FNav3DRegionBuilder MergedRegion = MergeRegions(MergedRegions[i], MergedRegions[j]);
                    
                    // Update references in other regions
                    int32 RemovedRegionId = MergedRegions[j].Id;
                    MergedRegions[i] = MergedRegion;
                    UpdateAdjacentRegionReferences(MergedRegions, i, RemovedRegionId);
                    
                    // Remove the second region
                    MergedRegions.RemoveAt(j);
                    
                    MergeCount++;
                    bMergedAny = true;
                    j--; // Adjust index after removal
                }
            }
        }
        
        UE_LOG(LogNav3D, Verbose, TEXT("Merge pass %d: Performed %d merges, %d regions remaining"), 
               PassCount, MergeCount, MergedRegions.Num());
        
    } while (bMergedAny && PassCount < MaxPasses);
    
    UE_LOG(LogNav3D, Verbose, TEXT("Merged %d box regions into %d regions after %d passes"), 
           BoxRegions.Num(), MergedRegions.Num(), PassCount);
    
    return MergedRegions;
}

bool FNav3DTacticalReasoning::CanMergeRegions(const FNav3DRegionBuilder& RegionA, const FNav3DRegionBuilder& RegionB)
{
    // Only merge regions from the same layer
    if (RegionA.LayerIndex != RegionB.LayerIndex)
        return false;
    
    // Primary criterion: are they adjacent based on voxel connectivity?
    if (RegionA.AdjacentRegionIds.Contains(RegionB.Id))
    {
        // Check if merging would create a box-like shape (optional geometric validation)
        // This helps ensure we still create nice rectangular regions when possible
        
        // Case 1: Adjacent along X axis - YZ dimensions must match
        if ((RegionA.MaxCoord.X + 1 == RegionB.MinCoord.X || RegionB.MaxCoord.X + 1 == RegionA.MinCoord.X) &&
            (RegionA.MinCoord.Y == RegionB.MinCoord.Y && RegionA.MaxCoord.Y == RegionB.MaxCoord.Y &&
             RegionA.MinCoord.Z == RegionB.MinCoord.Z && RegionA.MaxCoord.Z == RegionB.MaxCoord.Z))
        {
            return true;
        }
        
        // Case 2: Adjacent along Y axis - XZ dimensions must match
        if ((RegionA.MaxCoord.Y + 1 == RegionB.MinCoord.Y || RegionB.MaxCoord.Y + 1 == RegionA.MinCoord.Y) &&
            (RegionA.MinCoord.X == RegionB.MinCoord.X && RegionA.MaxCoord.X == RegionB.MaxCoord.X &&
             RegionA.MinCoord.Z == RegionB.MinCoord.Z && RegionA.MaxCoord.Z == RegionB.MaxCoord.Z))
        {
            return true;
        }
        
        // Case 3: Adjacent along Z axis - XY dimensions must match
        if ((RegionA.MaxCoord.Z + 1 == RegionB.MinCoord.Z || RegionB.MaxCoord.Z + 1 == RegionA.MinCoord.Z) &&
            (RegionA.MinCoord.X == RegionB.MinCoord.X && RegionA.MaxCoord.X == RegionB.MaxCoord.X &&
             RegionA.MinCoord.Y == RegionB.MinCoord.Y && RegionA.MaxCoord.Y == RegionB.MaxCoord.Y))
        {
            return true;
        }
    }
    
    return false;
}

TArray<FNav3DRegion> FNav3DTacticalReasoning::SplitLargeRegion(const FNav3DRegion& Region, float MaxRegionSize)
{
    TArray<FNav3DRegion> Results;
    
    // Calculate the region size
    FVector Size = Region.Bounds.GetSize();
    
    // Determine which axis to split along (pick the longest)
    int32 SplitAxis = 0;
    float MaxAxis = Size.X;
    
    if (Size.Y > MaxAxis)
    {
        SplitAxis = 1;
        MaxAxis = Size.Y;
    }
    
    if (Size.Z > MaxAxis)
    {
        SplitAxis = 2;
        MaxAxis = Size.Z;
    }
    
    // Calculate split position
    FVector Min = Region.Bounds.Min;
    float SplitPos = Min[SplitAxis] + (MaxAxis / 2.0f);
    
    // Create two new regions
    FNav3DRegion LowerRegion;
    LowerRegion.Id = NextRegionId++;
    LowerRegion.LayerIndex = Region.LayerIndex;
    LowerRegion.Bounds = Region.Bounds;
    LowerRegion.Bounds.Max[SplitAxis] = SplitPos;
    
    FNav3DRegion UpperRegion;
    UpperRegion.Id = NextRegionId++;
    UpperRegion.LayerIndex = Region.LayerIndex;
    UpperRegion.Bounds = Region.Bounds;
    UpperRegion.Bounds.Min[SplitAxis] = SplitPos;
    
    UE_LOG(LogNav3D, VeryVerbose, TEXT("Split large region %d along axis %d at %.1f"), 
           Region.Id, SplitAxis, SplitPos);
    
    // Check if these regions need further splitting
    FVector LowerSize = LowerRegion.Bounds.GetSize();
    float LowerMaxDim = FMath::Max3(LowerSize.X, LowerSize.Y, LowerSize.Z);
    
    FVector UpperSize = UpperRegion.Bounds.GetSize();
    float UpperMaxDim = FMath::Max3(UpperSize.X, UpperSize.Y, UpperSize.Z);
    
    if (LowerMaxDim > MaxRegionSize)
    {
        Results.Append(SplitLargeRegion(LowerRegion, MaxRegionSize));
    }
    else
    {
        Results.Add(LowerRegion);
    }
    
    if (UpperMaxDim > MaxRegionSize)
    {
        Results.Append(SplitLargeRegion(UpperRegion, MaxRegionSize));
    }
    else
    {
        Results.Add(UpperRegion);
    }
    
    return Results;
}

TArray<FNav3DRegionBuilder> FNav3DTacticalReasoning::SubdivideVoxelsForLayer(
    const TArray<TPair<uint64, FIntVector>>& FreeVoxels,
    const int32 LayerIndex,
    const int32 MaxRegionLayer,
    const FNav3DVolumeNavigationData* VolumeData)
{
    TArray<FNav3DRegionBuilder> SubdividedRegions;
    
    // Calculate subdivision factor based on layer difference
    const int32 LayerDifference = LayerIndex - MaxRegionLayer;
    const int32 SubdivisionFactor = FMath::RoundToInt(FMath::Pow(2.0f, static_cast<float>(LayerDifference)));
    
    // Get navigation bounds information
    const auto& NavigationBounds = VolumeData->GetData().GetNavigationBounds();
    const FVector NavigationBoundsCenter = NavigationBounds.GetCenter();
    const FVector NavigationBoundsExtent = NavigationBounds.GetExtent();
    const FVector ZOrigin = NavigationBoundsCenter - NavigationBoundsExtent;
    
    // Get node information for both layers
    const float SourceLayerNodeExtent = VolumeData->GetData().GetLayer(LayerIndex).GetNodeExtent();
    const float TargetLayerNodeSize = VolumeData->GetData().GetLayer(MaxRegionLayer).GetNodeSize();
    
    UE_LOG(LogNav3D, Log, TEXT("Subdividing voxels from layer %d to layer %d (subdivision factor: %d)"),
           LayerIndex, MaxRegionLayer, SubdivisionFactor);
    
    // Process each free voxel in this layer
    for (const auto& VoxelPair : FreeVoxels)
    {
        const uint64 ParentMortonCode = VoxelPair.Key;
        
        // Get the world space center of the parent voxel
        FVector ParentWorldCenter = VolumeData->GetNodePositionFromLayerAndMortonCode(
            LayerIndex, ParentMortonCode);
        
        // Calculate world space bounds of the parent voxel
        FVector ParentMin = ParentWorldCenter - FVector(SourceLayerNodeExtent);
        
        // Calculate the size of each subdivision in world space
        float ChildSize = 2.0f * SourceLayerNodeExtent / SubdivisionFactor;
        
        // Create subdivision regions
        for (int32 x = 0; x < SubdivisionFactor; ++x)
        {
            for (int32 y = 0; y < SubdivisionFactor; ++y)
            {
                for (int32 z = 0; z < SubdivisionFactor; ++z)
                {
                    // Calculate world position for this subdivision (centered in the subdivision cell)
                    FVector SubdivisionCenter = ParentMin + FVector(
                        (x + 0.5f) * ChildSize,
                        (y + 0.5f) * ChildSize,
                        (z + 0.5f) * ChildSize
                    );
                    
                    // Convert to local position in navigation volume
                    FVector LocalPosition = SubdivisionCenter - ZOrigin;
                    
                    // Convert to target layer coordinates
                    FIntVector TargetCoord;
                    TargetCoord.X = FMath::FloorToInt(LocalPosition.X / TargetLayerNodeSize);
                    TargetCoord.Y = FMath::FloorToInt(LocalPosition.Y / TargetLayerNodeSize);
                    TargetCoord.Z = FMath::FloorToInt(LocalPosition.Z / TargetLayerNodeSize);
                    
                    // Create a region builder for this subdivision at the MaxRegionLayer
                    FNav3DRegionBuilder Builder;
                    Builder.Id = NextRegionId++;
                    Builder.LayerIndex = MaxRegionLayer;  // Important: Use target layer, not original
                    
                    // Set coordinates for this region
                    Builder.MinCoord = TargetCoord;
                    Builder.MaxCoord = TargetCoord;
                    
                    // Generate Morton code for this target coordinate
                    const uint64 TargetMortonCode = FNav3DUtils::GetMortonCodeFromVector(TargetCoord);
                    Builder.MortonCodes.Add(TargetMortonCode);
                    SubdividedRegions.Add(Builder);
                }
            }
        }
    }
    
    return SubdividedRegions;
}

void FNav3DTacticalReasoning::ReindexRegions(TArray<FNav3DRegion>& Regions)
{
    // Create a mapping from old IDs to new IDs
    TMap<int32, int32> IdRemapping;
    
    // Assign new sequential IDs
    for (int32 i = 0; i < Regions.Num(); ++i)
    {
        int32 OldId = Regions[i].Id;
        int32 NewId = i;
        
        IdRemapping.Add(OldId, NewId);
        Regions[i].Id = NewId;
    }
    
    // Update adjacency and visibility references
    for (FNav3DRegion& Region : Regions)
    {
        // Update adjacency links
        TArray<int32> NewAdjacentIds;
        for (const int32 OldId : Region.AdjacentRegionIds)
        {
            if (const int32* NewId = IdRemapping.Find(OldId))
            {
                NewAdjacentIds.Add(*NewId);
            }
        }
        Region.AdjacentRegionIds = NewAdjacentIds;
        
        // Update visibility set
        TArray<int32> NewVisibilitySet;
        for (const int32 OldId : Region.VisibilitySet)
        {
            if (const int32* NewId = IdRemapping.Find(OldId))
            {
                NewVisibilitySet.Add(*NewId);
            }
        }
        Region.VisibilitySet = NewVisibilitySet;
    }
    
    // Note: NextRegionId in FNav3DTacticalReasoning should be updated separately
    NextRegionId = Regions.Num();
}

void FNav3DTacticalReasoning::VerifyRegionsAgainstStaticGeometry(TArray<FNav3DRegion>& Regions, const FNav3DVolumeNavigationData* VolumeData)
{
    QUICK_SCOPE_CYCLE_COUNTER(STAT_Nav3DTacticalReasoning_VerifyRegionsAgainstStaticGeometry);
    
    if (!VolumeData || !VolumeData->Settings.World || Regions.Num() == 0)
    {
        return;
    }
    
    // Cache the collision settings
    const FCollisionQueryParams& QueryParams = VolumeData->Settings.GenerationSettings.CollisionQueryParameters;
    const ECollisionChannel CollisionChannel = VolumeData->Settings.GenerationSettings.CollisionChannel;
    
    UE_LOG(LogNav3D, Verbose, TEXT("Verifying %d regions against static geometry"), Regions.Num());
    
    // Track regions to remove
    TArray<int32> RegionsToRemove;
    
    // Test each region
    for (int32 RegionIndex = 0; RegionIndex < Regions.Num(); ++RegionIndex)
    {
        const FNav3DRegion& Region = Regions[RegionIndex];
        
        // Create a slightly smaller box for the test to allow regions that just touch geometry
        FVector BoxCenter = Region.Bounds.GetCenter();
        FVector BoxExtent = Region.Bounds.GetExtent() * 0.9f; // 10% smaller
        
        TArray<FOverlapResult> Overlaps;
        const bool bHasOverlap = VolumeData->Settings.World->OverlapMultiByChannel(
            Overlaps,
            BoxCenter,
            FQuat::Identity,
            CollisionChannel,
            FCollisionShape::MakeBox(BoxExtent),
            QueryParams
        );
        
        if (bHasOverlap)
        {
            // Filter out non-static or non-navigation affecting components
            for (const FOverlapResult& Overlap : Overlaps)
            {
                UPrimitiveComponent* Component = Overlap.GetComponent();
                if (Component && Component->CanEverAffectNavigation())
                {
                    // Only count static meshes, landscape, and nav modifiers
                    if (Cast<UStaticMeshComponent>(Component) || 
                        Cast<ULandscapeHeightfieldCollisionComponent>(Component))
                    {
                        // Further verify with a more precise test
                        if (IsRegionInsideGeometry(Region, Component, VolumeData))
                        {
                            RegionsToRemove.Add(RegionIndex);
                            UE_LOG(LogNav3D, Verbose, TEXT("Region %d marked for removal - detected inside geometry"), 
                                   Region.Id);
                            break;
                        }
                    }
                }
            }
        }
    }
    
    // Apply initial removals
    int32 RemovedCount = 0;
    if (RegionsToRemove.Num() > 0)
    {
        // Sort in descending order to safely remove from array
        RegionsToRemove.Sort([](const int32 A, const int32 B) { return A > B; });
        
        // Remove invalid regions
        for (const int32 IndexToRemove : RegionsToRemove)
        {
            if (IndexToRemove < Regions.Num())
            {
                UE_LOG(LogNav3D, Verbose, TEXT("Removing region %d due to static mesh overlap"), 
                      Regions[IndexToRemove].Id);
                Regions.RemoveAt(IndexToRemove);
                RemovedCount++;
            }
        }
        
        // Temporary reindex after initial removals to establish adjacency for next step
        ReindexRegions(Regions);
    }
    
    // Build adjacency graph before checking for isolated regions
    BuildAdjacencyGraph(Regions);
    
    // Now check for isolated regions (surrounded by removed regions)
    TArray<int32> PotentialIsolatedRegions;
    
    for (int32 RegionIndex = 0; RegionIndex < Regions.Num(); ++RegionIndex)
    {
        const FNav3DRegion& Region = Regions[RegionIndex];
        
        // If this region has no adjacent regions, it might be isolated inside geometry
        if (Region.AdjacentRegionIds.Num() == 0)
        {
            // Cast rays in many directions to test for enclosure
            FVector RegionCenter = Region.Bounds.GetCenter();
            int32 RaysBlocked = 0;
            int32 TotalRays = 0;
            
            // Cast rays in multiple directions
            for (int32 Theta = 0; Theta < 180; Theta += 45)
            {
                for (int32 Phi = 0; Phi < 360; Phi += 45)
                {
                    // Convert spherical to Cartesian coordinates
                    const float RadTheta = FMath::DegreesToRadians(Theta);
                    const float RadPhi = FMath::DegreesToRadians(Phi);
                    
                    FVector RayDir(
                        FMath::Sin(RadTheta) * FMath::Cos(RadPhi),
                        FMath::Sin(RadTheta) * FMath::Sin(RadPhi),
                        FMath::Cos(RadTheta)
                    );
                    
                    // Cast ray
                    FVector RayEnd = RegionCenter + RayDir * 10000.0f;
                    
                    FHitResult Hit;
                    const bool bHit = VolumeData->Settings.World->LineTraceSingleByChannel(
                        Hit,
                        RegionCenter,
                        RayEnd,
                        CollisionChannel,
                        QueryParams
                    );
                    
                    TotalRays++;
                    if (bHit)
                    {
                        RaysBlocked++;
                    }
                }
            }
            
            // If most rays are blocked (90%+), this region is likely inside a hollow mesh
            if (constexpr float BlockageThreshold = 0.90f; TotalRays > 0
                && static_cast<float>(RaysBlocked) / TotalRays > BlockageThreshold)
            {
                PotentialIsolatedRegions.Add(RegionIndex);
                UE_LOG(LogNav3D, Verbose, TEXT("Region %d appears to be isolated inside geometry (%d/%d rays blocked)"), 
                       Region.Id, RaysBlocked, TotalRays);
            }
        }
    }
    
    // Remove the isolated regions (in reverse order to maintain valid indices)
    if (PotentialIsolatedRegions.Num() > 0)
    {
        PotentialIsolatedRegions.Sort([](const int32 A, const int32 B) { return A > B; });
        
        for (const int32 IndexToRemove : PotentialIsolatedRegions)
        {
            if (IndexToRemove < Regions.Num())
            {
                UE_LOG(LogNav3D, Verbose, TEXT("Removing isolated region %d"), Regions[IndexToRemove].Id);
                Regions.RemoveAt(IndexToRemove);
                RemovedCount++;
            }
        }
    }
    
    // Reindex if any regions were removed
    if (RemovedCount > 0)
    {
        ReindexRegions(Regions);
        UE_LOG(LogNav3D, Log, TEXT("Region verification complete: removed %d regions (%d direct overlap, %d isolated)"), 
              RemovedCount, RegionsToRemove.Num(), PotentialIsolatedRegions.Num());
    }
    else
    {
        UE_LOG(LogNav3D, Log, TEXT("Region verification complete: all %d regions are valid"), Regions.Num());
    }
}

bool FNav3DTacticalReasoning::IsRegionInsideGeometry(
    const FNav3DRegion& Region, 
    UPrimitiveComponent* Component,
    const FNav3DVolumeNavigationData* VolumeData)
{
    if (!Component || !VolumeData)
    {
        return false;
    }

    // Get region center for ray casting and additional tests
    FVector RegionCenter = Region.Bounds.GetCenter();
    
    // For static mesh components, use ray casting approach
    if (UStaticMeshComponent* StaticMeshComp = Cast<UStaticMeshComponent>(Component))
    {
        // First do a quick AABB test
        const FBox MeshBounds = StaticMeshComp->Bounds.GetBox();
        bool bBoundsOverlap = Region.Bounds.Intersect(MeshBounds);
        
        if (!bBoundsOverlap)
        {
            return false;
        }
        
        // Special handling for concave meshes:
        // 1. Cast rays from region center through corners
        // 2. Count intersection patterns to determine if inside a concave mesh
        
        // Sample points at corners plus center point
        TArray<FVector> SamplePoints;
        
        // Get points at the corners of the region bounds
        for (int32 X = 0; X <= 1; X++)
        {
            for (int32 Y = 0; Y <= 1; Y++)
            {
                for (int32 Z = 0; Z <= 1; Z++)
                {
                    FVector Corner(
                        X ? Region.Bounds.Max.X : Region.Bounds.Min.X,
                        Y ? Region.Bounds.Max.Y : Region.Bounds.Min.Y,
                        Z ? Region.Bounds.Max.Z : Region.Bounds.Min.Z
                    );
                    
                    SamplePoints.Add(Corner);
                }
            }
        }
        
        // Count patterns that suggest we're inside a concave mesh
        int32 InsidePatternCount = 0;
        int32 OutsidePatternCount = 0;
        
        for (const FVector& Corner : SamplePoints)
        {
            // Ray direction from center to corner
            FVector RayDir = (Corner - RegionCenter).GetSafeNormal();
            
            // Cast a ray from center to corner
            FHitResult HitToCorner;
            bool bHitToCorner = VolumeData->Settings.World->LineTraceSingleByChannel(
                HitToCorner,
                RegionCenter,
                Corner,
                VolumeData->Settings.GenerationSettings.CollisionChannel,
                VolumeData->Settings.GenerationSettings.CollisionQueryParameters
            );
            
            // If ray hits mesh before reaching corner, it's a sign we're inside a concave part
            if (bHitToCorner && Cast<UStaticMeshComponent>(HitToCorner.GetComponent()) == StaticMeshComp)
            {
                InsidePatternCount++;
            }
            
            // Cast ray from corner outward (away from center)
            FVector OutwardRayEnd = Corner + RayDir * Region.Bounds.GetExtent().GetMax() * 2.0f;
            
            FHitResult OutwardHit;
            bool bOutwardHit = VolumeData->Settings.World->LineTraceSingleByChannel(
                OutwardHit,
                Corner,
                OutwardRayEnd,
                VolumeData->Settings.GenerationSettings.CollisionChannel,
                VolumeData->Settings.GenerationSettings.CollisionQueryParameters
            );
            
            // If outward ray doesn't hit, or hits far away, we're likely outside
            if (!bOutwardHit || OutwardHit.Distance > Region.Bounds.GetExtent().GetMax())
            {
                OutsidePatternCount++;
            }
        }
        
        // Now use corner penetration test for additional verification
        int32 CornersPenetrating = 0;
        
        for (const FVector& Corner : SamplePoints)
        {
            if (VolumeData->CheckStaticMeshOcclusion(StaticMeshComp, Corner, 1.0f))
            {
                CornersPenetrating++;
            }
        }
        
        // Determining factors:
        // 1. High inside pattern count suggests we're inside a concave mesh
        // 2. High outside pattern count suggests we're outside
        // 3. Corner penetration suggests mesh walls pass through the region
        
        // Region is considered inside geometry if:
        // - More than half of corners penetrate geometry OR
        // - Inside patterns significantly outweigh outside patterns
        const int32 TotalSamples = SamplePoints.Num();
        
        bool bIsInsideMesh = (CornersPenetrating > TotalSamples / 2) || 
                            (InsidePatternCount > OutsidePatternCount * 2);
        
        // For debugging
        UE_LOG(LogNav3D, VeryVerbose, TEXT("Region %d concave test: %d/%d corners penetrating, %d inside patterns, %d outside patterns, Result: %s"),
              Region.Id, CornersPenetrating, TotalSamples, InsidePatternCount, OutsidePatternCount,
              bIsInsideMesh ? TEXT("Inside") : TEXT("Outside"));
        
        return bIsInsideMesh;
    }
    
    // For landscape components, test if region is below landscape height
    if (ULandscapeHeightfieldCollisionComponent* LandscapeComp = Cast<ULandscapeHeightfieldCollisionComponent>(Component))
    {
        // Get the region's bounds
        const FVector& Min = Region.Bounds.Min;
        const FVector& Max = Region.Bounds.Max;
        
        // Test bottom corners
        int32 PointsBelow = 0;
        for (int32 X = 0; X <= 1; X++)
        {
            for (int32 Y = 0; Y <= 1; Y++)
            {
                FVector Point(
                    X ? Max.X : Min.X,
                    Y ? Max.Y : Min.Y,
                    Min.Z
                );
                
                // Get landscape height at this XY location
                FVector LocalPoint = LandscapeComp->GetComponentTransform().InverseTransformPosition(Point);

                if (TOptional<float> Height = LandscapeComp->GetHeight(LocalPoint.X, LocalPoint.Y, EHeightfieldSource::Complex); Height.IsSet())
                {
                    // Convert height to world space
                    float WorldHeight = LandscapeComp->GetComponentTransform().TransformPosition(
                        FVector(0, 0, Height.GetValue())).Z;
                    
                    // Check if point is below landscape
                    if (Point.Z < WorldHeight)
                    {
                        PointsBelow++;
                    }
                }
            }
        }
        
        // If any corner is below, consider it an underground region
        return (PointsBelow > 0);
    }
    
    // For other component types, use general collision
    {
        const FBox& RegionBox = Region.Bounds;
        FVector RegionExtent = RegionBox.GetExtent() * 0.9f; // Slightly smaller box
    
        // Sample points within the region
        TArray<FVector> TestPoints;
    
        // Add the center point
        TestPoints.Add(RegionCenter);
    
        // Add a few slightly offset points for better coverage
        for (float Offset = -0.5f; Offset <= 0.5f; Offset += 0.5f)
        {
            if (Offset == 0.0f) continue; // Skip center (already added)
        
            TestPoints.Add(RegionCenter + FVector(Offset * RegionExtent.X, 0, 0));
            TestPoints.Add(RegionCenter + FVector(0, Offset * RegionExtent.Y, 0));
            TestPoints.Add(RegionCenter + FVector(0, 0, Offset * RegionExtent.Z));
        }
    
        // Test each point
        int32 PointsInside = 0;
    
        for (const FVector& Point : TestPoints)
        {
            // Use small sphere for point testing
            TArray<FOverlapResult> Overlaps;
            bool bHasOverlap = VolumeData->Settings.World->OverlapMultiByChannel(
                Overlaps,
                Point,
                FQuat::Identity,
                VolumeData->Settings.GenerationSettings.CollisionChannel,
                FCollisionShape::MakeSphere(1.0f), // Small sphere as point
                VolumeData->Settings.GenerationSettings.CollisionQueryParameters
            );
        
            // Check if our component is in the results
            if (bHasOverlap)
            {
                for (const FOverlapResult& OverlapResult : Overlaps)
                {
                    if (OverlapResult.GetComponent() == Component)
                    {
                        PointsInside++;
                        break;
                    }
                }
            }
        }
    
        // Consider inside if the majority of points overlap
        return (PointsInside > TestPoints.Num() / 2);
    }
}

void FNav3DTacticalReasoning::BuildAdjacencyGraph(FNav3DTacticalData& TargetData)
{
    // Check for adjacency between all region pairs
    for (int32 i = 0; i < TargetData.Regions.Num(); ++i)
    {
        for (int32 j = i + 1; j < TargetData.Regions.Num(); ++j)
        {
            // First check if there's already an adjacency relationship
            bool bAdjacent = TargetData.Regions[i].AdjacentRegionIds.Contains(TargetData.Regions[j].Id);
            bool bReverseAdjacent = TargetData.Regions[j].AdjacentRegionIds.Contains(TargetData.Regions[i].Id);
            
            // Ensure bidirectional consistency for existing relationships
            if (bAdjacent != bReverseAdjacent)
            {
                if (bAdjacent)
                {
                    TargetData.Regions[j].AdjacentRegionIds.Add(TargetData.Regions[i].Id);
                }
                else
                {
                    TargetData.Regions[i].AdjacentRegionIds.Add(TargetData.Regions[j].Id);
                }
            }
            
            // If no adjacency already exists, check using the improved method
            if (!bAdjacent && !bReverseAdjacent)
            {
                if (AreRegionsAdjacent(TargetData.Regions[i], TargetData.Regions[j]))
                {
                    TargetData.Regions[i].AdjacentRegionIds.Add(TargetData.Regions[j].Id);
                    TargetData.Regions[j].AdjacentRegionIds.Add(TargetData.Regions[i].Id);
                    
                    UE_LOG(LogNav3D, Verbose, TEXT("New adjacency detected between Region %d (Layer %d) and Region %d (Layer %d)"),
                          TargetData.Regions[i].Id, TargetData.Regions[i].LayerIndex,
                          TargetData.Regions[j].Id, TargetData.Regions[j].LayerIndex);
                }
            }
        }
    }
    
    UE_LOG(LogNav3D, Verbose, TEXT("Built adjacency graph for %d regions"), TargetData.Regions.Num());
}

void FNav3DTacticalReasoning::BuildAdjacencyGraph(TArray<FNav3DRegion>& Regions)
{
    // Implementation for any region array
    for (int32 i = 0; i < Regions.Num(); ++i)
    {
        for (int32 j = i + 1; j < Regions.Num(); ++j)
        {
            // First check if there's already an adjacency relationship
            bool bAdjacent = Regions[i].AdjacentRegionIds.Contains(Regions[j].Id);
            bool bReverseAdjacent = Regions[j].AdjacentRegionIds.Contains(Regions[i].Id);
            
            // Ensure bidirectional consistency for existing relationships
            if (bAdjacent != bReverseAdjacent)
            {
                if (bAdjacent)
                {
                    Regions[j].AdjacentRegionIds.Add(Regions[i].Id);
                }
                else
                {
                    Regions[i].AdjacentRegionIds.Add(Regions[j].Id);
                }
            }
            
            // If no adjacency already exists, check using the improved method
            if (!bAdjacent && !bReverseAdjacent)
            {
                if (AreRegionsAdjacent(Regions[i], Regions[j]))
                {
                    Regions[i].AdjacentRegionIds.Add(Regions[j].Id);
                    Regions[j].AdjacentRegionIds.Add(Regions[i].Id);
                }
            }
        }
    }
}

bool FNav3DTacticalReasoning::FindBestLocation(
    const FNav3DTacticalData& TargetData,
    const FVector& StartPosition,
    const TArray<FVector>& ObserverPositions,
    ETacticalVisibility Visibility,
    ETacticalDistance DistancePreference,
    ETacticalRegion RegionPreference,
    bool bForceNewRegion,
    bool bUseRaycasting,
    TArray<FPositionCandidate>& OutCandidatePositions) const
{
    // Edge case: No observers
    if (ObserverPositions.Num() == 0)
    {
        UE_LOG(LogNav3D, Warning, TEXT("FindBestLocation called with no observer positions"));
        return false;
    }
    
    // Find containing region for start position
    const int32 StartRegionId = TargetData.FindContainingRegion(StartPosition);
    if (StartRegionId == -1)
    {
        UE_LOG(LogNav3D, Warning, TEXT("Invalid start position - not in any region"));
        return false;
    }
    
    // Find containing regions for all observers
    // Also remove duplicates to optimize the query
    TSet<int32> UniqueObserverRegionIds;
    for (const FVector& ObserverPos : ObserverPositions)
    {
        const int32 ObserverRegionId = TargetData.FindContainingRegion(ObserverPos);
        if (ObserverRegionId != -1)
        {
            UniqueObserverRegionIds.Add(ObserverRegionId);
        }
        else
        {
            UE_LOG(LogNav3D, Warning, TEXT("Observer position (%.1f, %.1f, %.1f) not in any region - skipping"),
                ObserverPos.X, ObserverPos.Y, ObserverPos.Z);
        }
    }
    
    // Check if we have any valid observer regions
    if (UniqueObserverRegionIds.Num() == 0)
    {
        UE_LOG(LogNav3D, Warning, TEXT("No valid observer regions found"));
        return false;
    }
    
    UE_LOG(LogNav3D, Verbose, TEXT("Finding best location from region %d with %d unique observer regions"),
        StartRegionId, UniqueObserverRegionIds.Num());
        
    // Get start region position for direct distance calculation
    const FNav3DRegion* StartRegion = GetRegionById(TargetData, StartRegionId);
    FVector StartRegionPos = StartRegion ? StartRegion->Bounds.GetCenter() : StartPosition;
    
    // Track position candidates
    TArray<FPositionCandidate> Candidates;
    TSet<int32> VisitedRegionIds;
    
    // BFS queue with distance tracking
    TArray<TPair<int32, float>> RegionsQueue;
    RegionsQueue.Add(TPair<int32, float>(StartRegionId, 0.0f));
    VisitedRegionIds.Add(StartRegionId);
    
    // Config parameters from settings
    const float MaxSearchDistance = NavDataRef->TacticalSettings.MaxCoverSearchDistance;
    
    // Data for normalization
    float MinDirectDistance = MAX_FLT;
    float MaxDirectDistance = 0.0f;
    float MinRegionSize = MAX_FLT;
    float MaxRegionSize = 0.0f;
    
    // BFS traversal
    int32 TraversalCount = 0;
    while (RegionsQueue.Num() > 0)
    {
        TPair<int32, float> Current = RegionsQueue[0];
        RegionsQueue.RemoveAt(0);
        TraversalCount++;
        
        if (Current.Value > MaxSearchDistance) 
        {
            UE_LOG(LogNav3D, Verbose, TEXT("Skipping region %d (exceeds max search distance: %.1f > %.1f)"), 
                Current.Key, Current.Value, MaxSearchDistance);
            continue;
        }
        
        UE_LOG(LogNav3D, VeryVerbose, TEXT("Checking region %d (path distance: %.1f)"), Current.Key, Current.Value);

        // Skip the start region if we're forcing a new region
        if (bForceNewRegion && Current.Key == StartRegionId)
        {
            UE_LOG(LogNav3D, Verbose, TEXT("Skipping start region %d (bForceNewRegion=true)"), Current.Key);
        }
        else
        {
            // Check visibility requirements against all observers
            bool bIsVisibilityMatch = true;
            
            for (int32 ObserverRegionId : UniqueObserverRegionIds)
            {
                bool bThisObserverMatch = TargetData.IsRegionVisibilityMatch(
                    ObserverRegionId, Current.Key, Visibility);
                
                if (!bThisObserverMatch)
                {
                    bIsVisibilityMatch = false;
                    break;
                }
            }
            
            if (bIsVisibilityMatch)
            {
                // Get the region for more details
                if (const FNav3DRegion* CurrentRegion = GetRegionById(TargetData, Current.Key))
                {
                    FVector CurrentRegionPos = CurrentRegion->Bounds.GetCenter();
                    float DirectDistance = FVector::Distance(StartRegionPos, CurrentRegionPos);
                    float RegionSize = CurrentRegion->Bounds.GetVolume();
                    
                    UE_LOG(LogNav3D, VeryVerbose, TEXT("Found potential location in region %d, PathDistance: %.1f, DirectDistance: %.1f, RegionSize: %.1f"), 
                        Current.Key, Current.Value, DirectDistance, RegionSize);
                    
                    // Create candidate
                    FPositionCandidate Candidate;
                    Candidate.RegionId = Current.Key;
                    Candidate.Position = CurrentRegionPos;  // Initially use region center
                    Candidate.PathDistance = Current.Value;
                    Candidate.DirectDistance = DirectDistance;
                    Candidate.RegionSize = RegionSize;
                    Candidate.Score = 1.0f;  // Base score, will be adjusted later
                    
                    // Update min/max values for normalization
                    MinDirectDistance = FMath::Min(MinDirectDistance, DirectDistance);
                    MaxDirectDistance = FMath::Max(MaxDirectDistance, DirectDistance);
                    MinRegionSize = FMath::Min(MinRegionSize, RegionSize);
                    MaxRegionSize = FMath::Max(MaxRegionSize, RegionSize);
                    
                    Candidates.Add(Candidate);
                }
            }
            else
            {
                UE_LOG(LogNav3D, VeryVerbose, TEXT("Skipping region %d (visibility requirements not met)"), Current.Key);
            }
        }
        
        // Always explore adjacent regions regardless of whether current region is a valid candidate
        if (const FNav3DRegion* CurrentRegion = GetRegionById(TargetData, Current.Key))
        {
            UE_LOG(LogNav3D, VeryVerbose, TEXT("Exploring %d adjacent regions for region %d"), CurrentRegion->AdjacentRegionIds.Num(), Current.Key);
            
            for (int32 NeighborId : CurrentRegion->AdjacentRegionIds)
            {
                if (!VisitedRegionIds.Contains(NeighborId))
                {
                    // Calculate distance to neighbor center (approximation)
                    const FNav3DRegion* NeighborRegion = GetRegionById(TargetData, NeighborId);
                    float NewDistance = Current.Value;
                    
                    if (NeighborRegion)
                    {
                        FVector NeighborCenter = NeighborRegion->Bounds.GetCenter();
                        FVector CurrentCenter = CurrentRegion->Bounds.GetCenter();
                        NewDistance += FVector::Distance(CurrentCenter, NeighborCenter);
                        
                        UE_LOG(LogNav3D, VeryVerbose, TEXT("Adding adjacent region %d to search queue, new path distance: %.1f"), NeighborId, NewDistance);
                    }
                    
                    VisitedRegionIds.Add(NeighborId);
                    RegionsQueue.Add(TPair<int32, float>(NeighborId, NewDistance));
                }
            }
        }
    }
    
    UE_LOG(LogNav3D, Verbose, TEXT("BFS completed: traversed %d regions, visited %d regions, found %d candidates"), 
           TraversalCount, VisitedRegionIds.Num(), Candidates.Num());
    
    // Score and rank candidates
    if (Candidates.Num() > 0)
    {
        // Normalize the distance range to handle distance preferences
        float DistanceRange = MaxDirectDistance - MinDirectDistance;
        if (DistanceRange <= 0.0f) DistanceRange = 1.0f; // Avoid division by zero
        
        // Normalize the region size range
        float RegionSizeRange = MaxRegionSize - MinRegionSize;
        if (RegionSizeRange <= 0.0f) RegionSizeRange = 1.0f; // Avoid division by zero
        
        // Calculate median values if needed
        float MedianDistance = 0.0f;
        float MedianRegionSize = 0.0f;
        
        if (DistancePreference == ETacticalDistance::Median ||
            RegionPreference == ETacticalRegion::Median)
        {
            // Sort copies of the arrays to find medians
            TArray<float> Distances;
            TArray<float> RegionSizes;
            
            for (const FPositionCandidate& Candidate : Candidates)
            {
                Distances.Add(Candidate.DirectDistance);
                RegionSizes.Add(Candidate.RegionSize);
            }
            
            Distances.Sort();
            RegionSizes.Sort();
            
            int32 MiddleIndex = Distances.Num() / 2;
            MedianDistance = Distances[MiddleIndex];
            MedianRegionSize = RegionSizes[MiddleIndex];
        }
        
        // Apply scoring based on preferences
        for (FPositionCandidate& Candidate : Candidates)
        {
            float DistanceScore = 1.0f;
            float RegionSizeScore = 1.0f;
            
            // Calculate distance score
            switch (DistancePreference)
            {
                case ETacticalDistance::Any:
                    // No adjustment needed
                    break;
                    
                case ETacticalDistance::Closest:
                    // Normalize and invert (0=farthest, 1=closest)
                    DistanceScore = 1.0f - ((Candidate.DirectDistance - MinDirectDistance) / DistanceRange);
                    break;
                    
                case ETacticalDistance::Furthest:
                    // Normalize (0=closest, 1=farthest)
                    DistanceScore = (Candidate.DirectDistance - MinDirectDistance) / DistanceRange;
                    break;
                    
                case ETacticalDistance::Median:
                    // Score based on closeness to median (0=farthest from median, 1=at median)
                    DistanceScore = 1.0f - (FMath::Abs(Candidate.DirectDistance - MedianDistance) / DistanceRange);
                    break;
            }
            
            // Calculate region size score
            switch (RegionPreference)
            {
                case ETacticalRegion::Any:
                    // No adjustment needed
                    break;
                    
                case ETacticalRegion::Smallest:
                    // Normalize and invert (0=largest, 1=smallest)
                    RegionSizeScore = 1.0f - ((Candidate.RegionSize - MinRegionSize) / RegionSizeRange);
                    break;
                    
                case ETacticalRegion::Largest:
                    // Normalize (0=smallest, 1=largest)
                    RegionSizeScore = (Candidate.RegionSize - MinRegionSize) / RegionSizeRange;
                    break;
                    
                case ETacticalRegion::Median:
                    // Score based on closeness to median (0=farthest from median, 1=at median)
                    RegionSizeScore = 1.0f - (FMath::Abs(Candidate.RegionSize - MedianRegionSize) / RegionSizeRange);
                    break;
            }
            
            // Combine scores - visibility already filtered out invalid candidates
            // Apply distance and region size as weighted factors
            Candidate.Score *= DistanceScore * RegionSizeScore;
            
            // Log the score components
            UE_LOG(LogNav3D, Verbose, TEXT("Candidate in region %d: Distance Score: %.2f, Region Size Score: %.2f, Total Score: %.2f"), 
                Candidate.RegionId, DistanceScore, RegionSizeScore, Candidate.Score);
        }
        
        // If raycasting is enabled, verify visibility with actual raycasts
        if (bUseRaycasting)
        {
            FCriticalSection CriticalSection;
            
            // Process each candidate
            ParallelFor(Candidates.Num(), [&](const int32 CandidateIndex)
            {
                FPositionCandidate& Candidate = Candidates[CandidateIndex];
                
                // Select a test point - either the current position or a random point in the region
                FVector TestPoint = Candidate.Position;
                
                if (const FNav3DRegion* Region = GetRegionById(TargetData, Candidate.RegionId))
                {
                    // Use a random point instead of region center for more realistic testing
                    TestPoint = GetRandomPointInRegion(*Region);
                    
                    // Update the candidate position to the test point if it passes
                    Candidate.Position = TestPoint;
                }
                
                // Track raycast results for each observer
                int32 SuccessfulVisibilityTests = 0;
                
                // Test against all observers
                for (const FVector& ObserverPos : ObserverPositions)
                {
                    bool bRaycastSuccess = false;
                    
                    if (NavDataRef.IsValid())
                    {
                        if (const auto* Raycaster = NewObject<UNav3DRaycaster>())
                        {
                            // Find the volume containing both points
                            if (const auto* NavData = NavDataRef->GetVolumeNavigationDataContainingPoints({ObserverPos, TestPoint}))
                            {
                                // Perform the raycast
                                FNav3DRaycastHit Hit;
                                const bool bHit = Raycaster->Trace(*NavData, ObserverPos, TestPoint, Hit);
                                
                                // Interpret hit results based on visibility requirement
                                switch (Visibility)
                                {
                                    case ETacticalVisibility::TargetVisible:
                                    case ETacticalVisibility::MutuallyVisible:
                                        bRaycastSuccess = !bHit; // No hit = visible
                                        break;
                                        
                                    case ETacticalVisibility::TargetOccluded:
                                    case ETacticalVisibility::MutuallyOccluded:
                                        bRaycastSuccess = bHit; // Hit = occluded
                                        break;
                                }
                                
                                // For mutual visibility, also check the reverse direction
                                if (bRaycastSuccess && 
                                    (Visibility == ETacticalVisibility::MutuallyVisible || 
                                     Visibility == ETacticalVisibility::MutuallyOccluded))
                                {
                                    const bool bReverseHit = Raycaster->Trace(*NavData, TestPoint, ObserverPos, Hit);
                                    
                                    // Check if reverse direction matches the requirement
                                    if (Visibility == ETacticalVisibility::MutuallyVisible)
                                    {
                                        bRaycastSuccess = !bReverseHit; // No hit = visible
                                    }
                                    else // MutuallyOccluded
                                    {
                                        bRaycastSuccess = bReverseHit; // Hit = occluded
                                    }
                                }
                            }
                        }
                    }
                    
                    if (bRaycastSuccess)
                    {
                        SuccessfulVisibilityTests++;
                    }
                }
                
                // Calculate the success ratio
                const float SuccessRatio = static_cast<float>(SuccessfulVisibilityTests) / ObserverPositions.Num();
                
                // Adjust score based on raycast results
                // Using square to penalize partial successes more
                const float RaycastMultiplier = FMath::Pow(SuccessRatio, 2.0f);
                
                // Update candidate score with raycast results
                FScopeLock Lock(&CriticalSection);
                Candidate.Score *= RaycastMultiplier;
                
                UE_LOG(LogNav3D, VeryVerbose, TEXT("Raycast test for region %d: %d/%d successful (%.2f), new score: %.2f"),
                       Candidate.RegionId, SuccessfulVisibilityTests, ObserverPositions.Num(), SuccessRatio, Candidate.Score);
            });
        }
        else
        {
            // If not raycasting, generate more natural positions
            for (FPositionCandidate& Candidate : Candidates)
            {
                if (const FNav3DRegion* Region = GetRegionById(TargetData, Candidate.RegionId))
                {
                    // Use a random point instead of region center
                    Candidate.Position = GetRandomPointInRegion(*Region);
                }
            }
        }
        
        // Sort candidates by score (highest first)
        Candidates.Sort([](const FPositionCandidate& A, const FPositionCandidate& B) {
            return A.Score > B.Score;
        });
        
        // Log all candidates with their scores for debugging
        for (int32 i = 0; i < FMath::Min(10, Candidates.Num()); i++)
        {
            const FPositionCandidate& Candidate = Candidates[i];
            UE_LOG(LogNav3D, Verbose, TEXT("Candidate %d: Region %d, Position (%.1f, %.1f, %.1f), Score: %.2f%s"), 
                i+1, Candidate.RegionId, 
                Candidate.Position.X, Candidate.Position.Y, Candidate.Position.Z,
                Candidate.Score, (i == 0) ? TEXT(" (best)") : TEXT(""));
        }
        
        OutCandidatePositions = Candidates;
        
        return Candidates.Num() > 0;
    }
    
    return false;
}

void FNav3DTacticalReasoning::BuildVoxelLevelAdjacency(TArray<FNav3DRegionBuilder>& Regions)
{
    // Create a map from coordinate to region index for quick lookups
    TMap<FIntVector, int32> CoordToRegionIndex;
    
    // Populate the map
    for (int32 i = 0; i < Regions.Num(); ++i)
    {
        for (const uint64 MortonCode : Regions[i].MortonCodes)
        {
            FIntVector Coord = FIntVector(FNav3DUtils::GetVectorFromMortonCode(MortonCode));
            CoordToRegionIndex.Add(Coord, i);
        }
    }
    
    // Check each voxel's 6 potential neighbors
    static const FIntVector Directions[6] = {
        FIntVector(1, 0, 0),  // +X
        FIntVector(-1, 0, 0), // -X
        FIntVector(0, 1, 0),  // +Y
        FIntVector(0, -1, 0), // -Y
        FIntVector(0, 0, 1),  // +Z
        FIntVector(0, 0, -1)  // -Z
    };
    
    for (int32 i = 0; i < Regions.Num(); ++i)
    {
        FNav3DRegionBuilder& Region = Regions[i];
        
        for (const uint64 MortonCode : Region.MortonCodes)
        {
            FIntVector Coord = FIntVector(FNav3DUtils::GetVectorFromMortonCode(MortonCode));
            
            // Check all 6 adjacent directions
            for (int32 Dir = 0; Dir < 6; ++Dir)
            {
                FIntVector NeighborCoord = Coord + Directions[Dir];
                
                // See if this neighbor exists and is in a different region
                const int32* NeighborRegionIndex = CoordToRegionIndex.Find(NeighborCoord);
                
                if (NeighborRegionIndex && *NeighborRegionIndex != i)
                {
                    // These regions are adjacent - add to each other's adjacency lists
                    Region.AdjacentRegionIds.Add(Regions[*NeighborRegionIndex].Id);
                    Regions[*NeighborRegionIndex].AdjacentRegionIds.Add(Region.Id);
                }
            }
        }
    }
    
    UE_LOG(LogNav3D, Verbose, TEXT("Built voxel-level adjacency for %d regions"), Regions.Num());
}

FNav3DRegionBuilder FNav3DTacticalReasoning::MergeRegions(const FNav3DRegionBuilder& RegionA, 
                                                        const FNav3DRegionBuilder& RegionB)
{
    FNav3DRegionBuilder MergedRegion;
    MergedRegion.Id = RegionA.Id;  // Keep the ID of the first region
    MergedRegion.LayerIndex = RegionA.LayerIndex;
    
    // Merge coordinate bounds
    MergedRegion.MinCoord = FIntVector(
        FMath::Min(RegionA.MinCoord.X, RegionB.MinCoord.X),
        FMath::Min(RegionA.MinCoord.Y, RegionB.MinCoord.Y),
        FMath::Min(RegionA.MinCoord.Z, RegionB.MinCoord.Z)
    );
    
    MergedRegion.MaxCoord = FIntVector(
        FMath::Max(RegionA.MaxCoord.X, RegionB.MaxCoord.X),
        FMath::Max(RegionA.MaxCoord.Y, RegionB.MaxCoord.Y),
        FMath::Max(RegionA.MaxCoord.Z, RegionB.MaxCoord.Z)
    );
    
    // Merge Morton codes
    MergedRegion.MortonCodes = RegionA.MortonCodes;
    for (const uint64 Code : RegionB.MortonCodes)
    {
        MergedRegion.MortonCodes.Add(Code);
    }
    
    // Merge adjacency sets (take the union, removing self-references)
    MergedRegion.AdjacentRegionIds = RegionA.AdjacentRegionIds;
    for (int32 AdjId : RegionB.AdjacentRegionIds)
    {
        if (AdjId != RegionA.Id && AdjId != RegionB.Id)
        {
            MergedRegion.AdjacentRegionIds.Add(AdjId);
        }
    }
    
    return MergedRegion;
}

void FNav3DTacticalReasoning::UpdateAdjacentRegionReferences(
    TArray<FNav3DRegionBuilder>& Regions, const int32 RegionAIndex, const int32 RegionBId)
{
    int32 RegionAId = Regions[RegionAIndex].Id;
    
    for (int32 i = 0; i < Regions.Num(); ++i)
    {
        // Skip the merged region
        if (Regions[i].Id == RegionAId) continue;
        
        // If this region was adjacent to RegionB, make it adjacent to RegionA
        if (Regions[i].AdjacentRegionIds.Contains(RegionBId))
        {
            Regions[i].AdjacentRegionIds.Remove(RegionBId);
            Regions[i].AdjacentRegionIds.Add(RegionAId);
        }
    }
}

void FNav3DTacticalReasoning::BuildCrossLayerAdjacency(TArray<FNav3DRegion>& Regions, const FNav3DVolumeNavigationData* VolumeData)
{
    // Check each region pair for cross-layer adjacency
    for (int32 i = 0; i < Regions.Num(); ++i)
    {
        FNav3DRegion& RegionA = Regions[i];
        
        for (int32 j = i + 1; j < Regions.Num(); ++j)
        {
            FNav3DRegion& RegionB = Regions[j];
            
            // Skip if regions are in the same layer - already handled by BuildVoxelLevelAdjacency
            if (RegionA.LayerIndex == RegionB.LayerIndex)
                continue;
                
            // Check if these regions are adjacent
            if (AreRegionsAdjacent(RegionA, RegionB))
            {
                RegionA.AdjacentRegionIds.AddUnique(RegionB.Id);
                RegionB.AdjacentRegionIds.AddUnique(RegionA.Id);
                
                UE_LOG(LogNav3D, Verbose, TEXT("Cross-layer adjacency detected: Region %d (Layer %d) <-> Region %d (Layer %d)"),
                      RegionA.Id, RegionA.LayerIndex, RegionB.Id, RegionB.LayerIndex);
            }
        }
    }
    
    UE_LOG(LogNav3D, Verbose, TEXT("Built cross-layer adjacency for %d regions"), Regions.Num());
}

bool FNav3DTacticalReasoning::AreRegionsAdjacent(const FNav3DRegion& RegionA, const FNav3DRegion& RegionB)
{
    // Check for box adjacency
    const FBox& BoxA = RegionA.Bounds;
    const FBox& BoxB = RegionB.Bounds;
    
    // Use a more appropriate epsilon value based on typical voxel sizes
    constexpr float Epsilon = 2.0f;
    
    // Check if the boxes are touching along any axis
    const bool TouchX = FMath::Abs(BoxA.Max.X - BoxB.Min.X) <= Epsilon || 
                        FMath::Abs(BoxA.Min.X - BoxB.Max.X) <= Epsilon;
    
    const bool TouchY = FMath::Abs(BoxA.Max.Y - BoxB.Min.Y) <= Epsilon || 
                        FMath::Abs(BoxA.Min.Y - BoxB.Max.Y) <= Epsilon;
    
    const bool TouchZ = FMath::Abs(BoxA.Max.Z - BoxB.Min.Z) <= Epsilon || 
                        FMath::Abs(BoxA.Min.Z - BoxB.Max.Z) <= Epsilon;
    
    // Check for projection overlaps in the perpendicular planes
    // For two regions to be adjacent, they must have a significant face overlap, not just touch at corners
    
    // Calculate actual overlap amounts in each dimension
    float XOverlap = FMath::Min(BoxA.Max.X, BoxB.Max.X) - FMath::Max(BoxA.Min.X, BoxB.Min.X);
    float YOverlap = FMath::Min(BoxA.Max.Y, BoxB.Max.Y) - FMath::Max(BoxA.Min.Y, BoxB.Min.Y);
    float ZOverlap = FMath::Min(BoxA.Max.Z, BoxB.Max.Z) - FMath::Max(BoxA.Min.Z, BoxB.Min.Z);
    
    // Adjust for numerical precision 
    XOverlap = FMath::Max(0.0f, XOverlap);
    YOverlap = FMath::Max(0.0f, YOverlap);
    ZOverlap = FMath::Max(0.0f, ZOverlap);
    
    // Get the minimum dimension sizes for percentage calculations
    float MinXSize = FMath::Min(BoxA.Max.X - BoxA.Min.X, BoxB.Max.X - BoxB.Min.X);
    float MinYSize = FMath::Min(BoxA.Max.Y - BoxA.Min.Y, BoxB.Max.Y - BoxB.Min.Y);
    float MinZSize = FMath::Min(BoxA.Max.Z - BoxA.Min.Z, BoxB.Max.Z - BoxB.Min.Z);
    
    // Check for significant overlap (at least 25% of the smallest dimension)
    const bool OverlapYZ = (YOverlap >= 0.25f * MinYSize) && (ZOverlap >= 0.25f * MinZSize);
    const bool OverlapXZ = (XOverlap >= 0.25f * MinXSize) && (ZOverlap >= 0.25f * MinZSize);
    const bool OverlapXY = (XOverlap >= 0.25f * MinXSize) && (YOverlap >= 0.25f * MinYSize);
    
    // The boxes are adjacent if they touch on one axis and have significant overlap on the perpendicular plane
    return (TouchX && OverlapYZ) || (TouchY && OverlapXZ) || (TouchZ && OverlapXY);
}



