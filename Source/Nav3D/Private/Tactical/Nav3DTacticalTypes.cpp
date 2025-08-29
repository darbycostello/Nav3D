#include "Tactical/Nav3DTacticalTypes.h"
#include "Nav3DUtils.h"
#include "Nav3DVolumeNavigationData.h"

FNav3DRegion FNav3DRegionBuilder::ToRegion(const FNav3DVolumeNavigationData* VolumeData) const
{
    // Calculate centers of min/max voxels
    const FVector MinPosCenter = VolumeData->GetNodePositionFromLayerAndMortonCode(
        LayerIndex, 
        FNav3DUtils::GetMortonCodeFromVector(MinCoord)
    );
    
    const FVector MaxPosCenter = VolumeData->GetNodePositionFromLayerAndMortonCode(
        LayerIndex, 
        FNav3DUtils::GetMortonCodeFromVector(MaxCoord)
    );
    
    // Get node extent
    const float NodeExtent = VolumeData->GetData().GetLayer(LayerIndex).GetNodeExtent();
    
    // Create bounds properly from centers to corners
    const FBox WorldBounds(
        MinPosCenter - FVector(NodeExtent),  // Min corner = min center - extent
        MaxPosCenter + FVector(NodeExtent)   // Max corner = max center + extent
    );
    
    FNav3DRegion Region(Id, WorldBounds, LayerIndex);
    
    // Copy adjacency information
    for (int32 AdjId : AdjacentRegionIds)
    {
        Region.AdjacentRegionIds.Add(AdjId);
    }
    
    return Region;
}

FNav3DRegionBuilder FBoxRegion::ToRegionBuilder(const TArray<TPair<uint64, FIntVector>>& FreeVoxels) const
{
    FNav3DRegionBuilder Builder;
    Builder.Id = Id;
    Builder.LayerIndex = LayerIndex;
    Builder.MinCoord = Min;
    Builder.MaxCoord = Max;
    
    // Add all morton codes for voxels in this region
    for (const auto& VoxelPair : FreeVoxels)
    {
        if (Contains(VoxelPair.Value))
        {
            Builder.MortonCodes.Add(VoxelPair.Key);
        }
    }
    
    return Builder;
}

bool FBoxRegion::Contains(const FIntVector& Coord) const
{
    return 
        Coord.X >= Min.X && Coord.X <= Max.X &&
        Coord.Y >= Min.Y && Coord.Y <= Max.Y &&
        Coord.Z >= Min.Z && Coord.Z <= Max.Z;
}

int32 FBoxRegion::GetVolume() const
{
    return (Max.X - Min.X + 1) * (Max.Y - Min.Y + 1) * (Max.Z - Min.Z + 1);
}

bool FNav3DTacticalData::IsRegionVisibilityMatch(int32 ViewerRegionId, int32 TargetRegionId, ETacticalVisibility VisibilityType) const
{
    // Find the viewer region
    const FNav3DRegion* ViewerRegion = nullptr;
    for (const auto& Region : Regions)
    {
        if (Region.Id == ViewerRegionId)
        {
            ViewerRegion = &Region;
            break;
        }
    }
    
    if (!ViewerRegion)
    {
        return false;
    }
    
    // Find the target region
    const FNav3DRegion* TargetRegion = nullptr;
    for (const auto& Region : Regions)
    {
        if (Region.Id == TargetRegionId)
        {
            TargetRegion = &Region;
            break;
        }
    }
    
    if (!TargetRegion)
    {
        return false;
    }
    
    // Check visibility based on the requested type
    bool ViewerCanSeeTarget = ViewerRegion->VisibilitySet.Contains(TargetRegionId);
    bool TargetCanSeeViewer = TargetRegion->VisibilitySet.Contains(ViewerRegionId);
    
    switch (VisibilityType)
    {
        case ETacticalVisibility::TargetVisible:
            return ViewerCanSeeTarget;
        
        case ETacticalVisibility::MutuallyVisible:
            return ViewerCanSeeTarget && TargetCanSeeViewer;
        
        case ETacticalVisibility::TargetOccluded:
            return !ViewerCanSeeTarget;
        
        case ETacticalVisibility::MutuallyOccluded:
            return !ViewerCanSeeTarget && !TargetCanSeeViewer;
        
        default:
            return false;
    }
}

void FNav3DTacticalData::AddToVisibilitySet(const int32 ViewerRegionId, const int32 VisibleRegionId)
{
    for (auto& Region : Regions)
    {
        if (Region.Id == ViewerRegionId)
        {
            if (!Region.VisibilitySet.Contains(VisibleRegionId))
            {
                Region.VisibilitySet.Add(VisibleRegionId);
            }
            break;
        }
    }
}

int32 FNav3DTacticalData::FindContainingRegion(const FVector& Position) const
{
    for (const auto& Region : Regions)
    {
        if (Region.Bounds.IsInside(Position))
        {
            return Region.Id;
        }
    }
    
    return -1; // No containing region found
}