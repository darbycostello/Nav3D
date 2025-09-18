#include "Nav3DTypes.h"

#include "Nav3D.h"
#include "Nav3DUtils.h"
#include "Nav3DVolumeNavigationData.h"

const FNav3DNodeAddress FNav3DNodeAddress::InvalidAddress;

void FNav3DLeafNodes::Initialize(const float LeafSize)
{
	LeafNodeSize = LeafSize;
}

void FNav3DLeafNodes::Reset() { LeafNodes.Reset(); }

FNav3DNode::FNav3DNode()
	: MortonCode(0), Parent(FNav3DNodeAddress::InvalidAddress),
	  FirstChild(FNav3DNodeAddress::InvalidAddress)
{
}

FNav3DNode::FNav3DNode(const ::MortonCode InMortonCode)
	: MortonCode(InMortonCode), Parent(FNav3DNodeAddress::InvalidAddress),
	  FirstChild(FNav3DNodeAddress::InvalidAddress)
{
}

int FNav3DLeafNodes::GetAllocatedSize() const
{
	return LeafNodes.Num() * sizeof(FNav3DLeafNode);
}

void FNav3DLeafNodes::AllocateLeafNodes(const int LeafCount)
{
	LeafNodes.Reserve(LeafCount);
}

void FNav3DLeafNodes::AddLeafNode(const LeafIndex LeafIndex,
                                  const SubNodeIndex SubNodeIndex,
                                  const bool IsOccluded)
{
	if (LeafIndex > LeafNodes.Num() - 1)
	{
		AddEmptyLeafNode();
	}

	if (IsOccluded)
	{
		LeafNodes[LeafIndex].MarkSubNodeAsOccluded(SubNodeIndex);
	}
}

void FNav3DLeafNodes::AddEmptyLeafNode() { LeafNodes.AddDefaulted(); }

FNav3DLayer::FNav3DLayer() : MaxNodeCount(-1), NodeSize(0.0f)
{
}

FNav3DLayer::FNav3DLayer(const int MaxNodeCount, const float NodeSize)
	: MaxNodeCount(MaxNodeCount), NodeSize(NodeSize)
{
}

int FNav3DLayer::GetAllocatedSize() const
{
	return Nodes.Num() * sizeof(FNav3DNode);
}

bool FNav3DData::Initialize(const float VoxelSize, const FBox& Bounds)
{
	Reset();

	VolumeBounds = Bounds;

	const auto VolumeSize = VolumeBounds.GetSize().GetAbsMax();

	const auto LeafSize = VoxelSize * 4;
	const auto VoxelExponent =
		FMath::CeilToInt(FMath::Log2(VolumeSize / LeafSize));
	const auto LayerCount = VoxelExponent + 1;

	if (LayerCount < 2)
	{
		bIsValid = false;
		return false;
	}

	LeafNodes.Initialize(LeafSize);

	const auto NavigationBoundsSize = FMath::Pow(2.0f, VoxelExponent) * LeafSize;
	NavigationBounds = FBox::BuildAABB(VolumeBounds.GetCenter(),
									   FVector(NavigationBoundsSize * 0.5f));
	UE_LOG(LogNav3D, Warning, TEXT("FNav3DData::Initialize - Calculated Navigation Bounds: %s"),
		*NavigationBounds.ToString());
	
	for (LayerIndex LayerIndex = 0; LayerIndex < LayerCount; ++LayerIndex)
	{
		const auto LayerEdgeNodeCount =
			FMath::Pow(2.0f, VoxelExponent - LayerIndex);
		const auto LayerMaxNodeCount =
			LayerEdgeNodeCount * LayerEdgeNodeCount * LayerEdgeNodeCount;
		// FMath::CeilToInt(FMath::Pow(LayerEdgeNodeCount, 3));
		const auto LayerVoxelSize = NavigationBoundsSize / LayerEdgeNodeCount;

		Layers.Emplace(LayerMaxNodeCount, LayerVoxelSize);
	}

	BlockedNodes.SetNumZeroed(LayerCount + 1);

	return true;
}

void FNav3DData::AddBlockedNode(const LayerIndex LayerIndex,
                                const NodeIndex NodeIndex)
{
	BlockedNodes[LayerIndex].Add(NodeIndex);
}

FNav3DData::FNav3DData() : LeafNodes(), bIsValid(false)
{
}

void FNav3DData::Reset()
{
	Layers.Reset();
	LeafNodes.Reset();
}

int FNav3DData::GetAllocatedSize() const
{
	int Size = LeafNodes.GetAllocatedSize();

	for (const auto& Layer : Layers)
	{
		Size += Layer.GetAllocatedSize();
	}

	return Size;
}

FNav3DTacticalDebugData::FNav3DTacticalDebugData()
	: bDebugDrawPortals(0)
    , bDebugDrawRegions(false)
	, bDebugDrawRegionIds(false)
	, bDebugDrawRegionAdjacency(false)
	, bDebugDrawVisibility(false)
	, VisibilityViewRegionId(-1)
	, bDrawBestCover(false)
{
}


FNav3DPerformanceStats::FNav3DPerformanceStats()
	: TotalRegions(0)
	, LoadedChunks(0)
	, TotalAdjacencies(0)
	, IntraChunkAdjacencies(0)
	, CrossChunkAdjacencies(0)
	, TotalVisibilityPairs(0)
	, EstimatedMemoryUsage(0.0f)
	, LastUpdateTime(0.0)
{
}

FNav3DVolumeDebugData::FNav3DVolumeDebugData()
	: bDebugDrawBounds(false)
	, bDebugDrawVolumes(false)
	, bDebugDrawLayers(false)
	, LayerIndexToDraw(0)
	, bDebugDrawOccludedVoxels(false)
	, bDebugDrawFreeVoxels(false)
	, bDebugDrawNodeCoords(false)
	, bDebugDrawMortonCodes(false)
{
}

void FConsolidatedTacticalData::Reset()
{
	AllLoadedRegions.Reset();
	RegionAdjacency.Reset();
	RegionVisibility.Reset();
	SourceChunks.Reset();
}

int32 FConsolidatedTacticalData::FindContainingRegion(const FVector& Position) const
{
	for (const FNav3DRegion& Region : AllLoadedRegions)
	{
		if (Region.Bounds.IsInside(Position))
		{
			return Region.Id;
		}
	}
	return -1;
}

FNav3DRegion FNav3DRegionBuilder::ToRegion(const FNav3DVolumeNavigationData* VolumeData) const
{
	// Calculate centers of min/max voxels
	const FVector MinPosCenter = VolumeData->GetNodePositionFromLayerAndMortonCode(
		LayerIndex, 
		FNav3DUtils::GetMortonCodeFromIntVector(MinCoord)
	);
    
	const FVector MaxPosCenter = VolumeData->GetNodePositionFromLayerAndMortonCode(
		LayerIndex, 
		FNav3DUtils::GetMortonCodeFromIntVector(MaxCoord)
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

FCompactRegion FNav3DRegionBuilder::ToCompactRegion() const
{
    // Convert to a temporary region first to get proper bounds
    // This uses the existing working ToRegion method
    FNav3DRegion TempRegion = ToRegion(nullptr); // We'll calculate bounds manually
    
    // Calculate bounds manually using the existing working logic
    FBox WorldBounds;
    if (MortonCodes.Num() > 0)
    {
        // Convert Morton codes to world positions (this would need VolumeData)
        // For now, use a reasonable default size
        const FVector EstimatedCenter = FVector(MinCoord + MaxCoord) * 50.0f; // Default voxel size
        const FVector EstimatedSize = FVector(GetSize()) * 100.0f; // Default voxel size
        
        WorldBounds = FBox::BuildAABB(EstimatedCenter, EstimatedSize * 0.5f);
    }
    else
    {
        // Fallback for empty regions
        WorldBounds = FBox::BuildAABB(FVector::ZeroVector, FVector(100.0f));
    }
    
    // Store center and size directly
    const FVector WorldCenter = WorldBounds.GetCenter();
    const FVector WorldSize = WorldBounds.GetSize();
    
    return FCompactRegion(LayerIndex, WorldCenter, WorldSize);
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

bool FConsolidatedTacticalData::IsRegionVisibilityMatch(const int32 ViewerRegionId, const int32 TargetRegionId, ETacticalVisibility Visibility) const
{
	if (ViewerRegionId == TargetRegionId)
	{
		return true; // Always visible to self
	}
	
	// Check if TargetRegionId is in ViewerRegionId's visibility set
	if (const FRegionIdArray* VisibilitySet = RegionVisibility.Find(ViewerRegionId))
	{
		return VisibilitySet->Contains(TargetRegionId);
	}
	
	return false;
}

void FConsolidatedTacticalData::AddToVisibilitySet(const int32 ViewerRegionId, const int32 TargetRegionId)
{
	if (ViewerRegionId == TargetRegionId)
	{
		return; // Don't add self to visibility set
	}
	
	FRegionIdArray& VisibilitySet = RegionVisibility.FindOrAdd(ViewerRegionId);
	VisibilitySet.Add(TargetRegionId);
}

const FNav3DRegion* FConsolidatedTacticalData::GetRegionById(const int32 RegionId) const
{
	for (const FNav3DRegion& Region : AllLoadedRegions)
	{
		if (Region.Id == RegionId)
		{
			return &Region;
		}
	}
	return nullptr;
}
