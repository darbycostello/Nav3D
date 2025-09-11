#include "Nav3DTypes.h"

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

	NavigationBounds = FBox::BuildAABB(VolumeBounds.GetCenter(),
	                                   FVector(NavigationBoundsSize * 0.5f));

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
	: bDebugDrawRegions(false)
	, bDebugDrawRegionIds(false)
	, bDebugDrawAdjacencyGraph(false)
	, bDebugDrawVisibility(false)
	, VisibilityViewRegionId(-1)
	, bDrawBestCover(false)
{
}

FNav3DVolumeDebugData::FNav3DVolumeDebugData()
	: bDebugDrawBounds(false)
	, bDebugDrawVolumes(false)
	, bDebugDrawAdjacency(false)
	, bDebugDrawLayers(false)
	, LayerIndexToDraw(0)
	, bDebugDrawOccludedVoxels(false)
	, bDebugDrawFreeVoxels(false)
	, bDebugDrawNodeCoords(false)
	, bDebugDrawMortonCodes(false)
{
}