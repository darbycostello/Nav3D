// ReSharper disable CommentTypo
#pragma once
#include "Nav3D.h"
#include <CoreMinimal.h>
#include "Nav3DTypes.generated.h"

class UNav3DPathFindingSearch;
class UNav3DPathHeuristicCalculator;
class UNav3DPathTraversalCostCalculator;

using MortonCode = uint_fast64_t;
using LayerIndex = uint8;
using NodeIndex = uint32;
using LeafIndex = int32;
using SubNodeIndex = uint8;
using NeighbourDirection = uint8;

// Azimuth ranges from -180° to +180° (-π to π)
static constexpr int32 GNumAzimuthSamples = 16;

// Elevation ranges from -90° to +90° (-π/2 to π/2)
static constexpr int32 GNumElevationSamples = 8;

// Total directions from each node
static constexpr int32 GNumDirections = GNumAzimuthSamples * GNumElevationSamples;

// Orthogonal neighbour directions, expressed as unit vectors 
static const FIntVector GNeighbourDirections[6] = {
	{1, 0, 0}, {-1, 0, 0}, {0, 1, 0},
	{0, -1, 0}, {0, 0, 1}, {0, 0, -1}
};

DECLARE_DELEGATE_ThreeParams(FNav3DPathQueryDelegate, uint32, ENavigationQueryResult::Type, FNavPathSharedPtr);

USTRUCT()
struct FNav3DDataGenerationSettings
{
	GENERATED_BODY()

	FNav3DDataGenerationSettings()
	{
		CollisionChannel = ECC_WorldStatic;
		Clearance = 0.0f;

		CollisionQueryParameters.bFindInitialOverlaps = true;
		CollisionQueryParameters.bTraceComplex = false;
		CollisionQueryParameters.TraceTag = "Nav3DRasterize";
	}

	UPROPERTY(EditAnywhere, Category = "Nav3D")
	TEnumAsByte<ECollisionChannel> CollisionChannel;

	UPROPERTY(EditAnywhere, Category = "Nav3D")
	float Clearance;

	FCollisionQueryParams CollisionQueryParameters;
};

struct FNav3DNodeAddress
{
	FNav3DNodeAddress() : LayerIndex(15), NodeIndex(0), SubNodeIndex(0)
	{
	}

	explicit FNav3DNodeAddress(const int32 Index)
		: LayerIndex(Index << 28), NodeIndex(Index << 6), SubNodeIndex(Index)
	{
	}

	FNav3DNodeAddress(const LayerIndex InLayerIndex, const MortonCode InNodeIndex,
	                  const SubNodeIndex InSubNodeIndex = 0)
		: LayerIndex(InLayerIndex), NodeIndex(InNodeIndex),
		  SubNodeIndex(InSubNodeIndex)
	{
	}

	bool IsValid() const;
	void Invalidate();

	bool operator==(const FNav3DNodeAddress& Other) const
	{
		return LayerIndex == Other.LayerIndex && NodeIndex == Other.NodeIndex &&
			SubNodeIndex == Other.SubNodeIndex;
	}

	bool operator!=(const FNav3DNodeAddress& Other) const
	{
		return !operator==(Other);
	}

	NavNodeRef GetNavNodeRef() const
	{
		const int32 Address = LayerIndex << 28 | NodeIndex << 6 | SubNodeIndex;
		return static_cast<NavNodeRef>(Address);
	}

	FString ToString() const
	{
		return FString::Printf(TEXT("%i %i %i"), LayerIndex, NodeIndex,
		                       SubNodeIndex);
	}

	static const FNav3DNodeAddress InvalidAddress;

	uint8 LayerIndex : 4;
	uint_fast32_t NodeIndex : 22;
	uint8 SubNodeIndex : 6;
};

FORCEINLINE bool FNav3DNodeAddress::IsValid() const { return LayerIndex != 15; }

FORCEINLINE void FNav3DNodeAddress::Invalidate() { LayerIndex = 15; }

FORCEINLINE uint32 GetTypeHash(const FNav3DNodeAddress& Address)
{
	return HashCombine(HashCombine(GetTypeHash(Address.LayerIndex),
	                               GetTypeHash(Address.NodeIndex)),
	                   GetTypeHash(Address.SubNodeIndex));
}

FORCEINLINE FArchive& operator<<(FArchive& Archive, FNav3DNodeAddress& Data)
{
	Archive.Serialize(&Data, sizeof(FNav3DNodeAddress));
	return Archive;
}

struct FNav3DLeafNode
{
	void MarkSubNodeAsOccluded(const SubNodeIndex Index);
	bool IsSubNodeOccluded(const MortonCode InMortonCode) const;
	bool IsOccluded() const;
	bool IsCompletelyOccluded() const;
	bool IsCompletelyFree() const;

	uint_fast64_t SubNodes = 0;
	FNav3DNodeAddress Parent;
};

FORCEINLINE void
FNav3DLeafNode::MarkSubNodeAsOccluded(const SubNodeIndex Index)
{
	SubNodes |= 1ULL << Index;
}

FORCEINLINE bool
FNav3DLeafNode::IsSubNodeOccluded(const MortonCode InMortonCode) const
{
	return (SubNodes & 1ULL << InMortonCode) != 0;
}

FORCEINLINE bool FNav3DLeafNode::IsOccluded() const
{
	return SubNodes != 0;
}

FORCEINLINE bool FNav3DLeafNode::IsCompletelyOccluded() const
{
	return SubNodes == -1;
}

FORCEINLINE bool FNav3DLeafNode::IsCompletelyFree() const
{
	return SubNodes == 0;
}

FORCEINLINE FArchive& operator<<(FArchive& Archive, FNav3DLeafNode& Data)
{
	Archive << Data.SubNodes;
	Archive << Data.Parent;
	return Archive;
}

struct FNav3DNode
{
	FNav3DNode();
	explicit FNav3DNode(MortonCode InMortonCode);
	bool HasChildren() const;

	MortonCode MortonCode;
	FNav3DNodeAddress Parent;
	FNav3DNodeAddress FirstChild;
	FNav3DNodeAddress Neighbours[6];
};

FORCEINLINE bool FNav3DNode::HasChildren() const
{
	return FirstChild.IsValid();
}

FORCEINLINE bool operator<(const FNav3DNode& Left, const FNav3DNode& Right)
{
	return Left.MortonCode < Right.MortonCode;
}

FORCEINLINE FArchive& operator<<(FArchive& Archive, FNav3DNode& Data)
{
	Archive << Data.MortonCode;
	Archive << Data.Parent;
	Archive << Data.FirstChild;

	for (int32 NeighbourIndex = 0; NeighbourIndex < 6; NeighbourIndex++)
	{
		Archive << Data.Neighbours[NeighbourIndex];
	}

	return Archive;
}

class NAV3D_API FNav3DLeafNodes
{
public:
	friend FArchive& operator<<(FArchive& Archive, FNav3DLeafNodes& LeafNodes);
	friend class FNav3DVolumeNavigationData;
	friend class FNav3DData;

	const FNav3DLeafNode& GetLeafNode(const LeafIndex LeafIndex) const;
	const TArray<FNav3DLeafNode>& GetLeafNodes() const;
	float GetLeafNodeSize() const;
	float GetLeafNodeExtent() const;
	float GetLeafSubNodeSize() const;
	float GetLeafSubNodeExtent() const;

	int GetAllocatedSize() const;

private:
	FNav3DLeafNode& GetLeafNode(const LeafIndex LeafIndex);

	void Initialize(float LeafSize);
	void Reset();
	void AllocateLeafNodes(int LeafCount);
	void AddLeafNode(LeafIndex LeafIndex, SubNodeIndex SubNodeIndex,
	                 bool IsOccluded);
	void AddEmptyLeafNode();

	float LeafNodeSize;
	TArray<FNav3DLeafNode> LeafNodes;
};

FORCEINLINE const FNav3DLeafNode&
FNav3DLeafNodes::GetLeafNode(const LeafIndex LeafIndex) const
{
	return LeafNodes[LeafIndex];
}

FORCEINLINE const TArray<FNav3DLeafNode>&
FNav3DLeafNodes::GetLeafNodes() const
{
	return LeafNodes;
}

FORCEINLINE float FNav3DLeafNodes::GetLeafNodeSize() const
{
	return LeafNodeSize;
}

FORCEINLINE float FNav3DLeafNodes::GetLeafNodeExtent() const
{
	return GetLeafNodeSize() * 0.5f;
}

FORCEINLINE float FNav3DLeafNodes::GetLeafSubNodeSize() const
{
	return GetLeafNodeSize() * 0.25f;
}

FORCEINLINE float FNav3DLeafNodes::GetLeafSubNodeExtent() const
{
	return GetLeafSubNodeSize() * 0.5f;
}

FORCEINLINE FNav3DLeafNode&
FNav3DLeafNodes::GetLeafNode(const LeafIndex LeafIndex)
{
	return LeafNodes[LeafIndex];
}

FORCEINLINE FArchive& operator<<(FArchive& Archive,
                                 FNav3DLeafNodes& LeafNodes)
{
	Archive << LeafNodes.LeafNodes;
	Archive << LeafNodes.LeafNodeSize;
	return Archive;
}

class NAV3D_API FNav3DLayer
{
public:
	friend FArchive& operator<<(FArchive& Archive, FNav3DLayer& Layer);
	friend class FNav3DVolumeNavigationData;

	FNav3DLayer();
	FNav3DLayer(int MaxNodeCount, float NodeSize);

	const TArray<FNav3DNode>& GetNodes() const;
	int32 GetNodeCount() const;
	const FNav3DNode& GetNode(NodeIndex NodeIndex) const;
	float GetNodeSize() const;
	float GetNodeExtent() const;
	uint32 GetMaxNodeCount() const;
	int GetAllocatedSize() const;

private:
	TArray<FNav3DNode>& GetNodes();
	TArray<FNav3DNode> Nodes;
	int MaxNodeCount;
	float NodeSize;
};

FORCEINLINE const TArray<FNav3DNode>& FNav3DLayer::GetNodes() const
{
	return Nodes;
}

FORCEINLINE TArray<FNav3DNode>& FNav3DLayer::GetNodes() { return Nodes; }

FORCEINLINE int32 FNav3DLayer::GetNodeCount() const { return Nodes.Num(); }

FORCEINLINE const FNav3DNode&
FNav3DLayer::GetNode(const NodeIndex NodeIndex) const
{
	return Nodes[NodeIndex];
}

FORCEINLINE float FNav3DLayer::GetNodeSize() const { return NodeSize; }

FORCEINLINE float FNav3DLayer::GetNodeExtent() const
{
	return GetNodeSize() * 0.5f;
}

FORCEINLINE uint32 FNav3DLayer::GetMaxNodeCount() const { return MaxNodeCount; }

FORCEINLINE FArchive& operator<<(FArchive& Archive, FNav3DLayer& Layer)
{
	Archive << Layer.Nodes;
	Archive << Layer.NodeSize;
	return Archive;
}

USTRUCT()
struct NAV3D_API FNav3DVolumeDebugData
{
	GENERATED_BODY()

	FNav3DVolumeDebugData();

	UPROPERTY(EditInstanceOnly)
	uint8 bDebugDrawBounds : 1;

	UPROPERTY(EditInstanceOnly)
	uint8 bDebugDrawNodeCoords : 1;

	UPROPERTY(EditInstanceOnly)
	uint8 bDebugDrawMortonCodes : 1;

	UPROPERTY(EditInstanceOnly)
	uint8 bDebugDrawLayers : 1;

	UPROPERTY(EditInstanceOnly, meta = (EditCondition = "bDebugDrawLayers", ClampMin = "0", UIMin = "0"))
	uint8 LayerIndexToDraw;

	UPROPERTY(EditInstanceOnly, meta = (EditCondition = "bDebugDrawLayers"))
	uint8 bDebugDrawOccludedVoxels : 1;

	UPROPERTY(EditInstanceOnly, meta = (EditCondition = "bDebugDrawLayers"))
	uint8 bDebugDrawFreeVoxels : 1;

	UPROPERTY(EditInstanceOnly)
	uint8 bDebugDrawActivePaths : 1;
};

USTRUCT()
struct NAV3D_API FNav3DTacticalDebugData
{
	GENERATED_BODY()

	FNav3DTacticalDebugData();

	UPROPERTY(EditInstanceOnly)
	uint8 bDebugDrawRegions : 1;

	UPROPERTY(EditInstanceOnly)
	uint8 bDebugDrawRegionIds : 1;

	UPROPERTY(EditInstanceOnly)
	uint8 bDebugDrawAdjacencyGraph : 1;

	UPROPERTY(EditInstanceOnly)
	uint8 bDebugDrawVisibility : 1;

	// Region ID to view visibility lines from (-1 = disabled)
	UPROPERTY(EditInstanceOnly, meta = (ClampMin = "-1", UIMin = "-1"))
	int32 VisibilityViewRegionId;
	
	// Draw best cover from VisibilityViewRegionId to observer position
	UPROPERTY(EditInstanceOnly)
	uint8 bDrawBestCover : 1;
};

USTRUCT()
struct FNav3DTacticalSettings
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Nav3D")
	bool bEnableTacticalReasoning;

	// Region generation settings
	UPROPERTY(EditAnywhere, Category = "Nav3D|Regions", meta = (ClampMin = "0", UIMin = "0", ToolTip = "Free voxels below this layer index will not be used to create regions"))
	int32 MinRegioningLayer;

	UPROPERTY(EditAnywhere, Category = "Nav3D|Regions", meta = (ClampMin = "0", UIMin = "0", ToolTip = "Free voxels at or above this layer index will be clamped to this layer's voxel size"))
	int32 MaxRegioningLayer;

	// Sample-based visibility settings
	UPROPERTY(EditAnywhere, Category = "Nav3D|Visibility", meta = (ClampMin = "4", UIMin = "4", ToolTip = "Minimum number of sample points to generate per region."))
	int32 MinSamplesPerRegion;

	UPROPERTY(EditAnywhere, Category = "Nav3D|Visibility", meta = (ClampMin = "4", UIMin = "4", ToolTip = "Maximum number of sample points to generate per region, regardless of size."))
	int32 MaxSamplesPerRegion;

	UPROPERTY(EditAnywhere, Category = "Nav3D|Visibility", meta = (ClampMin = "0.1", UIMin = "0.1", ClampMax = "2.0", UIMax = "2.0", ToolTip = "Controls how sample count scales with region volume. Higher values create more samples in larger regions."))
	float RegionSampleDensityFactor;

	UPROPERTY(EditAnywhere, Category = "Nav3D|Visibility", meta = (ClampMin = "0.0", UIMin = "0.0", ClampMax = "1.0", UIMax = "1.0", ToolTip = "Minimum visibility score to consider a region visible (0.0 = fully occluded, 1.0 = fully visible)."))
	float VisibilityScoreThreshold;

	UPROPERTY(EditAnywhere, Category = "Nav3D|Visibility", meta = (ClampMin = "1", UIMin = "1", ClampMax = "20", UIMax = "20", ToolTip = "Minimum number of voxel occlusions to consider a raycast blocked."))
	int32 MinOcclusions;

	// Cover finding settings
	UPROPERTY(EditAnywhere, Category = "Nav3D|Cover", meta = (ClampMin = "500.0", UIMin = "500.0", ToolTip = "Maximum search distance when looking for cover positions."))
	float MaxCoverSearchDistance;

	UPROPERTY(EditAnywhere, Category = "Nav3D|Cover", meta = (ClampMin = "1", UIMin = "1", ToolTip = "Maximum number of raycasts to perform when validating cover positions."))
	int32 MaxCoverRaycasts;

	UPROPERTY(EditAnywhere, Category = "Nav3D|Debug")
	FNav3DTacticalDebugData TacticalDebugData;

	FNav3DTacticalSettings()
		: bEnableTacticalReasoning(false)
		, MinRegioningLayer(1)
		, MaxRegioningLayer(10)
		, MinSamplesPerRegion(8)
		, MaxSamplesPerRegion(32)
		, RegionSampleDensityFactor(0.5f)
		, VisibilityScoreThreshold(0.9f)
		, MinOcclusions(1)
		, MaxCoverSearchDistance(5000.0f)
		, MaxCoverRaycasts(16)
	{
	}
};

USTRUCT()
struct NAV3D_API FNav3DMetadata
{
	GENERATED_BODY()

	FNav3DMetadata()
		: VolumeLocation(ForceInit), bHasNavigationData(false),
		  LayerCount(INDEX_NONE)
	{
	}

	UPROPERTY(VisibleInstanceOnly)
	FVector VolumeLocation;

	UPROPERTY(VisibleInstanceOnly)
	uint8 bHasNavigationData : 1;

	UPROPERTY(VisibleInstanceOnly)
	int LayerCount;
};

USTRUCT()
struct NAV3D_API FNav3DMetadataList
{
	GENERATED_BODY()

	UPROPERTY(EditInstanceOnly)
	TArray<FNav3DMetadata> Metadata;
};

class NAV3D_API FNav3DData
{
public:
	friend FArchive& operator<<(FArchive& Archive, FNav3DData& Data);
	friend class FNav3DVolumeNavigationData;

	FNav3DData();

	int GetLayerCount() const;
	const FNav3DLayer& GetLayer(LayerIndex LayerIndex) const;
	const FNav3DLayer& GetLastLayer() const;
	const FNav3DLeafNodes& GetLeafNodes() const;
	const FBox& GetNavigationBounds() const;
	const FBox& GetVolumeBounds() const;
	float GetMaxDistance() const;
	bool IsValid() const;

	void Reset();
	int GetAllocatedSize() const;

	int32 GetTotalOccludedLeafNodes() const
	{
		int32 OccludedCount = 0;
		if (Layers.Num() == 0)
		{
			return 0;
		}

		const auto& LayerZero = Layers[0];
		UE_LOG(LogNav3D, VeryVerbose, TEXT("Checking %d nodes in layer 0"), LayerZero.GetNodes().Num());

		for (NodeIndex NodeIdx = 0; NodeIdx < static_cast<uint32>(LayerZero.GetNodes().Num()); NodeIdx++)
		{
			const auto& Node = LayerZero.GetNode(NodeIdx);
			if (Node.HasChildren() && Node.FirstChild.IsValid())
			{
				const auto& LeafNode = LeafNodes.GetLeafNode(Node.FirstChild.NodeIndex);
				// Check if there are ANY occluded subnodes
				if (LeafNode.IsOccluded())
				{
					OccludedCount++;
				}
			}
		}

		UE_LOG(LogNav3D, VeryVerbose, TEXT("Found %d occluded leaf nodes"), OccludedCount);
		return OccludedCount;
	}

private:
	FNav3DLayer& GetLayer(LayerIndex LayerIndex);
	FNav3DLeafNodes& GetLeafNodes();
	bool Initialize(float VoxelSize, const FBox& Bounds);
	void AddBlockedNode(LayerIndex LayerIndex, NodeIndex NodeIndex);
	const TArray<NodeIndex>& GetLayerBlockedNodes(LayerIndex LayerIndex) const;

	TArray<TArray<NodeIndex>> BlockedNodes;
	TArray<FNav3DLayer> Layers;
	FNav3DLeafNodes LeafNodes;
	FBox NavigationBounds;
	FBox VolumeBounds;
	uint8 bIsValid : 1;
};

FORCEINLINE int FNav3DData::GetLayerCount() const { return Layers.Num(); }

FORCEINLINE FNav3DLayer& FNav3DData::GetLayer(const LayerIndex LayerIndex)
{
	return Layers[LayerIndex];
}

FORCEINLINE const FNav3DLayer&
FNav3DData::GetLayer(const LayerIndex LayerIndex) const
{
	return Layers[LayerIndex];
}

FORCEINLINE const FNav3DLayer& FNav3DData::GetLastLayer() const
{
	return Layers.Last();
}

FORCEINLINE const FNav3DLeafNodes& FNav3DData::GetLeafNodes() const
{
	return LeafNodes;
}

FORCEINLINE FNav3DLeafNodes& FNav3DData::GetLeafNodes() { return LeafNodes; }

FORCEINLINE const FBox& FNav3DData::GetNavigationBounds() const
{
	return NavigationBounds;
}

FORCEINLINE const FBox& FNav3DData::GetVolumeBounds() const
{
	return VolumeBounds;
}

FORCEINLINE float FNav3DData::GetMaxDistance() const
{
	if (!VolumeBounds.IsValid)
	{
		return 0.0f;
	}
	const FVector Diagonal = VolumeBounds.Max - VolumeBounds.Min;
	return Diagonal.Size();
}

FORCEINLINE bool FNav3DData::IsValid() const
{
	return bIsValid && GetLayerCount() > 0;
}

FORCEINLINE const TArray<NodeIndex>&
FNav3DData::GetLayerBlockedNodes(const LayerIndex LayerIndex) const
{
	return BlockedNodes[LayerIndex];
}

FORCEINLINE FArchive& operator<<(FArchive& Archive, FNav3DData& Data)
{
	Archive << Data.Layers;
	Archive << Data.LeafNodes;
	Archive << Data.NavigationBounds;
	Archive << Data.VolumeBounds;

	return Archive;
}
