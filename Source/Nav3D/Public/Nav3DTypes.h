#pragma once
#include "Nav3DTypes.generated.h"

class UNav3DPathFindingSearch;
class UNav3DPathHeuristicCalculator;
class UNav3DPathTraversalCostCalculator;
class ANav3DDataChunkActor;

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

enum class NAV3D_API ENav3DVersion : uint8
{
	V_2025090900,
	MinCompatible = V_2025090900,
	Latest = V_2025090900
};

USTRUCT()
struct FNav3DDataGenerationSettings
{
	GENERATED_BODY()

	FNav3DDataGenerationSettings()
	{
		CollisionChannel = ECC_WorldStatic;
		Clearance = 0.0f;
		AdjacencyClearance = 500.0f;

		CollisionQueryParameters.bFindInitialOverlaps = true;
		CollisionQueryParameters.bTraceComplex = false;
		CollisionQueryParameters.TraceTag = "Nav3DRasterize";
	}

	UPROPERTY(EditAnywhere, Category = "Nav3D")
	TEnumAsByte<ECollisionChannel> CollisionChannel;

	UPROPERTY(EditAnywhere, Category = "Nav3D")
	float Clearance;

	UPROPERTY(EditAnywhere, Category = "Nav3D", meta = (
	    DisplayName = "Adjacency Clearance",
	    ToolTip = "Additional clearance distance added to layer-specific voxel extents for adjacency calculations"
	))
	float AdjacencyClearance = 500.0f;

	FCollisionQueryParams CollisionQueryParameters;


	// Maximum number of simultaneous box generation jobs (threaded tasks)
	UPROPERTY(EditAnywhere, Category = "Generation", meta = (
	    DisplayName = "Max Simultaneous Jobs",
	    ToolTip = "Maximum number of simultaneous box generation jobs",
	    ClampMin = "1"
	))
	int32 MaxSimultaneousBoxGenerationJobsCount = 4;
};

// Cache structure for storing overlap results per Layer 1 voxel
USTRUCT()
struct NAV3D_API FVoxelOverlapCache
{
	GENERATED_BODY()
	
	// Layer 1 Morton code this cache entry represents
	MortonCode Layer1MortonCode;
	
	// All actors that overlap this Layer 1 voxel
	TArray<TWeakObjectPtr<AActor>> OverlappingActors;
	
	// Bounds of this Layer 1 voxel for reference
	FBox VoxelBounds;
	
	FVoxelOverlapCache()
		: Layer1MortonCode(0)
		, VoxelBounds(ForceInit)
	{
	}
	
	FVoxelOverlapCache(MortonCode InMortonCode, const FBox& InBounds)
		: Layer1MortonCode(InMortonCode)
		, VoxelBounds(InBounds)
	{
	}
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

USTRUCT()
struct FRegionIdList
{
	GENERATED_BODY()
	
	UPROPERTY()
	TArray<int32> Ids;
};

USTRUCT()
struct NAV3D_API FNav3DVolumeDebugData
{
	GENERATED_BODY()

	FNav3DVolumeDebugData();

	UPROPERTY(EditInstanceOnly)
	uint8 bDebugDrawBounds : 1;

	UPROPERTY(EditInstanceOnly)
	uint8 bDebugDrawVolumes : 1;

	UPROPERTY(EditInstanceOnly)
	uint8 bDebugDrawLayers : 1;

	UPROPERTY(EditInstanceOnly, meta = (ClampMin = "0", UIMin = "0"))
	uint8 LayerIndexToDraw;

	UPROPERTY(EditInstanceOnly, meta = (EditCondition = "bDebugDrawLayers"))
	uint8 bDebugDrawOccludedVoxels : 1;

	UPROPERTY(EditInstanceOnly, meta = (EditCondition = "bDebugDrawLayers"))
	uint8 bDebugDrawFreeVoxels : 1;

	UPROPERTY(EditInstanceOnly, meta = (EditCondition = "bDebugDrawLayers"))
	uint8 bDebugDrawNodeCoords : 1;

	UPROPERTY(EditInstanceOnly, meta = (EditCondition = "bDebugDrawLayers"))
	uint8 bDebugDrawMortonCodes : 1;
};

USTRUCT()
struct NAV3D_API FNav3DTacticalDebugData
{
	GENERATED_BODY()

	FNav3DTacticalDebugData();

	UPROPERTY(EditInstanceOnly)
	uint8 bDebugDrawPortals : 1;

	UPROPERTY(EditInstanceOnly)
	uint8 bDebugDrawRegions : 1;

	UPROPERTY(EditInstanceOnly)
	uint8 bDebugDrawRegionIds : 1;
	
	UPROPERTY(EditInstanceOnly)
	uint8 bDebugDrawRegionAdjacency : 1;
	
	UPROPERTY(EditInstanceOnly)
	uint8 bDebugDrawVisibility : 1;

	// Region ID to view visibility lines from (-1 = disabled)
	UPROPERTY(EditInstanceOnly, meta = (EditCondition = "bDebugDrawVisibility", ClampMin = "-1", UIMin = "-1"))
	int32 VisibilityViewRegionId;
	
	// Draw best cover from VisibilityViewRegionId to observer position
	UPROPERTY(EditInstanceOnly, meta = (EditCondition = "bDebugDrawVisibility"))
	uint8 bDrawBestCover : 1;
};


USTRUCT()
struct NAV3D_API FNav3DPerformanceStats
{
	GENERATED_BODY()

	FNav3DPerformanceStats();

	// Region statistics
	UPROPERTY(VisibleAnywhere, Category="Regions")
	int32 TotalRegions;

	UPROPERTY(VisibleAnywhere, Category="Regions")
	int32 LoadedChunks;

	// Adjacency statistics
	UPROPERTY(VisibleAnywhere, Category="Adjacency")
	int32 TotalAdjacencies;

	UPROPERTY(VisibleAnywhere, Category="Adjacency")
	int32 IntraChunkAdjacencies;

	UPROPERTY(VisibleAnywhere, Category="Adjacency")
	int32 CrossChunkAdjacencies;

	// Visibility statistics
	UPROPERTY(VisibleAnywhere, Category="Visibility")
	int32 TotalVisibilityPairs;

	// Memory usage
	UPROPERTY(VisibleAnywhere, Category="Memory")
	float EstimatedMemoryUsage;

	// Timing
	UPROPERTY(VisibleAnywhere, Category="Timing")
	double LastUpdateTime;
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
	UPROPERTY(EditAnywhere, Category = "Nav3D|Visibility", meta = (ClampMin = "1", UIMin = "4", ToolTip = "Minimum number of sample points to generate per region."))
	int32 MinSamplesPerRegion;

	UPROPERTY(EditAnywhere, Category = "Nav3D|Visibility", meta = (ClampMin = "1", UIMin = "4", ToolTip = "Maximum number of sample points to generate per region, regardless of size."))
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

	UPROPERTY(EditAnywhere, Category = "Nav3D|Debug", meta = (EditCondition = "bEnableTacticalReasoning"))
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

USTRUCT()
struct FNav3DVoxelConnection
{
	GENERATED_BODY()

	FNav3DVoxelConnection()
		: Local(0)
		, LocalVolumeIndex(0)
		, LocalChunkIndex(0)
		, Remote(0)
		, RemoteVolumeIndex(0)
		, RemoteChunkIndex(0)
		, Distance(0.0f)
	{
	}

	UPROPERTY()
	uint64 Local;

	UPROPERTY()
	int32 LocalVolumeIndex;

	UPROPERTY()
	int32 LocalChunkIndex;

	UPROPERTY()
	uint64 Remote;

	UPROPERTY()
	int32 RemoteVolumeIndex;

	UPROPERTY()
	int32 RemoteChunkIndex;

	UPROPERTY()
	float Distance;
};


USTRUCT()
struct FNav3DActorPortal
{
	GENERATED_BODY()

	UPROPERTY()
	TWeakObjectPtr<ANav3DDataChunkActor> From;

	UPROPERTY()
	TWeakObjectPtr<ANav3DDataChunkActor> To;

	UPROPERTY()
	FNav3DVoxelConnection Connection;
};

USTRUCT()
struct NAV3D_API FCompactPortal
{
    GENERATED_BODY()

    UPROPERTY()
    uint64 Local = 0;

    UPROPERTY()
    uint64 Remote = 0;
};

USTRUCT()
struct FNav3DChunkAdjacency
{
	GENERATED_BODY()
	
	// Target chunk actor reference
	UPROPERTY()
	TWeakObjectPtr<ANav3DDataChunkActor> OtherChunkActor;
	
	// Compact, serialized portals (preferred minimal format)
	UPROPERTY()
	TArray<FCompactPortal> CompactPortals;
	
	// Spatial relationship data
	UPROPERTY()
	FVector SharedFaceNormal; // Direction from this chunk to adjacent chunk
	
	UPROPERTY()
	float ConnectionWeight; // Heuristic cost modifier
	
	// Runtime validity check
	bool IsValid() const { return OtherChunkActor.IsValid(); }
	
	// Default constructor
	FNav3DChunkAdjacency()
		: SharedFaceNormal(FVector::ZeroVector)
		, ConnectionWeight(1.0f)
	{
	}
};

// =============================================================================
// TACTICAL REASONING TYPES
// =============================================================================

// Visibility relationship between observers and targets
UENUM(BlueprintType)
enum class ETacticalVisibility : uint8
{
    TargetVisible    UMETA(DisplayName = "Target Visible to Observer"),
    MutuallyVisible  UMETA(DisplayName = "Mutually Visible"),
    TargetOccluded   UMETA(DisplayName = "Target Occluded from Observer"),
    MutuallyOccluded UMETA(DisplayName = "Mutually Occluded")
};

// Distance preference for tactical queries
UENUM(BlueprintType)
enum class ETacticalDistance : uint8
{
    Any     UMETA(DisplayName = "Any Distance"),
    Closest UMETA(DisplayName = "Closest"),
    Median  UMETA(DisplayName = "Median Distance"),
    Furthest UMETA(DisplayName = "Furthest")
};

// Region size preference for tactical queries
UENUM(BlueprintType)
enum class ETacticalRegion : uint8
{
    Any      UMETA(DisplayName = "Any Size"),
    Smallest UMETA(DisplayName = "Smallest"),
    Median   UMETA(DisplayName = "Medium Sized"),
    Largest  UMETA(DisplayName = "Largest")
};

// Struct to hold information about position candidates
USTRUCT(BlueprintType)
struct FPositionCandidate
{
    GENERATED_BODY()

    // Region ID this candidate belongs to
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Nav3D|Tactical")
    int32 RegionId;

    // Position in world space
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Nav3D|Tactical")
    FVector Position;

    // Path distance from start position (through region graph)
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Nav3D|Tactical")
    float PathDistance;

    // Direct distance (as the crow flies) from start position
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Nav3D|Tactical")
    float DirectDistance;

    // Region size (volume)
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Nav3D|Tactical")
    float RegionSize;

    // Overall score (higher is better)
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Nav3D|Tactical")
    float Score;

    // Constructor
    FPositionCandidate()
        : RegionId(-1)
        , Position(FVector::ZeroVector)
        , PathDistance(0.0f)
        , DirectDistance(0.0f)
        , RegionSize(0.0f)
        , Score(0.0f)
    {}
};

// A region is a box-shaped volume of free space
// @deprecated Use FCompactRegion instead for new code. This structure is kept for backward compatibility.
USTRUCT(BlueprintType)
struct FNav3DRegion
{
    GENERATED_BODY()

    // Unique identifier for this region
    UPROPERTY()
    int32 Id;

    // The region's bounds in world space
    UPROPERTY()
    FBox Bounds;

    // SVO layer index this region belongs to
    UPROPERTY()
    int32 LayerIndex;

    // List of adjacent region IDs
    UPROPERTY()
    TArray<int32> AdjacentRegionIds;

    UPROPERTY()
    TArray<int32> VisibilitySet;

    FNav3DRegion()
        : Id(-1)
        , Bounds(ForceInit)
        , LayerIndex(-1) {}

    FNav3DRegion(const int32 InId, const FBox& InBounds, const int32 InLayerIndex)
        : Id(InId)
        , Bounds(InBounds)
        , LayerIndex(InLayerIndex) {}

    friend FArchive& operator<<(FArchive& Ar, FNav3DRegion& Region)
    {
        Ar << Region.Id;
        Ar << Region.Bounds;
        Ar << Region.LayerIndex;
        Ar << Region.AdjacentRegionIds;
        Ar << Region.VisibilitySet;
    
        return Ar;
    }

    friend FArchive& operator<<(FArchive& Ar, FNav3DRegion& Region);
};

// Helper struct for region construction
USTRUCT()
struct NAV3D_API FNav3DRegionBuilder
{
    GENERATED_BODY()
    
    // Region ID
    int32 Id = -1;
    
    // SVO layer index
    int32 LayerIndex = 0;
    
    // Min/max coordinates in grid space
    FIntVector MinCoord = FIntVector::ZeroValue;
    FIntVector MaxCoord = FIntVector::ZeroValue;
    
    // Set of Morton codes contained in this region
    TArray<uint64> MortonCodes;
    
    // Set of adjacent region IDs
    TSet<int32> AdjacentRegionIds;

    FNav3DRegionBuilder() = default;
    
    FNav3DRegionBuilder(int32 InId, int32 InLayer)
        : Id(InId), LayerIndex(InLayer)
    {
        MortonCodes.Empty();
        AdjacentRegionIds.Empty();
    }
    
    // Convert to final FNav3DRegion
    FNav3DRegion ToRegion(const FNav3DVolumeNavigationData* VolumeData) const;
    
    // Convert to compact region with center and size
    FCompactRegion ToCompactRegion() const;
    
    // Utility methods
    bool IsValid() const { return Id >= 0 && MortonCodes.Num() > 0; }
    int32 GetVoxelCount() const { return MortonCodes.Num(); }
    FIntVector GetSize() const { return MaxCoord - MinCoord + FIntVector(1, 1, 1); }
};

USTRUCT()
struct NAV3D_API FBoxRegion
{
    GENERATED_BODY()
    
    int32 Id = -1;
    FIntVector Min = FIntVector::ZeroValue;
    FIntVector Max = FIntVector::ZeroValue;
    int32 LayerIndex = 0;
    
    FBoxRegion() = default;
    
    FBoxRegion(int32 InId, const FIntVector& InMin, const FIntVector& InMax, int32 InLayerIndex)
        : Id(InId), Min(InMin), Max(InMax), LayerIndex(InLayerIndex) {}
    
    int32 GetVolume() const 
    { 
        return (Max.X - Min.X + 1) * (Max.Y - Min.Y + 1) * (Max.Z - Min.Z + 1); 
    }
    
    FIntVector GetSize() const { return Max - Min + FIntVector(1, 1, 1); }
    FVector GetCenter() const { return FVector(Min + Max) * 0.5f; }
    bool IsValid() const { return Id >= 0 && Min.X <= Max.X && Min.Y <= Max.Y && Min.Z <= Max.Z; }
    
    // Convert to region builder
    FNav3DRegionBuilder ToRegionBuilder(const TArray<TPair<uint64, FIntVector>>& FreeVoxels) const;

	bool Contains(const FIntVector& Coord) const
	{
		return 
			Coord.X >= Min.X && Coord.X <= Max.X &&
			Coord.Y >= Min.Y && Coord.Y <= Max.Y &&
			Coord.Z >= Min.Z && Coord.Z <= Max.Z;
	}
};

/**
 * Wrapper for TArray<int32> to enable TMap values to be reflected by UE
 */
USTRUCT()
struct NAV3D_API FRegionIdArray
{
	GENERATED_BODY()
    
	UPROPERTY()
	TArray<int32> RegionIds;
    
	FRegionIdArray() = default;
    
	FRegionIdArray(const TArray<int32>& InRegionIds) : RegionIds(InRegionIds) {}
    
	void Add(int32 RegionId) { RegionIds.AddUnique(RegionId); }
	void Remove(int32 RegionId) { RegionIds.Remove(RegionId); }
	bool Contains(int32 RegionId) const { return RegionIds.Contains(RegionId); }
	int32 Num() const { return RegionIds.Num(); }
	bool IsEmpty() const { return RegionIds.IsEmpty(); }
	void Empty() { RegionIds.Empty(); }
    
	// Access to underlying array
	const TArray<int32>& GetArray() const { return RegionIds; }
	TArray<int32>& GetArray() { return RegionIds; }
    
	// Iterator support
	auto begin() const { return RegionIds.begin(); }
	auto end() const { return RegionIds.end(); }
};

/**
 * Local tactical data stored in each chunk actor (serialized)
 * @deprecated Use FCompactTacticalData instead for new code. This structure is kept for backward compatibility.
 */
USTRUCT()
struct NAV3D_API FLocalTacticalData
{
    GENERATED_BODY()
    
    UPROPERTY()
    TArray<FNav3DRegion> LocalRegions;
    
    UPROPERTY()
    TMap<int32, FRegionIdArray> IntraChunkAdjacency;
    
    FLocalTacticalData()
    {
        LocalRegions.Empty();
        IntraChunkAdjacency.Empty();
    }
    
    bool IsEmpty() const { return LocalRegions.Num() == 0; }
    void Reset() 
    { 
        LocalRegions.Empty(); 
        IntraChunkAdjacency.Empty(); 
    }
};

/**
 * Boundary connection interfaces between chunks (serialized)
 */
USTRUCT()
struct NAV3D_API FChunkConnectionInterface
{
    GENERATED_BODY()
    
    UPROPERTY()
    FVector ChunkFaceNormal = FVector::ZeroVector;
    
    UPROPERTY()
    TArray<int32> BoundaryRegionIds;
    
    UPROPERTY()
    TMap<int32, FBox> RegionBoundaryBoxes;
    
    FChunkConnectionInterface() = default;
    
    FChunkConnectionInterface(const FVector& FaceNormal)
        : ChunkFaceNormal(FaceNormal)
    {
        BoundaryRegionIds.Empty();
        RegionBoundaryBoxes.Empty();
    }
    
    bool IsEmpty() const { return BoundaryRegionIds.Num() == 0; }
};

USTRUCT(BlueprintType)
struct NAV3D_API FNav3DTacticalData
{
	GENERATED_BODY()

	// All regions in the system
	UPROPERTY()
	TArray<FNav3DRegion> Regions;

	friend FArchive& operator<<(FArchive& Ar, FNav3DTacticalData& TacticalData);

	bool IsRegionVisibilityMatch(int32 ViewerRegionId, int32 TargetRegionId, ETacticalVisibility VisibilityType) const;

	// Add a region ID to a region's visibility set
	void AddToVisibilitySet(const int32 ViewerRegionId, const int32 VisibleRegionId);

	// Find which region contains a position
	int32 FindContainingRegion(const FVector& Position) const;

	friend FArchive& operator<<(FArchive& Ar, FNav3DTacticalData& TacticalData)
	{
		// Serialize the regions array
		Ar << TacticalData.Regions;
		return Ar;
	}
};

/**
 * Data structure for region pruning analysis
 */
USTRUCT()
struct NAV3D_API FRegionPruningData
{
    GENERATED_BODY()
    
    UPROPERTY()
    int32 RegionId;
    
    UPROPERTY()
    float TacticalScore;
    
    UPROPERTY()
    FVector Position;
    
    UPROPERTY()
    float Volume;
    
    UPROPERTY()
    int32 VisibilityCount;      // How many regions this can see
    
    UPROPERTY()
    int32 VisibleFromCount;     // How many regions can see this
    
    UPROPERTY()
    int32 AdjacencyCount;       // Number of adjacent regions
    
    UPROPERTY()
    float ElevationRank;        // Normalized height (0=lowest, 1=highest)
    
    UPROPERTY()
    float DistanceVariance;     // How unique its distance profile is
    
    UPROPERTY()
    bool bIsBoundaryRegion;     // Near volume edges
    
    UPROPERTY()
    bool bIsChokepoint;         // Low adjacency relative to size
    
    FRegionPruningData()
        : RegionId(-1)
        , TacticalScore(0.0f)
        , Position(FVector::ZeroVector)
        , Volume(0.0f)
        , VisibilityCount(0)
        , VisibleFromCount(0)
        , AdjacencyCount(0)
        , ElevationRank(0.0f)
        , DistanceVariance(0.0f)
        , bIsBoundaryRegion(false)
        , bIsChokepoint(false)
    {}
};

/**
 * Data structure for density-focused region pruning analysis
 */
USTRUCT()
struct NAV3D_API FDensityRegionPruningData
{
    GENERATED_BODY()
    
    UPROPERTY()
    int32 RegionId;
    
    UPROPERTY()
    float TacticalComplexityScore;
    
    UPROPERTY()
    FVector Position;
    
    UPROPERTY()
    float Volume;
    
    // Geometry density indicators
    UPROPERTY()
    float LocalGeometryDensity;     // Nearby occluded voxel ratio
    
    UPROPERTY()
    float VisibilityComplexity;     // How varied the visibility is
    
    UPROPERTY()
    float AdjacencyComplexity;      // How complex the adjacency pattern is
    
    UPROPERTY()
    float GeometryProximity;        // How close to dense geometry
    
    // Traditional metrics (lower weight)
    UPROPERTY()
    int32 VisibilityCount;
    
    UPROPERTY()
    int32 VisibleFromCount; 
    
    UPROPERTY()
    int32 AdjacencyCount;
    
    UPROPERTY()
    float ElevationRank;
    
    UPROPERTY()
    bool bIsBoundaryRegion;
    
    UPROPERTY()
    bool bIsChokepoint;
    
    FDensityRegionPruningData()
        : RegionId(-1)
        , TacticalComplexityScore(0.0f)
        , Position(FVector::ZeroVector)
        , Volume(0.0f)
        , LocalGeometryDensity(0.0f)
        , VisibilityComplexity(0.0f)
        , AdjacencyComplexity(0.0f)
        , GeometryProximity(0.0f)
        , VisibilityCount(0)
        , VisibleFromCount(0)
        , AdjacencyCount(0)
        , ElevationRank(0.0f)
        , bIsBoundaryRegion(false)
        , bIsChokepoint(false)
    {}
};

/**
 * Consolidated tactical data built from loaded chunks at runtime (transient)
 * @deprecated Use FConsolidatedCompactTacticalData instead for new code. This structure is kept for backward compatibility.
 */
USTRUCT()
struct NAV3D_API FConsolidatedTacticalData
{
    GENERATED_BODY()
    
    UPROPERTY()
    TArray<FNav3DRegion> AllLoadedRegions;
    
    UPROPERTY()
    TMap<int32, FRegionIdArray> RegionAdjacency;
    
    UPROPERTY()
    TMap<int32, FRegionIdArray> RegionVisibility;
    
    TSet<TWeakObjectPtr<ANav3DDataChunkActor>> SourceChunks;
    
    // Runtime tactical query methods
    int32 FindContainingRegion(const FVector& Position) const;
    bool IsRegionVisibilityMatch(int32 ViewerRegionId, int32 TargetRegionId, ETacticalVisibility Visibility) const;
    void AddToVisibilitySet(int32 ViewerRegionId, int32 TargetRegionId);
    const FNav3DRegion* GetRegionById(int32 RegionId) const;
    
    // Utility methods
    void Reset();
    bool IsEmpty() const { return AllLoadedRegions.Num() == 0; }
    int32 GetRegionCount() const { return AllLoadedRegions.Num(); }
    int32 GetSourceChunkCount() const { return SourceChunks.Num(); }
    
    
    FConsolidatedTacticalData()
    {
        Reset();
    }
};

// =============================================================================
// COMPACT TACTICAL DATA STRUCTURES
// =============================================================================

/**
 * Compact region representation optimized for tactical queries
 * Replaces FNav3DRegion with minimal memory footprint
 * 
 * Uses Center and Size for perfect bounds reconstruction without coordinate conversion issues
 */
USTRUCT()
struct NAV3D_API FCompactRegion
{
	GENERATED_BODY()
    
	uint8 LayerIndex = 0;
    
	// World center and size for exact bounds reconstruction
	UPROPERTY()
	FVector Center = FVector::ZeroVector;
    
	UPROPERTY()
	FVector Size = FVector::ZeroVector;
    
	FCompactRegion() = default;
    
	FCompactRegion(uint8 InLayerIndex, const FVector& InCenter, const FVector& InSize)
		: LayerIndex(InLayerIndex), Center(InCenter), Size(InSize) {}
    
	// Core interface
	FVector GetWorldCenter() const { return Center; }
	FVector GetWorldSize() const { return Size; }
    
	// Perfect bounds reconstruction
	FBox GetWorldBounds() const 
	{ 
		return FBox::BuildAABB(Center, Size * 0.5f); 
	}
    
	// Region volume for tactical reasoning
	float GetWorldVolume() const
	{
		return Size.X * Size.Y * Size.Z;
	}
    
	// Region radius estimate for containment checks
	float GetEstimatedRadius() const
	{
		// Use the largest dimension as diameter, half for radius
		const float MaxDimension = FMath::Max3(Size.X, Size.Y, Size.Z);
		return MaxDimension * 0.5f;
	}
    
	bool HasValidCenter() const { return !Center.IsZero(); }
	bool HasValidSize() const { return !Size.IsZero(); }
    
	friend FArchive& operator<<(FArchive& Ar, FCompactRegion& Region)
	{
		Ar << Region.LayerIndex;
		Ar << Region.Center;
		Ar << Region.Size;
		return Ar;
	}
};

/**
 * Sparse visibility matrix for cross-volume region references
 */
USTRUCT()
struct NAV3D_API FVolumeRegionMatrix
{
    GENERATED_BODY()

	// Key encoding: (TargetVolumeID << 6) | LocalRegionID
	// Value: 64-bit bitmask of referenced regions in target volume
	UPROPERTY()
	TMap<uint16, uint64> SparseReferences;

	// Helper function to encode key
	FORCEINLINE static uint16 EncodeKey(uint8 LocalRegionID, uint16 TargetVolumeID)
	{
		checkSlow(LocalRegionID < 64);
		checkSlow(TargetVolumeID < 1024);
		return (TargetVolumeID << 6) | LocalRegionID;
	}

	// Helper function to decode key
	FORCEINLINE static void DecodeKey(uint16 Key, uint8& OutLocalRegionID, uint16& OutTargetVolumeID)
	{
		OutLocalRegionID = Key & 0x3F;
		OutTargetVolumeID = Key >> 6;
	}
	
    /**
     * Sets a reference from a local region to a target region in another volume
     */
    void SetRegionReference(uint8 LocalRegionID, uint16 TargetVolumeID, uint8 TargetRegionID);

    /**
     * Checks if a local region references a specific target region
     */
    bool HasReference(uint8 LocalRegionID, uint16 TargetVolumeID, uint8 TargetRegionID) const;

    /**
     * Gets the complete region bitmask for a local region's references to a target volume
     */
    uint64 GetReferenceMask(uint8 LocalRegionID, uint16 TargetVolumeID) const;

    /**
     * Sets multiple region references at once using a bitmask
     */
    void SetReferenceMask(uint8 LocalRegionID, uint16 TargetVolumeID, uint64 RegionMask);

    /**
     * Clears all references for a specific local region
     */
    void ClearRegionReferences(uint8 LocalRegionID);

    /**
     * Gets all target volume IDs that a local region references
     */
    void GetReferencedVolumes(uint8 LocalRegionID, TArray<uint16>& OutVolumeIDs) const;
    
    /**
     * Counts the number of set bits in a 64-bit integer (population count)
     */
    static int32 CountBits(uint64 Value)
    {
        // Brian Kernighan's algorithm for counting set bits
        int32 Count = 0;
        while (Value)
        {
            Value &= (Value - 1); // Remove the rightmost set bit
            Count++;
        }
        return Count;
    }

    /**
     * Serialization support
     */
    friend FArchive& operator<<(FArchive& Ar, FVolumeRegionMatrix& Matrix)
    {
        Ar << Matrix.SparseReferences;
        return Ar;
    }
};

/**
 * Complete tactical data for a single volume
 * Replaces FLocalTacticalData with compact storage
 */
USTRUCT()
struct NAV3D_API FCompactTacticalData
{
    GENERATED_BODY()

    // Array of compact regions (max 64)
    UPROPERTY()
    TArray<FCompactRegion> Regions;

    // Sparse cross-volume visibility matrix
    UPROPERTY()
    FVolumeRegionMatrix VisibilityMatrix;

    // Intra-volume adjacency: Region ID -> bitmask of adjacent regions
    UPROPERTY()
    TMap<uint8, uint64> RegionAdjacency;

    // Volume ID for this tactical data
    UPROPERTY()
    uint16 VolumeID;

    FCompactTacticalData()
        : VolumeID(0)
    {}

    bool IsEmpty() const { return Regions.Num() == 0; }
    void Reset() 
    { 
        Regions.Empty(); 
        VisibilityMatrix = FVolumeRegionMatrix();
        RegionAdjacency.Empty(); 
        VolumeID = 0;
    }

    // Get region by index with bounds checking
    const FCompactRegion* GetRegion(uint8 RegionIndex) const
    {
        return Regions.IsValidIndex(RegionIndex) ? &Regions[RegionIndex] : nullptr;
    }

    // Check if two regions within this volume are adjacent
    bool AreRegionsAdjacent(uint8 RegionA, uint8 RegionB) const
    {
        if (const uint64* AdjMask = RegionAdjacency.Find(RegionA))
        {
            return (*AdjMask & (1ULL << RegionB)) != 0;
        }
        return false;
    }

    // Serialization support
    friend FArchive& operator<<(FArchive& Ar, FCompactTacticalData& Data)
    {
        Ar << Data.Regions;
        Ar << Data.VisibilityMatrix;
        Ar << Data.RegionAdjacency;
        Ar << Data.VolumeID;
        return Ar;
    }
};

/**
 * Runtime-consolidated tactical data using compact regions
 */
USTRUCT()
struct NAV3D_API FConsolidatedCompactTacticalData
{
    GENERATED_BODY()

    // All loaded regions from all chunks, indexed by global region ID
    UPROPERTY()
    TMap<uint16, FCompactRegion> AllLoadedRegions;

    // Global cross-volume visibility matrix
    UPROPERTY()
    TMap<uint16, FVolumeRegionMatrix> VolumeVisibilityData;

    // Global adjacency: includes both intra-volume and cross-volume connections
    UPROPERTY()
    TMap<uint16, uint64> GlobalRegionAdjacency;

    // Source chunks that contributed to this consolidated data
    TArray<TWeakObjectPtr<ANav3DDataChunkActor>> SourceChunks;

    void Reset()
    {
        AllLoadedRegions.Empty();
        VolumeVisibilityData.Empty();
        GlobalRegionAdjacency.Empty();
        SourceChunks.Empty();
    }

    int32 GetRegionCount() const { return AllLoadedRegions.Num(); }
    
    bool IsEmpty() const { return AllLoadedRegions.Num() == 0; }
    
    const FCompactRegion* GetRegion(uint16 GlobalRegionID) const
    {
        return AllLoadedRegions.Find(GlobalRegionID);
    }

    bool AreRegionsVisible(uint16 RegionA, uint16 VolumeA, uint16 RegionB, uint16 VolumeB) const
    {
        if (VolumeA == VolumeB)
        {
            // Same volume - check intra-volume adjacency or assume visible
            return true;
        }

        if (const FVolumeRegionMatrix* Matrix = VolumeVisibilityData.Find(VolumeA))
        {
            return Matrix->HasReference(RegionA, VolumeB, RegionB);
        }
        return false;
    }
};

/**
 * Serialization operator for FConsolidatedCompactTacticalData
 */
FORCEINLINE FArchive& operator<<(FArchive& Ar, FConsolidatedCompactTacticalData& Data)
{
    Ar << Data.AllLoadedRegions;
    Ar << Data.VolumeVisibilityData;
    Ar << Data.GlobalRegionAdjacency;
    return Ar;
}