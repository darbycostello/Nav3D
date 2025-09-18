#pragma once
#include "Engine/OverlapResult.h"
#include "LandscapeComponent.h"
#include "Nav3DTypes.h"
#include <Templates/SubclassOf.h>
#include "CoreMinimal.h"
#include "Templates/Atomic.h"

class UNavigationQueryFilter;
class UNav3DQueryFilter;
enum class ENav3DVersion : uint8;

// Forward declarations
struct FVoxelOverlapCache;

struct FNav3DVolumeNavigationDataSettings
{
	FNav3DVolumeNavigationDataSettings();

	float VoxelExtent;
	UWorld* World;
	FNav3DDataGenerationSettings GenerationSettings;
	FNav3DTacticalSettings TacticalSettings;
	FString DebugLabel;
	int32 DebugVolumeIndex = -1;
	// Optional cooperative cancellation flag provided by the generator
	TAtomic<bool>* CancelFlag = nullptr;
};

class NAV3D_API FNav3DVolumeNavigationData
{
public:
	// Core data
	TArray<FOverlapResult> OverlappingObjects;
	FIntVector VoxelDimensions;
	FNav3DVolumeNavigationDataSettings Settings;
	FBox VolumeBounds;
	FNav3DData Nav3DData;
	bool bInNavigationDataChunk;
	FNav3DTacticalData TacticalData;

	// Public methods
	const FBox& GetVolumeBounds() const { return VolumeBounds; }
	const FNav3DData& GetData() const { return Nav3DData; }
	FVector GetLeafNodePositionFromMortonCode(const MortonCode MortonCode) const;
	FVector GetNodePositionFromLayerAndMortonCode(const LayerIndex LayerIndex, const MortonCode MortonCode) const;
	FNav3DVolumeNavigationDataSettings& GetSettings() { return Settings; }
	const FNav3DVolumeNavigationDataSettings& GetSettings() const { return Settings; }
	void SetSettings(const FNav3DVolumeNavigationDataSettings& NewSettings) { Settings = NewSettings; }
	void SetVolumeBounds(const FBox& NewBounds) { VolumeBounds = NewBounds; }
	void SetData(const FNav3DData& NewData) { Nav3DData = NewData; }
	void SetInNavigationDataChunk(bool bInChunk) { bInNavigationDataChunk = bInChunk; }
	bool IsInNavigationDataChunk() const { return bInNavigationDataChunk; }
	void SetVoxelDimensions(const FIntVector& NewDimensions) { VoxelDimensions = NewDimensions; }
	const FIntVector& GetVoxelDimensions() const { return VoxelDimensions; }
	void SetOverlappingObjects(const TArray<FOverlapResult>& NewObjects) { OverlappingObjects = NewObjects; }
	const TArray<FOverlapResult>& GetOverlappingObjects() const { return OverlappingObjects; }
	void SetNumCandidateObjects(int32 NewCount) const { NumCandidateObjects = NewCount; }
	void SetNumOccludedVoxels(int32 NewCount) const { NumOccludedVoxels = NewCount; }
	int32 GetNumCandidateObjects() const { return NumCandidateObjects; }
	int32 GetNumOccludedVoxels() const { return NumOccludedVoxels; }
	void Serialize(FArchive& Archive, const ENav3DVersion Version);

	using FNodeRef = FNav3DNodeAddress;

	FNav3DVolumeNavigationData() = default;
	static bool IsValidRef(const FNav3DNodeAddress Ref) { return Ref.IsValid(); }
	const FNav3DVolumeNavigationDataSettings& GetDataGenerationSettings() const;
	const FBox& GetNavigationBounds() const;
	const FNav3DNode& GetNodeFromAddress(const FNav3DNodeAddress& Address) const;
	FVector GetNodePositionFromAddress(const FNav3DNodeAddress& Address, bool TryGetSubNodePosition) const;
	bool GetNodeAddressFromPosition(FNav3DNodeAddress& OutNodeAddress, const FVector& Position,
	                                const LayerIndex MinLayerIndex) const;
	bool FindNearestNavigableNode(const FVector& Position, FNav3DNodeAddress& OutNodeAddress,
	                              LayerIndex MinLayerIndex) const;
	void GetNodeNeighbours(TArray<FNav3DNodeAddress>& Neighbours, const FNav3DNodeAddress& NodeAddress) const;
	float GetLayerRatio(LayerIndex LayerIndex) const;
	float GetLayerInverseRatio(LayerIndex LayerIndex) const;
	float GetNodeExtentFromNodeAddress(FNav3DNodeAddress NodeAddress) const;

	TOptional<FNavLocation> GetRandomPoint() const;
	TArray<TWeakObjectPtr<const AActor>> DynamicOccluders;

	void GenerateNavigationData(const FBox& Bounds, const FNav3DVolumeNavigationDataSettings& GenerationSettings);
	void Reset();
	void RebuildDirtyBounds(const TArray<FBox>& DirtyBounds);
	void AddDynamicOccluder(const AActor* Occluder);
	void RemoveDynamicOccluder(const AActor* Occluder);

	// Global cooperative cancel control for all builds in flight
	static void RequestCancelBuildAll();
	static void ClearCancelBuildAll();
	static bool IsCancelRequested();

	static bool CheckStaticMeshOcclusion(const UStaticMeshComponent* StaticMeshComp, const FVector& Position,
	                                     const float BoxExtent);
	static bool CheckInstancedStaticMeshOcclusion(const UInstancedStaticMeshComponent* InstancedMeshComp,
	                                              const FVector& Position,
	                                              float BoxExtent);
	bool IsPositionOccluded(const FVector& Position, const float BoxExtent) const;
	LayerIndex GetMinLayerIndexForAgentSize(const float AgentRadius) const;
	int32 GetLayerCount() const { return Nav3DData.GetLayerCount(); }
	bool GetNodeAddressFromMortonCode(FNav3DNodeAddress& OutNodeAddress, MortonCode MortonCode,
	                                  LayerIndex LayerIndex) const;
	const TArray<NodeIndex>& GetLayerBlockedNodes(const LayerIndex LayerIndex) const;
	static MortonCode GetParentMortonCodeAtLayer(MortonCode ChildCode, LayerIndex TargetLayer, LayerIndex ChildLayer);
	float GetLayerNodeSize(LayerIndex LayerIndex) const;
	float GetLayerNodeExtent(LayerIndex LayerIndex) const;

private:
	void FirstPass();
	FString GetLogPrefix() const;
	void RasterizeLeaf(const FVector& NodePosition, const LeafIndex LeafIndex);
	void RasterizeInitialLayer(TMap<LeafIndex, MortonCode>& LeafIndexToLayerOneNodeIndexMap);
	void RasterizeLayer(LayerIndex LayerIndex);
	int32 GetNodeIndexFromMortonCode(LayerIndex LayerIndex, MortonCode MortonCode) const;
	void BuildNeighbourLinks(LayerIndex LayerIdx);
	bool FindNeighbourInDirection(FNav3DNodeAddress& NodeAddress, const LayerIndex LayerIndex,
	                              const NodeIndex NodeIndex, const NeighbourDirection Direction);
	void GetLeafNeighbours(TArray<FNav3DNodeAddress>& Neighbours, const FNav3DNodeAddress& LeafAddress) const;
	void GetFreeNodesFromNodeAddress(FNav3DNodeAddress NodeAddress, TArray<FNav3DNodeAddress>& FreeNodes) const;
	void BuildParentLinkForLeafNodes(const TMap<LeafIndex, MortonCode>& LeafIndexToParentMortonCodeMap);
	static bool CheckLandscapeProxyOcclusion(const ALandscapeProxy* LandscapeProxy, const FVector& Position,
	                                         const float BoxExtent);
	void LogNavigationStats() const;
	void GatherOverlappingObjects();
	static bool IsCollisionOnlyComponent(const UPrimitiveComponent* Component);
	static bool HasValidCollisionGeometry(const UStaticMeshComponent* StaticMeshComp);
	static bool HasValidCollisionGeometry(const UInstancedStaticMeshComponent* ISMComp);
	void RebuildLeafNodesInBounds(const FBox& DirtyBounds);
	static bool CheckStaticMeshTrianglesWithTransform(const UStaticMesh* StaticMesh, const FTransform& Transform,
	                                                  const FVector& Position, float BoxExtent);
	void PropagateChangesToHigherLayers(const TSet<MortonCode>& ModifiedLeafCodes, LayerIndex StartLayer);
	static bool IsNodeInBounds(const FVector& NodePosition, float NodeExtent, const FBox& Bounds);
	void CacheLayer1Overlaps();
	void ClearOverlapCache();
	bool IsPositionOccludedPhysics(const FVector& Position, float BoxExtent) const;
	mutable int32 NumCandidateObjects;
	mutable int32 NumOccludedVoxels;
	TMap<MortonCode, FVoxelOverlapCache> Layer1VoxelOverlapCache;

	// Incremental progress state
	mutable int32 LastLoggedCorePercent = -1;
	mutable double BuildStartTime = 0.0;
	mutable double LastProgressUpdateTime = 0.0;

	// Display progress normalized to core work (e.g., 20%-80% mapped to 0-100)
	void UpdateCoreProgress(const float Fraction0To1) const;

	// Global cancel flag shared by all build tasks
	static TAtomic<bool> bSCancelRequested;
};

FORCEINLINE const FNav3DVolumeNavigationDataSettings&
FNav3DVolumeNavigationData::GetDataGenerationSettings() const
{
	return Settings;
}

FORCEINLINE const FBox&
FNav3DVolumeNavigationData::GetNavigationBounds() const
{
	return Nav3DData.GetNavigationBounds();
}

FORCEINLINE const FNav3DNode& FNav3DVolumeNavigationData::GetNodeFromAddress(
	const FNav3DNodeAddress& Address) const
{
	static const FNav3DNode InvalidNode; // Static invalid node to return for bad addresses

	// Basic validation
	if (!Address.IsValid())
	{
		return InvalidNode;
	}

	// Validate layer index
	if (Address.LayerIndex >= Nav3DData.GetLayerCount())
	{
		return InvalidNode;
	}

	// For leaf nodes
	if (Address.LayerIndex == 0)
	{
		const auto& LeafNodes = Nav3DData.GetLeafNodes();
		if (!LeafNodes.GetLeafNodes().IsValidIndex(Address.NodeIndex))
		{
			return InvalidNode;
		}
	}
	// For other layers
	else
	{
		const auto& Layer = Nav3DData.GetLayer(Address.LayerIndex);
		if (!Layer.GetNodes().IsValidIndex(Address.NodeIndex))
		{
			return InvalidNode;
		}
		return Layer.GetNode(Address.NodeIndex);
	}

	return Nav3DData.GetLayer(Address.LayerIndex).GetNode(Address.NodeIndex);
}

inline void FNav3DVolumeNavigationData::RequestCancelBuildAll() { bSCancelRequested.Store(true); }
inline void FNav3DVolumeNavigationData::ClearCancelBuildAll() { bSCancelRequested.Store(false); }
inline bool FNav3DVolumeNavigationData::IsCancelRequested() { return bSCancelRequested.Load(); }
