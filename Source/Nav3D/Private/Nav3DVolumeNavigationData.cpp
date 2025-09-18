#include "Nav3DVolumeNavigationData.h"

#include <libmorton/morton.h>

#include "LandscapeProxy.h"
#include "Nav3DUtils.h"
#include "Nav3DTypes.h"
#include "TriBoxOverlap.h"
#include "HAL/CriticalSection.h"
#include "HAL/PlatformAtomics.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformMisc.h"
#include "Engine/OverlapResult.h"
#include "Engine/World.h"
#include "LandscapeMeshCollisionComponent.h"
#include "Nav3D.h"
#include "Components/BoxComponent.h"
#include "Components/CapsuleComponent.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Components/PrimitiveComponent.h"
#include "Components/SphereComponent.h"
#include "PhysicsEngine/BodySetup.h"

#if !UE_BUILD_SHIPPING
#include <DrawDebugHelpers.h>
#endif


FNav3DVolumeNavigationDataSettings::
FNav3DVolumeNavigationDataSettings()
	: VoxelExtent(0.0f), World(nullptr)
{
}

// Static cancel flag definition
TAtomic<bool> FNav3DVolumeNavigationData::bSCancelRequested{false};

FVector FNav3DVolumeNavigationData::GetNodePositionFromAddress(
	const FNav3DNodeAddress& Address,
	const bool TryGetSubNodePosition) const
{
	// Basic validation
	if (!Address.IsValid())
	{
		UE_LOG(LogNav3D, Verbose, TEXT("Invalid address"));
		return FVector::ZeroVector;
	}

	if (Address.LayerIndex >= Nav3DData.GetLayerCount())
	{
		UE_LOG(LogNav3D, Verbose, TEXT("Layer index %d out of bounds (max: %d)"),
		       Address.LayerIndex, Nav3DData.GetLayerCount());
		return FVector::ZeroVector;
	}

	const auto& Layer = Nav3DData.GetLayer(Address.LayerIndex);
	if (!Layer.GetNodes().IsValidIndex(Address.NodeIndex))
	{
		UE_LOG(LogNav3D, Verbose, TEXT("Node index %d out of bounds for layer %d (max: %d)"),
		       Address.NodeIndex, Address.LayerIndex, Layer.GetNodes().Num());
		return FVector::ZeroVector;
	}

	if (Address.LayerIndex == 0)
	{
		const auto& LeafNodes = Nav3DData.GetLeafNodes();
		if (!LeafNodes.GetLeafNodes().IsValidIndex(Address.NodeIndex))
		{
			UE_LOG(LogNav3D, Verbose, TEXT("Leaf node index %d out of bounds (max: %d)"),
			       Address.NodeIndex, LeafNodes.GetLeafNodes().Num());
			return FVector::ZeroVector;
		}

		const auto& LeafNode = LeafNodes.GetLeafNode(Address.NodeIndex);
		const auto& Node = Layer.GetNode(Address.NodeIndex);

		// Log the morton code we're using (disabled to reduce log spam)
		// UE_LOG(LogNav3D, VeryVerbose, TEXT("Using morton code %llu for leaf node %d"),
		//        Node.MortonCode, Address.NodeIndex);

		// Validate parent reference
		if (!LeafNode.Parent.IsValid())
		{
			UE_LOG(LogNav3D, VeryVerbose, TEXT("Invalid parent reference for leaf node %d"),
			       Address.NodeIndex);
			return FVector::ZeroVector;
		}

		if (LeafNode.Parent.LayerIndex >= Nav3DData.GetLayerCount())
		{
			UE_LOG(LogNav3D, VeryVerbose, TEXT("Parent layer index %d out of bounds for leaf node %d"),
			       LeafNode.Parent.LayerIndex, Address.NodeIndex);
			return FVector::ZeroVector;
		}

		if (!Nav3DData.GetLayer(LeafNode.Parent.LayerIndex).GetNodes().IsValidIndex(LeafNode.Parent.NodeIndex))
		{
			UE_LOG(LogNav3D, VeryVerbose, TEXT("Parent node index %d out of bounds for leaf node %d"),
			       LeafNode.Parent.NodeIndex, Address.NodeIndex);
			return FVector::ZeroVector;
		}

		const auto NodePosition = GetLeafNodePositionFromMortonCode(Node.MortonCode);

		if (LeafNode.IsCompletelyFree() || !TryGetSubNodePosition)
		{
			return NodePosition;
		}

		const auto SubNodeMortonCoords = FNav3DUtils::GetVectorFromMortonCode(Address.SubNodeIndex);
		const auto NodeExtent = LeafNodes.GetLeafNodeExtent();
		const auto SubNodeSize = LeafNodes.GetLeafSubNodeSize();

		return NodePosition - NodeExtent +
			SubNodeMortonCoords * SubNodeSize +
			LeafNodes.GetLeafSubNodeExtent();
	}

	const auto& Node = Layer.GetNode(Address.NodeIndex);
	return GetNodePositionFromLayerAndMortonCode(Address.LayerIndex, Node.MortonCode);
}

FVector FNav3DVolumeNavigationData::GetNodePositionFromLayerAndMortonCode(
	const LayerIndex LayerIndex, const MortonCode MortonCode) const
{
	if (LayerIndex == 0)
	{
		return GetLeafNodePositionFromMortonCode(MortonCode);
	}

	const auto& Layer = Nav3DData.GetLayer(LayerIndex);
	const auto LayerNodeExtent = Layer.GetNodeExtent();
	const auto& NavigationBounds = Nav3DData.GetNavigationBounds();
	const auto NavigationBoundsCenter = NavigationBounds.GetCenter();
	const auto NavigationBoundsExtent = NavigationBounds.GetExtent();
	const auto LayerNodeSize = Layer.GetNodeSize();
	const auto MortonCoords = FNav3DUtils::GetVectorFromMortonCode(MortonCode);

	return NavigationBoundsCenter - NavigationBoundsExtent +
		MortonCoords * LayerNodeSize + LayerNodeExtent;
}

FVector FNav3DVolumeNavigationData::GetLeafNodePositionFromMortonCode(const MortonCode MortonCode) const
{
	const auto& NavigationBounds = Nav3DData.GetNavigationBounds();
	const auto NavigationBoundsCenter = NavigationBounds.GetCenter();
	const auto NavigationBoundsExtent = NavigationBounds.GetExtent();
	const auto& LeafNodes = Nav3DData.GetLeafNodes();
	const auto LeafNodeExtent = LeafNodes.GetLeafNodeExtent();
	const auto LeafNodeSize = LeafNodes.GetLeafNodeSize();
	const auto MortonCoords = FNav3DUtils::GetVectorFromMortonCode(MortonCode);

	// Calculate each term separately for debugging
	const FVector OriginOffset = NavigationBoundsCenter - NavigationBoundsExtent;
	const FVector MortonOffset = MortonCoords * LeafNodeSize;
	const FVector ExtentOffset = FVector(LeafNodeExtent);
	const FVector LeafNodePosition = OriginOffset + MortonOffset + ExtentOffset;
	return LeafNodePosition;
}

bool FNav3DVolumeNavigationData::GetNodeAddressFromPosition(
    FNav3DNodeAddress& OutNodeAddress,
    const FVector& Position,
    const LayerIndex MinLayerIndex /*= 0*/) const
{
    QUICK_SCOPE_CYCLE_COUNTER(STAT_Nav3DBoundsNavigationData_GetNodeAddressFromPosition);

    const auto& NavigationBounds = Nav3DData.GetNavigationBounds();

    // Quick bounds test with small tolerance to absorb FP error
    const FBox ExpandedBounds = NavigationBounds.ExpandBy(1.0f);
    if (!ExpandedBounds.IsInside(Position))
    {
        UE_LOG(LogNav3D, VeryVerbose, TEXT("GetNodeAddressFromPosition: Position %s is outside navigation bounds %s"),
            *Position.ToString(), *NavigationBounds.ToString());
        return false;
    }

    const auto LayerCount = GetLayerCount();
    if (LayerCount == 0)
    {
        UE_LOG(LogNav3D, Error, TEXT("GetNodeAddressFromPosition: No layers present"));
        return false;
    }

    // Compute local position within navigation bounds
    FVector Origin, Extent;
    NavigationBounds.GetCenterAndExtents(Origin, Extent);
    const FVector LocalPosition = Position - (Origin - Extent); // Position relative to min corner

    // Work from coarsest layer down to MinLayerIndex
    for (LayerIndex CurrentLayer = static_cast<LayerIndex>(LayerCount - 1); CurrentLayer >= MinLayerIndex; --CurrentLayer)
    {
        // Calculate Morton code for this layer using existing utility functions
        const float LayerNodeSize = GetLayerNodeSize(CurrentLayer);
        const FIntVector LayerCoords = FIntVector(
            FMath::FloorToInt(LocalPosition.X / LayerNodeSize),
            FMath::FloorToInt(LocalPosition.Y / LayerNodeSize),
            FMath::FloorToInt(LocalPosition.Z / LayerNodeSize)
        );
        const MortonCode LayerMortonCode = FNav3DUtils::GetMortonCodeFromIntVector(LayerCoords);
        
        // Try to find an exact node at this layer
        const int32 NodeIdx = GetNodeIndexFromMortonCode(CurrentLayer, LayerMortonCode);
        if (NodeIdx == INDEX_NONE)
        {
            // No node here → that means pure free space at this layer
            UE_LOG(LogNav3D, VeryVerbose, TEXT("GetNodeAddressFromPosition: Free space at layer %d, morton %llu"), CurrentLayer, LayerMortonCode);
            OutNodeAddress.LayerIndex = CurrentLayer;
            OutNodeAddress.NodeIndex = INDEX_NONE;   // "free space" marker
            OutNodeAddress.SubNodeIndex = LayerMortonCode; // store Morton for positioning
            return true;
        }

        // Found a node
        const auto& Layer = Nav3DData.GetLayer(CurrentLayer);
        const auto& Node = Layer.GetNode(NodeIdx);

        if (!Node.HasChildren())
        {
            // This is navigable free space
            UE_LOG(LogNav3D, VeryVerbose, TEXT("GetNodeAddressFromPosition: Found navigable node at layer %d, index %d"), CurrentLayer, NodeIdx);
            OutNodeAddress.LayerIndex = CurrentLayer;
            OutNodeAddress.NodeIndex = NodeIdx;
            OutNodeAddress.SubNodeIndex = 0;
            return true;
        }

        if (CurrentLayer == 0)
        {
            // Leaf node: need to check sub-nodes
            const auto& LeafNodes = Nav3DData.GetLeafNodes();
            const auto& LeafNode = LeafNodes.GetLeafNode(NodeIdx);
            
            // Calculate sub-node index from position
            const FVector NodeWorldPos = GetLeafNodePositionFromMortonCode(Node.MortonCode);
            const auto NodeExtent = LeafNodes.GetLeafNodeExtent();
            const auto SubVoxelSize = LeafNodes.GetLeafSubNodeSize();
            const FVector NodeLocalPos = Position - (NodeWorldPos - FVector(NodeExtent));
            
            const int32 SubX = FMath::Clamp(FMath::FloorToInt(NodeLocalPos.X / SubVoxelSize), 0, 3);
            const int32 SubY = FMath::Clamp(FMath::FloorToInt(NodeLocalPos.Y / SubVoxelSize), 0, 3);
            const int32 SubZ = FMath::Clamp(FMath::FloorToInt(NodeLocalPos.Z / SubVoxelSize), 0, 3);
            const auto SubNodeIndex = FNav3DUtils::GetMortonCodeFromIntVector(FIntVector(SubX, SubY, SubZ));

            if (!LeafNode.IsSubNodeOccluded(SubNodeIndex))
            {
                // Exact free sub-node
                UE_LOG(LogNav3D, VeryVerbose, TEXT("GetNodeAddressFromPosition: Found free sub-node at leaf %d, sub-node %llu"), NodeIdx, SubNodeIndex);
                OutNodeAddress.LayerIndex = 0;
                OutNodeAddress.NodeIndex = NodeIdx;
                OutNodeAddress.SubNodeIndex = SubNodeIndex;
                return true;
            }

            // Try nearest free sub-node in this leaf
            const FVector SubExtent = FVector(LeafNodes.GetLeafSubNodeExtent());
            const FVector LeafOrigin = NodeWorldPos - FVector(NodeExtent);
            
            float BestDistSq = TNumericLimits<float>::Max();
        	MortonCode BestSubNodeIndex = 0;
            bool bFoundFreeSubNode = false;

            for (int32 SubIdx = 0; SubIdx < 64; ++SubIdx)
            {
                if (!LeafNode.IsSubNodeOccluded(SubIdx))
                {
                    const FIntVector SubCoords = FNav3DUtils::GetIntVectorFromMortonCode(SubIdx);
                    const FVector SubCenter = LeafOrigin + FVector(SubCoords) * SubVoxelSize + SubExtent;
                    const float DistSq = FVector::DistSquared(SubCenter, Position);
                    if (DistSq < BestDistSq)
                    {
                        BestDistSq = DistSq;
                        BestSubNodeIndex = SubIdx;
                        bFoundFreeSubNode = true;
                    }
                }
            }

            if (bFoundFreeSubNode)
            {
                UE_LOG(LogNav3D, VeryVerbose, TEXT("GetNodeAddressFromPosition: Found nearest free sub-node at leaf %d, sub-node %llu"), NodeIdx, BestSubNodeIndex);
                OutNodeAddress.LayerIndex = 0;
                OutNodeAddress.NodeIndex = NodeIdx;
                OutNodeAddress.SubNodeIndex = BestSubNodeIndex;
                return true;
            }

            // Fully blocked leaf - fall through to global search
            UE_LOG(LogNav3D, VeryVerbose, TEXT("GetNodeAddressFromPosition: Leaf node %d fully blocked"), NodeIdx);
            break;
        }

        // Otherwise keep descending — children might contain navigable space
        // Continue to next iteration with CurrentLayer decremented
    }

    // Fallback: try to find nearest free node globally
    UE_LOG(LogNav3D, VeryVerbose, TEXT("GetNodeAddressFromPosition: Falling back to nearest navigable node search"));
    return FindNearestNavigableNode(Position, OutNodeAddress, MinLayerIndex);
}

bool FNav3DVolumeNavigationData::FindNearestNavigableNode(
    const FVector& Position, 
    FNav3DNodeAddress& OutNodeAddress,
    const LayerIndex MinLayerIndex /*= 0*/) const
{
    float BestDistSq = TNumericLimits<float>::Max();
    FNav3DNodeAddress BestAddress;
    bool bFoundAny = false;

    const LayerIndex LayerCount = GetLayerCount();
    
    // Search from MinLayerIndex up to top layer
    for (LayerIndex LayerIdx = MinLayerIndex; LayerIdx < LayerCount; ++LayerIdx)
    {
        const auto& Layer = Nav3DData.GetLayer(LayerIdx);
        const auto& LayerNodes = Layer.GetNodes();
        
        for (int32 NodeIdx = 0; NodeIdx < LayerNodes.Num(); ++NodeIdx)
        {
            const auto& Node = LayerNodes[NodeIdx];
            
            if (LayerIdx == 0)
            {
                // Check leaf sub-nodes
                const auto& LeafNodes = Nav3DData.GetLeafNodes();
                const auto& LeafNode = LeafNodes.GetLeafNode(NodeIdx);
            	
                for (int32 SubIdx = 0; SubIdx < 64; ++SubIdx)
                {
                    if (!LeafNode.IsSubNodeOccluded(SubIdx))
                    {
                        const FNav3DNodeAddress TestAddr = FNav3DNodeAddress(0, NodeIdx, SubIdx);
                        const FVector SubPos = GetNodePositionFromAddress(TestAddr, true);
                        const float DistSq = FVector::DistSquared(SubPos, Position);
                        
                        if (DistSq < BestDistSq)
                        {
                            BestDistSq = DistSq;
                            BestAddress = TestAddr;
                            bFoundAny = true;
                        }
                    }
                }
            }
            else if (!Node.HasChildren())
            {
                // Non-leaf navigable node
                const FVector NodePos = GetNodePositionFromLayerAndMortonCode(LayerIdx, Node.MortonCode);
                const float DistSq = FVector::DistSquared(NodePos, Position);
                
                if (DistSq < BestDistSq)
                {
                    BestDistSq = DistSq;
                    BestAddress = FNav3DNodeAddress(LayerIdx, NodeIdx, 0);
                    bFoundAny = true;
                }
            }
        }
    }

    if (bFoundAny)
    {
        OutNodeAddress = BestAddress;
        UE_LOG(LogNav3D, VeryVerbose, TEXT("FindNearestNavigableNode: Found node at layer %d, index %d, subnode %d"), 
               BestAddress.LayerIndex, BestAddress.NodeIndex, BestAddress.SubNodeIndex);
        return true;
    }

    UE_LOG(LogNav3D, Warning, TEXT("FindNearestNavigableNode: No navigable nodes found"));
    return false;
}

void FNav3DVolumeNavigationData::GetNodeNeighbours(
	TArray<FNav3DNodeAddress>& Neighbours,
	const FNav3DNodeAddress& NodeAddress) const
{
	QUICK_SCOPE_CYCLE_COUNTER(STAT_Nav3DBoundsNavigationData_GetNeighbours);

	const auto& Node = GetNodeFromAddress(NodeAddress);
	if (NodeAddress.LayerIndex == 0 && Node.FirstChild.IsValid())
	{
		GetLeafNeighbours(Neighbours, NodeAddress);
		return;
	}

	for (NeighbourDirection Direction = 0; Direction < 6; Direction++)
	{
		const auto& NeighbourAddress = Node.Neighbours[Direction];

		if (!NeighbourAddress.IsValid())
		{
			continue;
		}

		const auto& Neighbour = GetNodeFromAddress(NeighbourAddress);

		if (!Neighbour.HasChildren())
		{
			Neighbours.Add(NeighbourAddress);
			continue;
		}

		TArray<FNav3DNodeAddress> NeighbourAddressesWorkingSet;
		NeighbourAddressesWorkingSet.Push(NeighbourAddress);

		while (NeighbourAddressesWorkingSet.Num() > 0)
		{
			// Pop off the top of the working set
			auto ThisAddress = NeighbourAddressesWorkingSet.Pop();

			const auto& ThisNode = GetNodeFromAddress(ThisAddress);

			// If the node as no children, it's clear, so add to Neighbours and
			// continue
			if (!ThisNode.HasChildren())
			{
				Neighbours.Add(NeighbourAddress);
				continue;
			}

			if (ThisAddress.LayerIndex > 0)
			{
				/* Morton code node ordering
				    Z
				    ^
				    |          5 --- 7
				    |        / |   / |
				    |       4 --- 6  |
				    |  X    |  1 -|- 3
				    | /     | /   | /
				    |/      0 --- 2
				    +-------------------> Y
				*/

				static constexpr NodeIndex ChildOffsetsDirections[6][4] = {
					{0, 4, 2, 6}, {1, 3, 5, 7}, {0, 1, 4, 5},
					{2, 3, 6, 7}, {0, 1, 2, 3}, {4, 5, 6, 7}
				};

				for (const auto& ChildIndex : ChildOffsetsDirections[Direction])
				{
					auto FirstChildAddress = ThisNode.FirstChild;
					FirstChildAddress.NodeIndex += ChildIndex;

					if (const auto& ChildNode = GetNodeFromAddress(FirstChildAddress);
						ChildNode.HasChildren())
					// working set to keep going down
					{
						NeighbourAddressesWorkingSet.Emplace(FirstChildAddress);
					}
					else
					{
						Neighbours.Emplace(FirstChildAddress);
					}
				}
			}
			else
			{
				/*
				Sub node morton code ordering for the face pointing to Neighbour[0],
				which is (1,0,0) Use the debug draw options of the navigation data in
				the scene to show all the sub nodes
		
				Z
				|
				|   36 38 52 54
				|   32 34 48 50
				|   04 06 20 22
				|   00 02 16 18
				|
				------------------ Y
				*/

				static constexpr NodeIndex LeafChildOffsetsDirections[6][16] = {
					{0, 2, 16, 18, 4, 6, 20, 22, 32, 34, 48, 50, 36, 38, 52, 54},
					{9, 11, 25, 27, 13, 15, 29, 31, 41, 43, 57, 59, 45, 47, 61, 63},
					{0, 1, 8, 9, 4, 5, 12, 13, 32, 33, 40, 41, 36, 37, 44, 45},
					{18, 19, 26, 27, 22, 23, 30, 31, 50, 51, 58, 59, 54, 55, 62, 63},
					{0, 1, 8, 9, 2, 3, 10, 11, 16, 17, 24, 25, 18, 19, 26, 27},
					{36, 37, 44, 45, 38, 39, 46, 47, 52, 53, 60, 61, 54, 55, 62, 63}
				};

				for (const auto& LeafIndex : LeafChildOffsetsDirections[Direction])
				{
					auto FirstChildAddress = Neighbour.FirstChild;
					const auto& LeafNode =
						Nav3DData.GetLeafNodes().GetLeafNode(FirstChildAddress.NodeIndex);

					FirstChildAddress.LayerIndex = 0;
					FirstChildAddress.NodeIndex = ThisAddress.NodeIndex;
					FirstChildAddress.SubNodeIndex = LeafIndex;

					if (!LeafNode.IsSubNodeOccluded(LeafIndex))
					{
						Neighbours.Emplace(FirstChildAddress);
					}
				}
			}
		}
	}
}

float FNav3DVolumeNavigationData::GetLayerRatio(
	const LayerIndex LayerIndex) const
{
	return static_cast<float>(LayerIndex) / GetLayerCount();
}

float FNav3DVolumeNavigationData::GetLayerInverseRatio(
	const LayerIndex LayerIndex) const
{
	return 1.0f - GetLayerRatio(LayerIndex);
}

float FNav3DVolumeNavigationData::GetNodeExtentFromNodeAddress(
	const FNav3DNodeAddress NodeAddress) const
{
	// Validate layer index first
	if (NodeAddress.LayerIndex >= Nav3DData.GetLayerCount())
	{
		return 0.0f;
	}

	if (NodeAddress.LayerIndex == 0)
	{
		const auto& LeafNodes = Nav3DData.GetLeafNodes();

		// Validate leaf node index
		if (!LeafNodes.GetLeafNodes().IsValidIndex(NodeAddress.NodeIndex))
		{
			return 0.0f;
		}

		const auto& LeafNode = LeafNodes.GetLeafNode(NodeAddress.NodeIndex);
		if (LeafNode.IsCompletelyFree())
		{
			return LeafNodes.GetLeafNodeExtent();
		}

		return LeafNodes.GetLeafSubNodeExtent();
	}

	// Validate node index for non-leaf layers
	const auto& Layer = Nav3DData.GetLayer(NodeAddress.LayerIndex);
	if (!Layer.GetNodes().IsValidIndex(NodeAddress.NodeIndex))
	{
		return 0.0f;
	}

	return Layer.GetNodeExtent();
}

TOptional<FNavLocation> FNav3DVolumeNavigationData::GetRandomPoint() const
{
	TArray<FNav3DNodeAddress> NonOccludedNodes;
	const FNav3DNodeAddress TopMostNodeAddress(GetLayerCount(), 0, 0);

	GetFreeNodesFromNodeAddress(TopMostNodeAddress, NonOccludedNodes);

	if (NonOccludedNodes.Num() == 0)
	{
		return TOptional<FNavLocation>();
	}

	const auto RandomIndex = FMath::RandRange(0, NonOccludedNodes.Num() - 1);
	const auto RandomNode = NonOccludedNodes[RandomIndex];
	const auto RandomNodeLocation = GetNodePositionFromAddress(RandomNode, true);
	const auto RandomNodeExtent = GetNodeExtentFromNodeAddress(RandomNode);

	const auto NodeBounds =
		FBox::BuildAABB(RandomNodeLocation, FVector(RandomNodeExtent));
	const auto RandomPointInNode = FMath::RandPointInBox(NodeBounds);
	return FNavLocation(RandomPointInNode, RandomNode.GetNavNodeRef());
}

void FNav3DVolumeNavigationData::GenerateNavigationData(
    const FBox& Bounds,
    const FNav3DVolumeNavigationDataSettings& GenerationSettings)
{
    QUICK_SCOPE_CYCLE_COUNTER(STAT_Nav3DBoundsNavigationData_GenerateNavigationData);

    Settings = GenerationSettings;
    if (IsCancelRequested()) { return; }
    VolumeBounds = Bounds;
    NumCandidateObjects = 0;
    NumOccludedVoxels = 0;

    const auto VoxelExtent = Settings.VoxelExtent;

    if (!Nav3DData.Initialize(VoxelExtent, VolumeBounds))
    {
        return;
    }
    if (IsCancelRequested()) { return; }
    
    GatherOverlappingObjects();
    if (IsCancelRequested()) { return; }

	// Early-out: if the volume has no overlapping objects and no dynamic occluders,
	// we can skip the entire rasterization. This makes empty volumes essentially free.
	if (OverlappingObjects.Num() == 0 && DynamicOccluders.Num() == 0)
	{
		UE_LOG(LogNav3D, Log, TEXT("GenerateNavigationData: No overlaps in volume; skipping rasterization"));
		Nav3DData.bIsValid = true;
		LogNavigationStats();
		UpdateCoreProgress(1.0f);
		return;
	}

    // Reset progress tracking
    LastLoggedCorePercent = -1;

    const auto LayerCount = Nav3DData.GetLayerCount();

    FirstPass();
    if (IsCancelRequested()) { return; }

	UpdateCoreProgress(0.01f);

    {
        QUICK_SCOPE_CYCLE_COUNTER(STAT_Nav3DBoundsNavigationData_AllocateLeafNodes);
        const auto LeafCount = Nav3DData.GetLayerBlockedNodes(0).Num() * 8;
        Nav3DData.GetLeafNodes().AllocateLeafNodes(LeafCount);
    }
    if (IsCancelRequested()) { return; }

    TMap<LeafIndex, MortonCode> LeafIndexToParentMortonCodeMap;
    RasterizeInitialLayer(LeafIndexToParentMortonCodeMap);
    if (IsCancelRequested()) { return; }

    for (LayerIndex LayerIndex = 1; LayerIndex < LayerCount; ++LayerIndex)
    {
        if (IsCancelRequested()) { return; }
        RasterizeLayer(LayerIndex);
    }

    BuildParentLinkForLeafNodes(LeafIndexToParentMortonCodeMap);
    if (IsCancelRequested()) { return; }

    for (LayerIndex LayerIdx = LayerCount - 2; LayerIdx != static_cast<LayerIndex>(-1); --LayerIdx)
    {
        if (IsCancelRequested()) { return; }
        BuildNeighbourLinks(LayerIdx);
    }

    Nav3DData.bIsValid = true;
    
    LogNavigationStats();

    UpdateCoreProgress(1.0f);
}

static FString FormatElapsedTime(const double ElapsedSeconds)
{
	const int32 TotalSeconds = static_cast<int32>(ElapsedSeconds);
	const int32 Minutes = TotalSeconds / 60;
	const int32 Seconds = TotalSeconds % 60;
	
	if (Minutes > 0)
	{
		if (Seconds > 0)
		{
			return FString::Printf(TEXT("%d %s, %d %s"), 
				Minutes, Minutes == 1 ? TEXT("min") : TEXT("mins"),
				Seconds, Seconds == 1 ? TEXT("sec") : TEXT("secs"));
		}
		else
		{
			return FString::Printf(TEXT("%d %s"), 
				Minutes, Minutes == 1 ? TEXT("min") : TEXT("mins"));
		}
	}
	else
	{
		return FString::Printf(TEXT("%d %s"), 
			Seconds, Seconds == 1 ? TEXT("sec") : TEXT("secs"));
	}
}

void FNav3DVolumeNavigationData::UpdateCoreProgress(const float Fraction0To1) const
{
	const float ClampedFrac = FMath::Clamp(Fraction0To1, 0.0f, 1.0f);
	const int32 CoreRounded = FMath::RoundToInt(ClampedFrac * 100.0f);
	if (CoreRounded <= LastLoggedCorePercent)
	{
		return;
	}
	
	const double CurrentTime = FPlatformTime::Seconds();
	
	// Initialize timing on first progress update
	if (LastLoggedCorePercent == -1)
	{
		BuildStartTime = CurrentTime;
		LastProgressUpdateTime = CurrentTime;
	}
	
	// Calculate elapsed time since last update
	const double ElapsedSinceLastUpdate = CurrentTime - LastProgressUpdateTime;
	LastProgressUpdateTime = CurrentTime;
	LastLoggedCorePercent = CoreRounded;
	
	// Log progress with elapsed time (only show time for progress > 0%)
	const FString Prefix = GetLogPrefix();
	if (CoreRounded > 0)
	{
		UE_LOG(LogNav3D, Log, TEXT("%sNav3D build core progress: %d%% (%s)"), 
			*Prefix, CoreRounded, *FormatElapsedTime(ElapsedSinceLastUpdate));
	}
	else
	{
		UE_LOG(LogNav3D, Log, TEXT("%sNav3D build core progress: %d%%"), *Prefix, CoreRounded);
	}
	
	// Log total build time when complete
	if (CoreRounded >= 100)
	{
		const double TotalBuildTime = CurrentTime - BuildStartTime;
		UE_LOG(LogNav3D, Log, TEXT("%sNav3D build completed in %s"), *Prefix, *FormatElapsedTime(TotalBuildTime));
	}
}

FString FNav3DVolumeNavigationData::GetLogPrefix() const
{
	if (Settings.DebugVolumeIndex >= 0)
	{
		return FString::Printf(TEXT("[Vol#%d %s] "), Settings.DebugVolumeIndex, *Settings.DebugLabel);
	}
	return FString();
}

void FNav3DVolumeNavigationData::Serialize(FArchive& Archive, const ENav3DVersion Version)
{
	// Initialize size tracking
	auto N3DSizeBytes = 0;
	const auto N3DSizePosition = Archive.Tell();
	Archive << N3DSizeBytes;

	// Core data serialization
	Archive << VolumeBounds;
	Archive << Nav3DData;
	Archive << bInNavigationDataChunk;
	Archive << TacticalData;

	// Loading-specific validity restoration
	if (Archive.IsLoading())
	{
		// Mark loaded nav data as valid if it contains layers
		Nav3DData.bIsValid = (Nav3DData.GetLayerCount() > 0);
	}

	// Saving-specific size finalization
	if (Archive.IsSaving())
	{
		const auto CurrentPosition = Archive.Tell();
		N3DSizeBytes = CurrentPosition - N3DSizePosition;
		Archive.Seek(N3DSizePosition);
		Archive << N3DSizeBytes;
		Archive.Seek(CurrentPosition);
	}
}

void FNav3DVolumeNavigationData::Reset()
{
	VolumeBounds.Init();
	Nav3DData.Reset();
	
	// Clear optimization cache
	ClearOverlapCache();
}

void FNav3DVolumeNavigationData::GatherOverlappingObjects()
{
    OverlappingObjects.Reset();
    Settings.World->OverlapMultiByChannel(
        OverlappingObjects, VolumeBounds.GetCenter(), FQuat::Identity,
        Settings.GenerationSettings.CollisionChannel,
        FCollisionShape::MakeBox(VolumeBounds.GetExtent()),
        Settings.GenerationSettings.CollisionQueryParameters);

    const int32 InitialCount = OverlappingObjects.Num();

    // Aggressive filtering: only keep objects with actual collision geometry for navigation
    OverlappingObjects.RemoveAllSwap([this](const FOverlapResult& Result)
    {
        // First, safely validate the component
        if (!Result.Component.IsValid())
        {
            UE_LOG(LogNav3D, VeryVerbose, TEXT("Removing invalid component from overlap results"));
            return true; // Remove
        }

        UPrimitiveComponent* PrimComponent = Result.Component.Get();
        if (!PrimComponent || !IsValid(PrimComponent))
        {
            UE_LOG(LogNav3D, VeryVerbose, TEXT("Removing null/invalid component from overlap results"));
            return true; // Remove
        }

        // Basic navigation check
        if (!PrimComponent->CanEverAffectNavigation())
        {
            return true; // Remove
        }

        // Filter out collision-only components (like your PCG spheres)
        if (IsCollisionOnlyComponent(PrimComponent))
        {
            UE_LOG(LogNav3D, VeryVerbose, TEXT("Removing collision-only component: %s"), 
                   *PrimComponent->GetName());
            return true; // Remove
        }

        // For static mesh components, check if they have collision geometry
        if (const UStaticMeshComponent* StaticMeshComp = Cast<UStaticMeshComponent>(PrimComponent))
        {
            return !HasValidCollisionGeometry(StaticMeshComp);
        }

        // For ISMs, check the static mesh collision
        if (const UInstancedStaticMeshComponent* ISMComp = Cast<UInstancedStaticMeshComponent>(PrimComponent))
        {
            return !HasValidCollisionGeometry(ISMComp);
        }

        // Keep landscape and other navigation-relevant types
        if (PrimComponent->IsA<ULandscapeHeightfieldCollisionComponent>() ||
            PrimComponent->IsA<ULandscapeMeshCollisionComponent>())
        {
            return false; // Keep
        }

        // Keep other components that can affect navigation
        return false;
    });

    NumCandidateObjects = OverlappingObjects.Num();
    
    UE_LOG(LogNav3D, Log, TEXT("Navigation filtering: %d -> %d objects (%.1f%% reduction)"), 
           InitialCount, NumCandidateObjects, 
           InitialCount > 0 ? (100.0f * (InitialCount - NumCandidateObjects) / InitialCount) : 0.0f);
}

// Helper to identify collision-only components (like your PCG sphere colliders)
bool FNav3DVolumeNavigationData::IsCollisionOnlyComponent(const UPrimitiveComponent* Component)
{
    if (!Component)
    {
        return false;
    }

    // Check for sphere collision components (common in PCG setups)
    if (Component->IsA<USphereComponent>())
    {
        return true;
    }

    // Check for box collision components
    if (Component->IsA<UBoxComponent>())
    {
        return true;
    }

    // Check for capsule collision components
    if (Component->IsA<UCapsuleComponent>())
    {
        return true;
    }

    // You can add more collision-only component types here as needed
    
    return false;
}

// From the parallel conversation - check for actual collision geometry
bool FNav3DVolumeNavigationData::HasValidCollisionGeometry(const UStaticMeshComponent* StaticMeshComp)
{
    if (!StaticMeshComp || !StaticMeshComp->GetStaticMesh())
    {
        return false;
    }

    // Check collision settings
    if (StaticMeshComp->GetCollisionEnabled() == ECollisionEnabled::NoCollision)
    {
        return false;
    }

    const UStaticMesh* StaticMesh = StaticMeshComp->GetStaticMesh();
    const UBodySetup* BodySetup = StaticMesh->GetBodySetup();
    
    if (!BodySetup)
    {
        return false;
    }

    // Check if it has any collision geometry
    const FKAggregateGeom& AggGeom = BodySetup->AggGeom;
    return (AggGeom.ConvexElems.Num() > 0 || 
            AggGeom.BoxElems.Num() > 0 || 
            AggGeom.SphereElems.Num() > 0 || 
            AggGeom.SphylElems.Num() > 0 ||
            AggGeom.TaperedCapsuleElems.Num() > 0);
}

// Overload for ISMs
bool FNav3DVolumeNavigationData::HasValidCollisionGeometry(const UInstancedStaticMeshComponent* ISMComp)
{
    if (!ISMComp || !ISMComp->GetStaticMesh())
    {
        return false;
    }

    // Check collision settings
    if (ISMComp->GetCollisionEnabled() == ECollisionEnabled::NoCollision)
    {
        return false;
    }

    // Check if it has instances
    if (ISMComp->GetInstanceCount() == 0)
    {
        return false;
    }

    // Check the static mesh collision
    const UStaticMesh* StaticMesh = ISMComp->GetStaticMesh();
    const UBodySetup* BodySetup = StaticMesh->GetBodySetup();
    
    if (!BodySetup)
    {
        return false;
    }

    const FKAggregateGeom& AggGeom = BodySetup->AggGeom;
    return (AggGeom.ConvexElems.Num() > 0 || 
            AggGeom.BoxElems.Num() > 0 || 
            AggGeom.SphereElems.Num() > 0 || 
            AggGeom.SphylElems.Num() > 0 ||
            AggGeom.TaperedCapsuleElems.Num() > 0);
}

bool FNav3DVolumeNavigationData::IsPositionOccluded(const FVector& Position, const float BoxExtent) const
{
	QUICK_SCOPE_CYCLE_COUNTER(STAT_Nav3DBoundsNavigationData_IsPositionOccluded);

	if (IsCancelRequested())
	{
		return false;
	}

	// If we have a Layer 1 overlap cache, use it to early-out or to perform
	// a faster check limited to cached actors.
	if (Layer1VoxelOverlapCache.Num() > 0)
	{
		const auto& NavigationBounds = Nav3DData.GetNavigationBounds();
		const FVector Origin = NavigationBounds.GetCenter() - NavigationBounds.GetExtent();
		const FVector LocalPosition = Position - Origin;
		const float L1VoxelSize = Nav3DData.GetLayer(1).GetNodeSize();
		FIntVector VoxelCoords;
		VoxelCoords.X = FMath::FloorToInt(LocalPosition.X / L1VoxelSize);
		VoxelCoords.Y = FMath::FloorToInt(LocalPosition.Y / L1VoxelSize);
		VoxelCoords.Z = FMath::FloorToInt(LocalPosition.Z / L1VoxelSize);
		const MortonCode L1Code = FNav3DUtils::GetMortonCodeFromIntVector(VoxelCoords);

		if (const FVoxelOverlapCache* CacheEntry = Layer1VoxelOverlapCache.Find(L1Code))
		{
			if (CacheEntry->OverlappingActors.Num() == 0)
			{
				return false; // Parent L1 is empty; cannot be occluded
			}
			// Optimized, inlined path: test only against cached actors/components
			const FBox PositionBoxCached = FBox::BuildAABB(Position, FVector(BoxExtent + Settings.GenerationSettings.Clearance));
			for (const TWeakObjectPtr<AActor>& ActorWeak : CacheEntry->OverlappingActors)
			{
				if (IsCancelRequested()) { return false; }
				const AActor* Actor = ActorWeak.Get();
				if (!Actor || !IsValid(Actor)) { continue; }

				const FBox ActorBounds = Actor->GetComponentsBoundingBox(true);
				if (!ActorBounds.Intersect(PositionBoxCached)) { continue; }

				if (const ALandscapeProxy* LandscapeProxy = Cast<ALandscapeProxy>(Actor))
				{
					if (CheckLandscapeProxyOcclusion(LandscapeProxy, Position, BoxExtent))
					{
						if (FMath::IsNearlyEqual(BoxExtent, Nav3DData.GetLeafNodes().GetLeafSubNodeExtent()))
						{
							FPlatformAtomics::InterlockedIncrement(&NumOccludedVoxels);
						}
						return true;
					}
				}

				TInlineComponentArray<UInstancedStaticMeshComponent*> ISMComponents;
				Actor->GetComponents<UInstancedStaticMeshComponent>(ISMComponents);
				for (const UInstancedStaticMeshComponent* ISMComp : ISMComponents)
				{
					if (!ISMComp) { continue; }
					if (CheckInstancedStaticMeshOcclusion(ISMComp, Position, BoxExtent))
					{
						if (FMath::IsNearlyEqual(BoxExtent, Nav3DData.GetLeafNodes().GetLeafSubNodeExtent()))
						{
							FPlatformAtomics::InterlockedIncrement(&NumOccludedVoxels);
						}
						return true;
					}
				}

				TInlineComponentArray<UStaticMeshComponent*> StaticMeshComponents;
				Actor->GetComponents<UStaticMeshComponent>(StaticMeshComponents);
				for (const UStaticMeshComponent* SMC : StaticMeshComponents)
				{
					if (!SMC || SMC->IsA<UInstancedStaticMeshComponent>()) { continue; }
					if (CheckStaticMeshOcclusion(SMC, Position, BoxExtent))
					{
						if (FMath::IsNearlyEqual(BoxExtent, Nav3DData.GetLeafNodes().GetLeafSubNodeExtent()))
						{
							FPlatformAtomics::InterlockedIncrement(&NumOccludedVoxels);
						}
						return true;
					}
				}
			}
			return false;
		}
	}

	const FBox PositionBox = FBox::BuildAABB(Position, FVector(BoxExtent + Settings.GenerationSettings.Clearance));

	// Check dynamic occluders first since they might have moved
	for (const TWeakObjectPtr<const AActor>& OccluderWeak : DynamicOccluders)
	{
		if (IsCancelRequested())
		{
			return false;
		}
		const AActor* Occluder = OccluderWeak.Get();
		if (!Occluder || !IsValid(Occluder))
		{
			continue;
		}

		// First quick AABB test
		const FBox OccluderBounds = Occluder->GetComponentsBoundingBox(true);
		if (!OccluderBounds.Intersect(PositionBox))
		{
			continue;
		}

		if (const UStaticMeshComponent* StaticMeshComp = Occluder->FindComponentByClass<UStaticMeshComponent>())
		{
			if (CheckStaticMeshOcclusion(StaticMeshComp, Position, BoxExtent))
			{
				if (FMath::IsNearlyEqual(BoxExtent, Nav3DData.GetLeafNodes().GetLeafSubNodeExtent()))
				{
					FPlatformAtomics::InterlockedIncrement(&NumOccludedVoxels);
				}
				return true;
			}
		}
	}

	// Then check static geometry
	for (int32 i = 0; i < OverlappingObjects.Num(); ++i)
	{
		if (IsCancelRequested())
		{
			return false;
		}
	    const FOverlapResult& OverlapResult = OverlappingObjects[i];
	    
	    // Validate the OverlapResult structure itself
	    if (!OverlapResult.Component.IsValid())
	    {
	        UE_LOG(LogNav3D, Warning, TEXT("Skipping overlap result %d: Invalid component"), i);
	        continue;
	    }
	    
	    UPrimitiveComponent* PrimComponent = OverlapResult.Component.Get();
	    if (!PrimComponent || !IsValid(PrimComponent))
	    {
	        UE_LOG(LogNav3D, Warning, TEXT("Skipping overlap result %d: Component failed validation"), i);
	        continue;
	    }
	    
	    if (!PrimComponent->CanEverAffectNavigation())
	    {
	        UE_LOG(LogNav3D, Warning, TEXT("Skipping overlap result %d: Component can't affect navigation"), i);
	        continue;
	    }

	    const FBox ObjectBounds = PrimComponent->Bounds.GetBox();
	    if (!ObjectBounds.Intersect(PositionBox))
	    {
	        continue;
	    }

	    bool bIsOccluded = false;

	    // ULTRA-DEFENSIVE: Get actor through component owner instead of OverlapResult.GetActor()
	    const AActor* Actor = PrimComponent->GetOwner();
	    if (Actor && IsValid(Actor))
	    {
	        // Validate the actor's class before any operations
	        if (!Actor->GetClass())
	        {
	            UE_LOG(LogNav3D, Warning, TEXT("Skipping actor with null class: %s"), *Actor->GetName());
	            continue;
	        }
	        
	        UE_LOG(LogNav3D, VeryVerbose, TEXT("Checking actor: %s (Class: %s)"), 
	               *Actor->GetName(), 
	               *Actor->GetClass()->GetName());
	               
	        // Safe landscape check
	        if (Actor->GetClass()->IsChildOf<ALandscapeProxy>())
	        {
	            if (const ALandscapeProxy* LandscapeProxy = Cast<ALandscapeProxy>(Actor))
	            {
	                bIsOccluded = CheckLandscapeProxyOcclusion(LandscapeProxy, Position, BoxExtent);
	            }
	        }
	    }

	    // Component check
	    if (!bIsOccluded)
	    {
	        // Validate component class before any operations
	        if (!PrimComponent->GetClass())
	        {
	            UE_LOG(LogNav3D, Warning, TEXT("Skipping component with null class: %s"), *PrimComponent->GetName());
	            continue;
	        }
	        
	        UE_LOG(LogNav3D, VeryVerbose, TEXT("Checking component: %s (Class: %s)"), 
	               *PrimComponent->GetName(), 
	               *PrimComponent->GetClass()->GetName());
	               
	        // Safe static mesh check
	        if (PrimComponent->GetClass()->IsChildOf<UStaticMeshComponent>())
	        {
	        	if (const UInstancedStaticMeshComponent* InstancedMeshComp = Cast<UInstancedStaticMeshComponent>(PrimComponent))
	        	{
	        		bIsOccluded = CheckInstancedStaticMeshOcclusion(InstancedMeshComp, Position, BoxExtent);
	        	}
	            else if (const UStaticMeshComponent* StaticMeshComp = Cast<UStaticMeshComponent>(PrimComponent))
	            {
	                bIsOccluded = CheckStaticMeshOcclusion(StaticMeshComp, Position, BoxExtent);
	            }
	        }
	    }

	    if (bIsOccluded)
	    {
	        if (FMath::IsNearlyEqual(BoxExtent, Nav3DData.GetLeafNodes().GetLeafNodeExtent()))
	        {
	            FPlatformAtomics::InterlockedIncrement(&NumOccludedVoxels);
	            UE_LOG(LogNav3D, VeryVerbose, TEXT("Layer 0 voxel occluded at %s"), *Position.ToString());
	        }
	        return true;
	    }
	}

	return false;
}

LayerIndex FNav3DVolumeNavigationData::GetMinLayerIndexForAgentSize(const float AgentRadius) const
{
	if (!GetData().IsValid())
	{
		return 0;
	}
	const float AgentDiameter = AgentRadius * 2.0f;

	// Find the smallest layer that can fit this agent
	for (LayerIndex i = 0; i < GetData().GetLayerCount(); i++)
	{
		const float LayerVoxelSize = GetData().GetLayer(i).GetNodeSize();
		if (FMath::IsNearlyZero(LayerVoxelSize))
		{
			UE_LOG(LogNav3D, Warning, TEXT("Layer %d has zero voxel size"), i);
			continue;
		}
		if (LayerVoxelSize >= AgentDiameter)
		{
			return i;
		}
	}

	return GetData().GetLayerCount() - 1;
}

void FNav3DVolumeNavigationData::RebuildLeafNodesInBounds(const FBox& DirtyBounds)
{
	UE_LOG(LogNav3D, Verbose, TEXT("RebuildLeafNodesInBounds: DirtyBounds=%s, DynamicOccluders=%d"),
	       *DirtyBounds.ToString(), DynamicOccluders.Num());

	// First, collect all existing nodes that intersect with dirty bounds
	TArray<int32> NodesToRemove;
	auto& LayerZero = Nav3DData.GetLayer(0);

	// First pass - mark nodes for removal if they're no longer occluded
	for (int32 NodeIdx = 0; NodeIdx < LayerZero.GetNodes().Num(); NodeIdx++)
	{
		const auto& Node = LayerZero.GetNodes()[NodeIdx];
		const FVector NodePos = GetLeafNodePositionFromMortonCode(Node.MortonCode);
		const float NodeExtent = Nav3DData.GetLeafNodes().GetLeafNodeExtent();

		if (IsNodeInBounds(NodePos, NodeExtent, DirtyBounds))
		{
			if (!IsPositionOccluded(NodePos, NodeExtent))
			{
				NodesToRemove.Add(NodeIdx);
			}
		}
	}

	// Remove nodes from the highest index to lowest to maintain valid indices
	NodesToRemove.Sort([](const int32 A, const int32 B) { return A > B; });
	TSet<MortonCode> RemovedCodes;
	for (int32 NodeIdx : NodesToRemove)
	{
		// Store the morton code for later layer cleanup
		RemovedCodes.Add(LayerZero.GetNodes()[NodeIdx].MortonCode);

		// Remove the leaf node first
		if (LayerZero.GetNodes()[NodeIdx].HasChildren())
		{
			const auto LeafIndex = LayerZero.GetNodes()[NodeIdx].FirstChild.NodeIndex;
			auto& LeafNode = Nav3DData.GetLeafNodes().GetLeafNode(LeafIndex);
			LeafNode.SubNodes = 0;
		}

		// Remove the node itself
		LayerZero.GetNodes().RemoveAtSwap(NodeIdx, 1, EAllowShrinking::No);
	}

	// Calculate bounds for new node iteration
	const FVector BoundsCenter = Nav3DData.GetNavigationBounds().GetCenter();
	const FVector BoundsExtent = Nav3DData.GetNavigationBounds().GetExtent();
	const float LeafNodeSize = Nav3DData.GetLeafNodes().GetLeafNodeSize();

	FVector MinCorner = DirtyBounds.Min - BoundsCenter + BoundsExtent;
	FVector MaxCorner = DirtyBounds.Max - BoundsCenter + BoundsExtent;

	FIntVector MinCoords(
		FMath::Floor(MinCorner.X / LeafNodeSize),
		FMath::Floor(MinCorner.Y / LeafNodeSize),
		FMath::Floor(MinCorner.Z / LeafNodeSize)
	);

	FIntVector MaxCoords(
		FMath::CeilToInt(MaxCorner.X / LeafNodeSize),
		FMath::CeilToInt(MaxCorner.Y / LeafNodeSize),
		FMath::CeilToInt(MaxCorner.Z / LeafNodeSize)
	);

	TSet<MortonCode> ModifiedLeafCodes;
	int32 CheckedNodes = 0;
	int32 ModifiedNodes = 0;

	// Create or update nodes
	for (int32 Z = MinCoords.Z; Z <= MaxCoords.Z; Z++)
	{
		for (int32 Y = MinCoords.Y; Y <= MaxCoords.Y; Y++)
		{
			for (int32 X = MinCoords.X; X <= MaxCoords.X; X++)
			{
				CheckedNodes++;

				const MortonCode NodeMortonCode = FNav3DUtils::GetMortonCodeFromVector(FVector(X, Y, Z));
				int32 NodeIndex = GetNodeIndexFromMortonCode(0, NodeMortonCode);

				const FVector NodePos = GetLeafNodePositionFromMortonCode(NodeMortonCode);
				const float NodeExtent = Nav3DData.GetLeafNodes().GetLeafNodeExtent();

				bool bShouldBeOccluded = IsPositionOccluded(NodePos, NodeExtent);

				if (NodeIndex == INDEX_NONE && bShouldBeOccluded)
				{
					// Only create new nodes if they should be occluded
					FNav3DNode NewNode;
					NewNode.MortonCode = NodeMortonCode;
					NewNode.FirstChild.LayerIndex = 0;
					NewNode.FirstChild.NodeIndex = Nav3DData.GetLeafNodes().GetLeafNodes().Num();

					NodeIndex = LayerZero.GetNodes().Add(NewNode);
					Nav3DData.GetLeafNodes().AddEmptyLeafNode();
					ModifiedLeafCodes.Add(NodeMortonCode);
					ModifiedNodes++;

					RasterizeLeaf(NodePos, NewNode.FirstChild.NodeIndex);
				}
				else if (NodeIndex != INDEX_NONE)
				{
					auto& Node = LayerZero.GetNodes()[NodeIndex];
					if (bShouldBeOccluded)
					{
						if (!Node.HasChildren())
						{
							Node.FirstChild.LayerIndex = 0;
							Node.FirstChild.NodeIndex = Nav3DData.GetLeafNodes().GetLeafNodes().Num();
							Nav3DData.GetLeafNodes().AddEmptyLeafNode();
						}
						RasterizeLeaf(NodePos, Node.FirstChild.NodeIndex);
						ModifiedLeafCodes.Add(NodeMortonCode);
						ModifiedNodes++;
					}
				}
			}
		}
	}

	UE_LOG(LogNav3D, Verbose, TEXT("Checked %d leaf nodes, Modified %d nodes, Removed %d nodes"),
	       CheckedNodes, ModifiedNodes, NodesToRemove.Num());

	if (ModifiedLeafCodes.Num() > 0 || RemovedCodes.Num() > 0)
	{
		// Add removed codes to modified codes so their parents get cleaned up
		ModifiedLeafCodes.Append(RemovedCodes);
		PropagateChangesToHigherLayers(ModifiedLeafCodes, 1);
	}
}

bool FNav3DVolumeNavigationData::CheckStaticMeshTrianglesWithTransform(
    const UStaticMesh* StaticMesh,
    const FTransform& Transform,
    const FVector& Position,
    const float BoxExtent)
{
    if (!StaticMesh || !StaticMesh->GetRenderData() || StaticMesh->GetRenderData()->LODResources.Num() == 0)
    {
        return false;
    }

    // Quick bounds check first
    const FBox MeshBounds = StaticMesh->GetBounds().GetBox();
    const FBox WorldMeshBounds = MeshBounds.TransformBy(Transform);
    const FBox VoxelBounds = FBox::BuildAABB(Position, FVector(BoxExtent));
    
    if (!WorldMeshBounds.Intersect(VoxelBounds))
    {
        return false; // Early exit - no intersection possible
    }

    // Get mesh data
    const FPositionVertexBuffer* VertexBuffer = 
        &StaticMesh->GetRenderData()->LODResources[0].VertexBuffers.PositionVertexBuffer;
    const FRawStaticIndexBuffer* IndexBuffer = 
        &StaticMesh->GetRenderData()->LODResources[0].IndexBuffer;

    // Check triangles
    for (int32 i = 0; i < IndexBuffer->GetNumIndices(); i += 3)
    {
        if (IsCancelRequested())
        {
            return false;
        }
        const FVector3f V0F = VertexBuffer->VertexPosition(IndexBuffer->GetIndex(i));
        const FVector3f V1F = VertexBuffer->VertexPosition(IndexBuffer->GetIndex(i + 1));
        const FVector3f V2F = VertexBuffer->VertexPosition(IndexBuffer->GetIndex(i + 2));

        // Transform vertices
        FVector V0 = Transform.TransformPosition(FVector(V0F));
        FVector V1 = Transform.TransformPosition(FVector(V1F));
        FVector V2 = Transform.TransformPosition(FVector(V2F));

        if (TriBoxOverlap(Position, FVector(BoxExtent), V0, V1, V2))
        {
            return true;
        }
    }

    return false;
}

bool FNav3DVolumeNavigationData::CheckStaticMeshOcclusion(
    const UStaticMeshComponent* StaticMeshComp,
    const FVector& Position,
    const float BoxExtent)
{
    if (const UStaticMesh* StaticMesh = StaticMeshComp->GetStaticMesh())
    {
        return CheckStaticMeshTrianglesWithTransform(
            StaticMesh, 
            StaticMeshComp->GetComponentTransform(), 
            Position, 
            BoxExtent
        );
    }
    return false;
}

bool FNav3DVolumeNavigationData::CheckInstancedStaticMeshOcclusion(
    const UInstancedStaticMeshComponent* InstancedMeshComp,
    const FVector& Position,
    const float BoxExtent)
{
    if (!InstancedMeshComp || !InstancedMeshComp->GetStaticMesh())
    {
        return false;
    }

    const int32 InstanceCount = InstancedMeshComp->GetInstanceCount();
    if (InstanceCount == 0)
    {
        return false;
    }

    const UStaticMesh* StaticMesh = InstancedMeshComp->GetStaticMesh();

    // Check each instance
    for (int32 InstanceIndex = 0; InstanceIndex < InstanceCount; InstanceIndex++)
    {
        if (IsCancelRequested())
        {
            return false;
        }
        FTransform InstanceTransform;
        if (!InstancedMeshComp->GetInstanceTransform(InstanceIndex, InstanceTransform, /*bWorldSpace=*/true))
        {
            continue;
        }

        if (CheckStaticMeshTrianglesWithTransform(StaticMesh, InstanceTransform, Position, BoxExtent))
        {
            return true;
        }
    }

    return false;
}

bool FNav3DVolumeNavigationData::CheckLandscapeProxyOcclusion(
	const ALandscapeProxy* LandscapeProxy,
	const FVector& Position,
	const float BoxExtent)
{
	if (!LandscapeProxy)
	{
		return false;
	}

	const float FloorHeight = LandscapeProxy->GetActorLocation().Z;

	// If the voxel is entirely below floor level, it's not occluded
	if (Position.Z + BoxExtent <= FloorHeight)
	{
		return false;
	}

	// If the bottom of the voxel is at or below floor height, it's occluded
	if (Position.Z - BoxExtent <= FloorHeight)
	{
		return true;
	}

	// For voxels above floor level, check against landscape height
	const FVector LocalPosition = LandscapeProxy->LandscapeActorToWorld().InverseTransformPosition(Position);
	const FIntPoint Key = FIntPoint(
		FMath::FloorToInt(LocalPosition.X / LandscapeProxy->ComponentSizeQuads),
		FMath::FloorToInt(LocalPosition.Y / LandscapeProxy->ComponentSizeQuads)
	);
	const ULandscapeInfo* LandscapeInfo = LandscapeProxy->GetLandscapeInfo();
	if (!LandscapeInfo)
	{
		return false;
	}

	ULandscapeHeightfieldCollisionComponent* CollisionComponent = LandscapeInfo->XYtoCollisionComponentMap.FindRef(Key);
	if (!CollisionComponent)
	{
		return false;
	}
	const FVector ComponentLocalPosition = CollisionComponent->GetComponentTransform().
	                                                           InverseTransformPosition(Position);
	const TOptional<float> Height = CollisionComponent->GetHeight(ComponentLocalPosition.X, ComponentLocalPosition.Y,
	                                                              EHeightfieldSource::Complex);

	if (!Height.IsSet())
	{
		return false;
	}
	const float WorldHeight = CollisionComponent->GetComponentTransform().TransformPosition(
		FVector(0, 0, Height.GetValue())).Z;

	// If the bottom of the voxel is at or below the landscape height, it's occluded
	if (Position.Z - BoxExtent <= WorldHeight)
	{
		return true;
	}

	return false;
}

void FNav3DVolumeNavigationData::LogNavigationStats() const
{
	UE_LOG(LogNav3D, Log, TEXT("Navigation Data Generation Complete"));
	UE_LOG(LogNav3D, Log, TEXT("Number of Candidate Objects: %d"), NumCandidateObjects);
	UE_LOG(LogNav3D, Log, TEXT("Number of Occluded Voxels: %d"), NumOccludedVoxels);

	const int32 CachedVoxels = Layer1VoxelOverlapCache.Num();
	const int32 VoxelsWithOverlaps = Layer1VoxelOverlapCache.FilterByPredicate([](const auto& Pair) { 
		return Pair.Value.OverlappingActors.Num() > 0; 
	}).Num();
		
	UE_LOG(LogNav3D, Log, TEXT("Optimized First Pass: %d cached voxels, %d with overlaps (%.1f%% reduction)"), 
		CachedVoxels, VoxelsWithOverlaps, 
		(1.0f - static_cast<float>(VoxelsWithOverlaps) / FMath::Max(1, CachedVoxels)) * 100.0f);

	const FVector VolumeSize = VolumeBounds.GetSize();
	UE_LOG(LogNav3D, Log, TEXT("Volume Size: X=%.2f, Y=%.2f, Z=%.2f"), VolumeSize.X, VolumeSize.Y, VolumeSize.Z);
	UE_LOG(LogNav3D, Log, TEXT("Voxel Extent: %.2f"), Settings.VoxelExtent);
}

void FNav3DVolumeNavigationData::FirstPass()
{
	QUICK_SCOPE_CYCLE_COUNTER(STAT_Nav3DBoundsNavigationData_FirstPassRasterization);

	const double StartTime = FPlatformTime::Seconds();
	
	// Step 1: Cache all Layer 1 voxel overlaps using physics queries
	CacheLayer1Overlaps();
	
	// Step 2: Process Layer 1 nodes using cached data
	const auto& Layer1 = Nav3DData.GetLayer(1);
	const auto LayerMaxNodeCount = Layer1.GetMaxNodeCount();
	const auto LayerNodeExtent = Layer1.GetNodeExtent();
	
	UE_LOG(LogNav3D, Log, TEXT("FirstPassOptimized: Processing %d Layer 1 nodes using cached overlaps"), LayerMaxNodeCount);
	
	int32 ProcessedNodes = 0;
	const int32 TotalNodes = LayerMaxNodeCount;
	
	// Single-threaded processing since we're using cached data
	for (uint32 NodeIndex = 0; NodeIndex < LayerMaxNodeCount; NodeIndex++)
	{
		if (IsCancelRequested()) 
		{
			ClearOverlapCache();
			return;
		}
		
		const auto Position = GetNodePositionFromLayerAndMortonCode(1, NodeIndex);
		
		// Check if this Layer 1 voxel has any overlapping actors
		const FVoxelOverlapCache* CacheEntry = Layer1VoxelOverlapCache.Find(NodeIndex);
		if (CacheEntry && CacheEntry->OverlappingActors.Num() > 0)
		{
			// Use consolidated occlusion check (consults cache internally)
			if (IsPositionOccluded(Position, LayerNodeExtent))
			{
				Nav3DData.AddBlockedNode(0, NodeIndex);
			}
		}
		
		ProcessedNodes++;
		const float Fraction = static_cast<float>(ProcessedNodes) / FMath::Max(1, TotalNodes);
		UpdateCoreProgress(Fraction * 0.05f);
	}
	
	const double EndTime = FPlatformTime::Seconds();
	const double Duration = EndTime - StartTime;
	UE_LOG(LogNav3D, Log, TEXT("%sFirstPassOptimized: Complete (%s)"), *GetLogPrefix(), *FormatElapsedTime(Duration));

	// Common continuation for higher layers
	for (int32 LayerIndex = 1; LayerIndex < GetLayerCount(); LayerIndex++)
	{
		const auto& ParentLayerBlockedNodes =
			Nav3DData.GetLayerBlockedNodes(LayerIndex - 1);
		for (const MortonCode MortonCode : ParentLayerBlockedNodes)
		{
			Nav3DData.AddBlockedNode(
				LayerIndex, FNav3DUtils::GetParentMortonCode(MortonCode));
		}
	}

	UpdateCoreProgress(0.2f);

	UE_LOG(LogNav3D, Log, TEXT("FirstPass: Complete"));
}

void FNav3DVolumeNavigationData::CacheLayer1Overlaps()
{
	const double StartTime = FPlatformTime::Seconds();
	
	const auto& Layer1 = Nav3DData.GetLayer(1);
	const auto LayerMaxNodeCount = Layer1.GetMaxNodeCount();
	const auto LayerNodeExtent = Layer1.GetNodeExtent();
	
	UE_LOG(LogNav3D, Log, TEXT("%sCacheLayer1Overlaps: Preparing overlaps for %d L1 voxels"), *GetLogPrefix(), LayerMaxNodeCount);
	
	// Clear any existing cache
	Layer1VoxelOverlapCache.Empty(LayerMaxNodeCount);
	
	UWorld* World = Settings.World;
	if (!World)
	{
		UE_LOG(LogNav3D, Error, TEXT("CacheLayer1Overlaps: No valid world found"));
		return;
	}
	
	// Pre-allocate cache entries to avoid race conditions
	for (uint32 NodeIndex = 0; NodeIndex < LayerMaxNodeCount; NodeIndex++)
	{
		const FVector Position = GetNodePositionFromLayerAndMortonCode(1, NodeIndex);
		const FVector BoxExtent(LayerNodeExtent + Settings.GenerationSettings.Clearance);
		const FBox VoxelBox = FBox::BuildAABB(Position, BoxExtent);
		Layer1VoxelOverlapCache.Add(NodeIndex, FVoxelOverlapCache(NodeIndex, VoxelBox));
	}
	
	FCriticalSection CriticalSection;
	int32 ProcessedVoxels = 0;

	UE_LOG(LogNav3D, Log, TEXT("%sCacheLayer1Overlaps: Using sequential overlap queries"), *GetLogPrefix());
	for (uint32 NodeIndex = 0; NodeIndex < LayerMaxNodeCount; ++NodeIndex)
	{
		if (IsCancelRequested())
		{
			break;
		}
		const FVector Position = GetNodePositionFromLayerAndMortonCode(1, NodeIndex);
		const FVector BoxExtent(LayerNodeExtent + Settings.GenerationSettings.Clearance);
		
		TArray<FOverlapResult> OverlapResults;
		FCollisionQueryParams QueryParams = Settings.GenerationSettings.CollisionQueryParameters;
		QueryParams.bTraceComplex = false;
		const bool bHasOverlaps = World->OverlapMultiByChannel(
			OverlapResults,
			Position,
			FQuat::Identity,
			Settings.GenerationSettings.CollisionChannel,
			FCollisionShape::MakeBox(BoxExtent),
			QueryParams
		);
		if (bHasOverlaps)
		{
			TArray<TWeakObjectPtr<AActor>> OverlappingActors;
			for (const FOverlapResult& Result : OverlapResults)
			{
				if (AActor* Actor = Result.GetActor())
				{
					if (UPrimitiveComponent* PrimComponent = Result.Component.Get())
					{
						const bool bRelevant =
							Cast<ULandscapeComponent>(PrimComponent) != nullptr ||
							Cast<UInstancedStaticMeshComponent>(PrimComponent) != nullptr ||
							Cast<UStaticMeshComponent>(PrimComponent) != nullptr ||
							PrimComponent->CanEverAffectNavigation();
						if (bRelevant)
						{
							OverlappingActors.AddUnique(Actor);
						}
					}
				}
			}
			if (FVoxelOverlapCache* CacheEntry = Layer1VoxelOverlapCache.Find(NodeIndex))
			{
				CacheEntry->OverlappingActors = MoveTemp(OverlappingActors);
			}
		}
		ProcessedVoxels++;
		if (ProcessedVoxels % 1000 == 0)
		{
			const float Progress = static_cast<float>(ProcessedVoxels) / LayerMaxNodeCount;
			UE_LOG(LogNav3D, Log, TEXT("%sCacheLayer1Overlaps: %d/%d (%.1f%%)"), *GetLogPrefix(), ProcessedVoxels, LayerMaxNodeCount, Progress * 100.0f);
		}
	}
	
	const double EndTime = FPlatformTime::Seconds();
	const double Duration = EndTime - StartTime;
	
	const int32 WithOverlaps = Layer1VoxelOverlapCache.FilterByPredicate([](const auto& Pair) { return Pair.Value.OverlappingActors.Num() > 0; }).Num();
	UE_LOG(LogNav3D, Log, TEXT("%sCacheLayer1Overlaps: Complete (%s). Cached %d, with overlaps %d (%.1f%%)"), 
		*GetLogPrefix(), *FormatElapsedTime(Duration),
		Layer1VoxelOverlapCache.Num(), WithOverlaps, LayerMaxNodeCount > 0 ? (100.0f * WithOverlaps / LayerMaxNodeCount) : 0.0f);
}

bool FNav3DVolumeNavigationData::IsPositionOccludedPhysics(const FVector& Position, float BoxExtent) const
{
	// Use physics overlap query instead of tri-box testing for much better performance
	UWorld* World = Settings.World;
	if (!World)
	{
		return false;
	}
	
	const FVector BoxExtentVector(BoxExtent + Settings.GenerationSettings.Clearance);
	
	// Perform physics box overlap query
	TArray<FOverlapResult> OverlapResults;
	FCollisionQueryParams QueryParams = Settings.GenerationSettings.CollisionQueryParameters;
	QueryParams.bTraceComplex = false; // Use simple collision for faster queries
	
	bool bHasOverlaps = World->OverlapMultiByChannel(
		OverlapResults,
		Position,
		FQuat::Identity,
		Settings.GenerationSettings.CollisionChannel,
		FCollisionShape::MakeBox(BoxExtentVector),
		QueryParams
	);
	
	if (!bHasOverlaps)
	{
		return false; // No overlaps means no occlusion
	}
	
	// Check if any overlapping objects actually occlude the position
	for (const FOverlapResult& Result : OverlapResults)
	{
		if (IsCancelRequested())
		{
			return false;
		}
		
		UPrimitiveComponent* PrimComponent = Result.Component.Get();
		if (!PrimComponent || !IsValid(PrimComponent))
		{
			continue;
		}
		
		if (!PrimComponent->CanEverAffectNavigation())
		{
			continue;
		}
		
		// Quick bounds check first
		const FBox ObjectBounds = PrimComponent->Bounds.GetBox();
		const FBox PositionBox = FBox::BuildAABB(Position, BoxExtentVector);
		if (!ObjectBounds.Intersect(PositionBox))
		{
			continue;
		}
		
		// Detailed occlusion check using existing methods
		bool bIsOccluded = false;
		
		if (const UStaticMeshComponent* StaticMeshComp = Cast<UStaticMeshComponent>(PrimComponent))
		{
			if (const UInstancedStaticMeshComponent* InstancedMeshComp = Cast<UInstancedStaticMeshComponent>(StaticMeshComp))
			{
				bIsOccluded = CheckInstancedStaticMeshOcclusion(InstancedMeshComp, Position, BoxExtent);
			}
			else
			{
				bIsOccluded = CheckStaticMeshOcclusion(StaticMeshComp, Position, BoxExtent);
			}
		}
		else if (const ALandscapeProxy* LandscapeProxy = Cast<ALandscapeProxy>(PrimComponent->GetOwner()))
		{
			bIsOccluded = CheckLandscapeProxyOcclusion(LandscapeProxy, Position, BoxExtent);
		}
		
		if (bIsOccluded)
		{
			if (FMath::IsNearlyEqual(BoxExtent, Nav3DData.GetLeafNodes().GetLeafSubNodeExtent()))
			{
				FPlatformAtomics::InterlockedIncrement(&NumOccludedVoxels);
			}
			return true;
		}
	}
	
	return false;
}

void FNav3DVolumeNavigationData::ClearOverlapCache()
{
	Layer1VoxelOverlapCache.Empty();
	UE_LOG(LogNav3D, VeryVerbose, TEXT("Layer 1 overlap cache cleared"));
}

void FNav3DVolumeNavigationData::RasterizeLeaf(const FVector& NodePosition,
                                               const LeafIndex LeafIndex)
{
	QUICK_SCOPE_CYCLE_COUNTER(STAT_Nav3DBoundsNavigationData_RasterizeLeaf);

	const auto LeafNodeExtent = Nav3DData.GetLeafNodes().GetLeafNodeExtent();
	const auto LeafSubNodeSize = Nav3DData.GetLeafNodes().GetLeafSubNodeSize();
	const auto LeafSubNodeExtent = Nav3DData.GetLeafNodes().GetLeafSubNodeExtent();
	const auto Location = NodePosition - LeafNodeExtent;

	// Process sub-nodes
	for (SubNodeIndex SubNodeIndex = 0; SubNodeIndex < 64; SubNodeIndex++)
	{
		const auto MortonCoords = FNav3DUtils::GetVectorFromMortonCode(SubNodeIndex);
		const auto LeafNodeLocation = Location + MortonCoords * LeafSubNodeSize + LeafSubNodeExtent;
		const bool bIsSubNodeOccluded = IsPositionOccludedPhysics(LeafNodeLocation, LeafSubNodeExtent);
		Nav3DData.GetLeafNodes().AddLeafNode(LeafIndex, SubNodeIndex, bIsSubNodeOccluded);
	}
}

void FNav3DVolumeNavigationData::RasterizeInitialLayer(
	TMap<LeafIndex, MortonCode>& LeafIndexToLayerOneNodeIndexMap)
{
	UE_LOG(LogNav3D, Log, TEXT("Rasterize initial layer"));
	
	auto& LayerZero = Nav3DData.GetLayer(0);
	const auto& LayerZeroBlockedNodes = Nav3DData.GetLayerBlockedNodes(0);

	// Prepare a temporary array to hold the results of parallel processing
	TArray<TPair<NodeIndex, FNav3DNode>> TempNodes;
	TempNodes.Reserve(LayerZeroBlockedNodes.Num() * 8);

	// Iterate only children of blocked Layer 1 parents; avoid global scan and locks
	for (const MortonCode ParentMortonCode : LayerZeroBlockedNodes)
	{
		if (IsCancelRequested()) { break; }
		const MortonCode FirstChildCode = FNav3DUtils::GetFirstChildMortonCode(ParentMortonCode);
		for (int32 ChildIdx = 0; ChildIdx < 8; ++ChildIdx)
		{
			const MortonCode LeafMortonCode = FirstChildCode + ChildIdx;

			FNav3DNode LayerZeroNode;
			LayerZeroNode.MortonCode = LeafMortonCode;

			const auto LeafNodePosition = GetLeafNodePositionFromMortonCode(LayerZeroNode.MortonCode);
			const auto LeafNodeExtent = Nav3DData.GetLeafNodes().GetLeafNodeExtent();

			// Use consolidated occlusion (consults L1 cache)
			if (IsPositionOccluded(LeafNodePosition, LeafNodeExtent))
			{
				LayerZeroNode.FirstChild.LayerIndex = 0;
				LayerZeroNode.FirstChild.NodeIndex = INDEX_NONE;
				LayerZeroNode.FirstChild.SubNodeIndex = 0;
			}
			else
			{
				LayerZeroNode.FirstChild.Invalidate();
			}

			TempNodes.Emplace(LeafMortonCode, LayerZeroNode);
		}
	}

	// Sort nodes
	TempNodes.Sort([](const TPair<NodeIndex, FNav3DNode>& A, const TPair<NodeIndex, FNav3DNode>& B)
	{
		return A.Key < B.Key;
	});

	auto& LayerZeroNodes = LayerZero.GetNodes();
	LayerZeroNodes.Reserve(TempNodes.Num());

	LeafIndex LeafIdx = 0;
	for (const auto& Pair : TempNodes)
	{
		const auto& Node = Pair.Value;
		LayerZeroNodes.Add(Node);

		// Always add to parent map, even for invalid children
		LeafIndexToLayerOneNodeIndexMap.Add(LeafIdx, FNav3DUtils::GetParentMortonCode(Node.MortonCode));

		if (Node.FirstChild.IsValid())
		{
			LayerZeroNodes.Last().FirstChild.NodeIndex = LeafIdx;
			RasterizeLeaf(GetLeafNodePositionFromMortonCode(Node.MortonCode), LeafIdx);
		}
		else
		{
			Nav3DData.GetLeafNodes().AddEmptyLeafNode();
		}
		LeafIdx++;

		const float Fraction = static_cast<float>(LeafIdx) / FMath::Max(1, TempNodes.Num());
		UpdateCoreProgress(Fraction);
	}

	UpdateCoreProgress(0.8f);
}

void FNav3DVolumeNavigationData::RasterizeLayer(const LayerIndex LayerIndex)
{
	QUICK_SCOPE_CYCLE_COUNTER(STAT_Nav3DBoundsNavigationData_RasterizeLayer);

	UE_LOG(LogNav3D, Log, TEXT("Rasterize layer %d"), LayerIndex);

	auto& Layer = Nav3DData.GetLayer(LayerIndex);
	auto& LayerNodes = Layer.GetNodes();
	const auto& LayerBlockedNodes = Nav3DData.GetLayerBlockedNodes(LayerIndex);

	checkf(LayerIndex > 0 && LayerIndex < GetLayerCount(), TEXT("LayerIdx is out of bounds"));

	LayerNodes.Reserve(LayerBlockedNodes.Num() * 8);

	const auto LayerMaxNodeCount = Layer.GetMaxNodeCount();

	// Sequential loop avoids lock contention and scans
	for (int32 NodeIdx = 0; NodeIdx < static_cast<int32>(LayerMaxNodeCount); ++NodeIdx)
	{
		if (IsCancelRequested()) { break; }
		const bool bIsBlocked = LayerBlockedNodes.Contains(FNav3DUtils::GetParentMortonCode(NodeIdx));
		if (!bIsBlocked) { continue; }

		FNav3DNode LayerNode;
		LayerNode.MortonCode = NodeIdx;

		const auto ChildLayerIndex = LayerIndex - 1;
		const auto FirstChildMortonCode = FNav3DUtils::GetFirstChildMortonCode(LayerNode.MortonCode);
		const auto ChildIndexFromCode = GetNodeIndexFromMortonCode(ChildLayerIndex, FirstChildMortonCode);

		auto& FirstChild = LayerNode.FirstChild;
		if (ChildIndexFromCode != INDEX_NONE)
		{
			// Set parent to child links
			FirstChild.LayerIndex = ChildLayerIndex;
			FirstChild.NodeIndex = ChildIndexFromCode;

			auto& ChildLayer = Nav3DData.GetLayer(ChildLayerIndex);
			// Set child to parent links
			for (int32 ChildIndex = 0; ChildIndex < 8; ++ChildIndex)
			{
				auto& ChildNode = ChildLayer.GetNodes()[FirstChild.NodeIndex + ChildIndex];
				ChildNode.Parent.LayerIndex = LayerIndex;
				ChildNode.Parent.NodeIndex = LayerNodes.Num(); // index of the new node
			}
		}
		else
		{
			FirstChild.Invalidate();
		}

		LayerNodes.Add(LayerNode);
	}

	// Sort the LayerNodes by MortonCode to ensure they're in the correct order
	LayerNodes.Sort([](const FNav3DNode& A, const FNav3DNode& B)
	{
		return A.MortonCode < B.MortonCode;
	});

	// Progress: distribute 15% across layers above zero
	const int32 LayersAboveZero = FMath::Max(1, GetLayerCount() - 1);
	const float PerLayerShare = 15.0f / LayersAboveZero;
	const float LayerBase = 80.0f + PerLayerShare * (LayerIndex - 1);
	UpdateCoreProgress(LayerBase + PerLayerShare);

	// Core: nudge towards 0.95 as layers finish
	const float CorePerLayer = 0.15f / LayersAboveZero;
	UpdateCoreProgress(FMath::Min(0.95f, 0.80f + CorePerLayer * LayerIndex));

	if (LayerIndex == GetLayerCount() - 1)
	{
		UpdateCoreProgress(0.95f);
	}
}

int32 FNav3DVolumeNavigationData::GetNodeIndexFromMortonCode(
	const LayerIndex LayerIndex, const MortonCode MortonCode) const
{
	QUICK_SCOPE_CYCLE_COUNTER(STAT_Nav3DBoundsNavigationData_GetNodeIndexFromMortonCode);
	const auto& LayerNodes = Nav3DData.GetLayer(LayerIndex).GetNodes();
	return Algo::BinarySearch(LayerNodes, FNav3DNode(MortonCode));
}

void FNav3DVolumeNavigationData::BuildNeighbourLinks(const LayerIndex LayerIdx)
{
	QUICK_SCOPE_CYCLE_COUNTER(STAT_Nav3DBoundsNavigationData_BuildNeighbourLinks);

	UE_LOG(LogNav3D, Log, TEXT("Building neighbour links for layer %d"), LayerIdx);

	auto& LayerNodes = Nav3DData.GetLayer(LayerIdx).GetNodes();
	const auto MaxLayerIndex = GetLayerCount() - 2;

	for (NodeIndex LayerNodeIndex = 0;
	     LayerNodeIndex < static_cast<uint32>(LayerNodes.Num());
	     LayerNodeIndex++)
	{
		auto& Node = LayerNodes[LayerNodeIndex];

		for (NeighbourDirection Direction = 0; Direction < 6; Direction++)
		{
			NodeIndex CurrentNodeIndex = LayerNodeIndex;
			FNav3DNodeAddress& NeighbourAddress = Node.Neighbours[Direction];
			LayerIndex CurrentLayerIndex = LayerIdx;

			while (!FindNeighbourInDirection(NeighbourAddress, CurrentLayerIndex, CurrentNodeIndex, Direction)
				&& CurrentLayerIndex < MaxLayerIndex)
			{
				auto& ParentAddress = Nav3DData.GetLayer(CurrentLayerIndex).GetNodes()[CurrentNodeIndex].Parent;
				if (ParentAddress.IsValid())
				{
					CurrentNodeIndex = ParentAddress.NodeIndex;
					CurrentLayerIndex = ParentAddress.LayerIndex;
				}
				else
				{
					CurrentLayerIndex++;
					const auto NodeIndexFromMorton = GetNodeIndexFromMortonCode(
						CurrentLayerIndex,
						FNav3DUtils::GetParentMortonCode(Node.MortonCode));
					check(NodeIndexFromMorton != INDEX_NONE);
					CurrentNodeIndex = static_cast<NodeIndex>(NodeIndexFromMorton);
				}
			}
		}
	}
}

bool FNav3DVolumeNavigationData::FindNeighbourInDirection(
	FNav3DNodeAddress& NodeAddress, const LayerIndex LayerIndex,
	const NodeIndex NodeIndex, const NeighbourDirection Direction)
{
	QUICK_SCOPE_CYCLE_COUNTER(STAT_Nav3DBoundsNavigationData_FindNeighbourInDirection);

	const auto MaxCoordinates = static_cast<int32>(Nav3DData.GetLayer(LayerIndex).GetMaxNodeCount());
	const auto& LayerNodes = Nav3DData.GetLayer(LayerIndex).GetNodes();
	const auto LayerNodesCount = LayerNodes.Num();
	const auto& TargetNode = LayerNodes[NodeIndex];

	FIntVector NeighbourCoords(FNav3DUtils::GetVectorFromMortonCode(TargetNode.MortonCode));
	NeighbourCoords += GNeighbourDirections[Direction];

	if (NeighbourCoords.X < 0 || NeighbourCoords.X >= MaxCoordinates ||
		NeighbourCoords.Y < 0 || NeighbourCoords.Y >= MaxCoordinates ||
		NeighbourCoords.Z < 0 || NeighbourCoords.Z >= MaxCoordinates)
	{
		NodeAddress.Invalidate();
		return true;
	}

	const auto NeighbourCode = FNav3DUtils::GetMortonCodeFromIntVector(NeighbourCoords);
	int32 StopIndex = LayerNodesCount;
	int32 Increment = 1;

	if (NeighbourCode < TargetNode.MortonCode)
	{
		Increment = -1;
		StopIndex = -1;
	}

	for (int32 NeighbourNodeIndex = NodeIndex + Increment;
	     NeighbourNodeIndex != StopIndex; NeighbourNodeIndex += Increment)
	{
		auto& Node = LayerNodes[NeighbourNodeIndex];

		if (Node.MortonCode == NeighbourCode)
		{
			if (LayerIndex == 0 && Node.HasChildren() &&
				Nav3DData.GetLeafNodes()
				         .GetLeafNode(Node.FirstChild.NodeIndex)
				         .IsCompletelyOccluded())
			{
				NodeAddress.Invalidate();
				return true;
			}

			NodeAddress.LayerIndex = LayerIndex;
			if (NeighbourNodeIndex >= LayerNodesCount || NeighbourNodeIndex < 0)
			{
				break;
			}

			NodeAddress.NodeIndex = NeighbourNodeIndex;
			return true;
		}

		// If we've passed the code we're looking for, it's not on this layer
		if (Increment == -1 && Node.MortonCode < NeighbourCode ||
			Increment == 1 && Node.MortonCode > NeighbourCode)
		{
			return false;
		}
	}
	return false;
}

void FNav3DVolumeNavigationData::GetLeafNeighbours(
	TArray<FNav3DNodeAddress>& Neighbours,
	const FNav3DNodeAddress& LeafAddress) const
{
	QUICK_SCOPE_CYCLE_COUNTER(STAT_Nav3DBoundsNavigationData_GetLeafNeighbours);

	const MortonCode LeafIndex = LeafAddress.SubNodeIndex;
	const FNav3DNode& Node = GetNodeFromAddress(LeafAddress);
	const FNav3DLeafNode& Leaf = Nav3DData.GetLeafNodes().GetLeafNode(Node.FirstChild.NodeIndex);

	uint_fast32_t X = 0, Y = 0, Z = 0;
	morton3D_64_decode(LeafIndex, X, Y, Z);

	for (NeighbourDirection NeighbourDirection = 0; NeighbourDirection < 6;
	     NeighbourDirection++)
	{
		FIntVector NeighbourCoords(X, Y, Z);
		NeighbourCoords += GNeighbourDirections[NeighbourDirection];

		// If the Neighbour is in bounds of this leaf node
		if (NeighbourCoords.X >= 0 && NeighbourCoords.X < 4 &&
			NeighbourCoords.Y >= 0 && NeighbourCoords.Y < 4 &&
			NeighbourCoords.Z >= 0 && NeighbourCoords.Z < 4)
		{
			const MortonCode SubNodeIndex = FNav3DUtils::GetMortonCodeFromIntVector(NeighbourCoords);

			if (!Leaf.IsSubNodeOccluded(SubNodeIndex))
			{
				Neighbours.Emplace(
					FNav3DNodeAddress(0, LeafAddress.NodeIndex, SubNodeIndex));
			}
		}
		else
		{
			const FNav3DNodeAddress& NeighbourAddress =
				Node.Neighbours[NeighbourDirection];
			const FNav3DNode& NeighbourNode = GetNodeFromAddress(NeighbourAddress);

			if (!NeighbourNode.FirstChild.IsValid())
			{
				Neighbours.Add(NeighbourAddress);
				continue;
			}

			const FNav3DLeafNode& LeafNode = Nav3DData.GetLeafNodes().GetLeafNode(
				NeighbourNode.FirstChild.NodeIndex);

			if (!LeafNode.IsCompletelyOccluded())
			{
				if (NeighbourCoords.X < 0)
				{
					NeighbourCoords.X = 3;
				}
				else if (NeighbourCoords.X > 3)
				{
					NeighbourCoords.X = 0;
				}
				else if (NeighbourCoords.Y < 0)
				{
					NeighbourCoords.Y = 3;
				}
				else if (NeighbourCoords.Y > 3)
				{
					NeighbourCoords.Y = 0;
				}
				else if (NeighbourCoords.Z < 0)
				{
					NeighbourCoords.Z = 3;
				}
				else if (NeighbourCoords.Z > 3)
				{
					NeighbourCoords.Z = 0;
				}

				const MortonCode SubNodeIndex =
					FNav3DUtils::GetMortonCodeFromIntVector(NeighbourCoords);

				if (!LeafNode.IsSubNodeOccluded(SubNodeIndex))
				{
					Neighbours.Emplace(FNav3DNodeAddress(
						0, NeighbourNode.FirstChild.NodeIndex, SubNodeIndex));
				}
			}
		}
	}
}

void FNav3DVolumeNavigationData::GetFreeNodesFromNodeAddress(
	const FNav3DNodeAddress NodeAddress,
	TArray<FNav3DNodeAddress>& FreeNodes) const
{
	const auto LayerIndex = NodeAddress.LayerIndex;
	const auto NodeIndex = NodeAddress.NodeIndex;

	if (LayerIndex == 0)
	{
		const auto& LeafNode = Nav3DData.LeafNodes.GetLeafNode(NodeIndex);

		if (LeafNode.IsCompletelyOccluded())
		{
			return;
		}

		if (LeafNode.IsCompletelyFree())
		{
			FreeNodes.Emplace(NodeAddress);
			return;
		}

		for (auto MortonCode = 0; MortonCode < 64; ++MortonCode)
		{
			if (!LeafNode.IsSubNodeOccluded(MortonCode))
			{
				FreeNodes.Emplace(FNav3DNodeAddress(0, NodeIndex, MortonCode));
			}
		}
	}
	else
	{
		const auto& Node = Nav3DData.GetLayer(LayerIndex).GetNode(NodeIndex);

		if (!Node.HasChildren())
		{
			FreeNodes.Emplace(NodeAddress);
		}
		else
		{
			const auto& FirstChild = Node.FirstChild;
			const auto ChildLayerIndex = FirstChild.LayerIndex;
			const auto& ChildLayer = Nav3DData.GetLayer(ChildLayerIndex);

			for (auto ChildIndex = 0; ChildIndex < 8; ++ChildIndex)
			{
				const auto& ChildNode =
					ChildLayer.GetNodes()[FirstChild.NodeIndex + ChildIndex];
				GetFreeNodesFromNodeAddress(
					FNav3DNodeAddress(ChildLayerIndex, ChildNode.MortonCode, 0),
					FreeNodes);
			}
		}
	}
}

void FNav3DVolumeNavigationData::BuildParentLinkForLeafNodes(
	const TMap<LeafIndex, MortonCode>& LeafIndexToParentMortonCodeMap)
{
	for (const auto& KeyPair : LeafIndexToParentMortonCodeMap)
	{
		auto& LeafNode = Nav3DData.GetLeafNodes().GetLeafNode(KeyPair.Key);
		LeafNode.Parent.LayerIndex = 1;

		const auto NodeIndex = GetNodeIndexFromMortonCode(1, KeyPair.Value);
		check(NodeIndex != INDEX_NONE);

		LeafNode.Parent.NodeIndex = NodeIndex;
	}
}

void FNav3DVolumeNavigationData::PropagateChangesToHigherLayers(const TSet<MortonCode>& ModifiedLeafCodes,
                                                                const LayerIndex StartLayer)
{
	const int32 LayerCount = Nav3DData.GetLayerCount();
	TSet<MortonCode> CurrentLayerModifiedCodes = ModifiedLeafCodes;
	for (LayerIndex LayerIdx = StartLayer; LayerIdx < LayerCount; LayerIdx++)
	{
		TSet<MortonCode> ParentCodes;
		auto& Layer = Nav3DData.GetLayer(LayerIdx);

		// Remove nodes that no longer have any occluded children
		for (int32 NodeIdx = Layer.GetNodes().Num() - 1; NodeIdx >= 0; --NodeIdx)
		{
			auto& Node = Layer.GetNodes()[NodeIdx];
			bool bHasOccludedChildren = false;

			if (Node.HasChildren())
			{
				// Check if any children are still occluded
				const MortonCode FirstChildCode = FNav3DUtils::GetFirstChildMortonCode(Node.MortonCode);
				for (int32 ChildIdx = 0; ChildIdx < 8; ChildIdx++)
				{
					const MortonCode ChildCode = FirstChildCode + ChildIdx;
					const int32 ChildNodeIdx = GetNodeIndexFromMortonCode(LayerIdx - 1, ChildCode);

					if (ChildNodeIdx != INDEX_NONE)
					{
						const auto& ChildNode = Nav3DData.GetLayer(LayerIdx - 1).GetNodes()[ChildNodeIdx];
						if (ChildNode.HasChildren())
						{
							bHasOccludedChildren = true;
							break;
						}
					}
				}
			}

			if (!bHasOccludedChildren)
			{
				Layer.GetNodes().RemoveAtSwap(NodeIdx, 1, EAllowShrinking::No);
			}
		}

		// Get parent codes for all modified nodes in previous layer
		for (const MortonCode& Code : CurrentLayerModifiedCodes)
		{
			ParentCodes.Add(FNav3DUtils::GetParentMortonCode(Code));
		}

		// Update or create parent nodes
		for (const MortonCode& ParentCode : ParentCodes)
		{
			// Find node index for this morton code
			int32 NodeIdx = GetNodeIndexFromMortonCode(LayerIdx, ParentCode);
			const bool bNodeExists = (NodeIdx != INDEX_NONE);

			if (!bNodeExists)
			{
				// Create new node if it doesn't exist
				FNav3DNode NewNode;
				NewNode.MortonCode = ParentCode;
				NewNode.FirstChild.LayerIndex = LayerIdx - 1;
				NodeIdx = Layer.GetNodes().Add(NewNode);
			}

			auto& ParentNode = Layer.GetNodes()[NodeIdx];

			// Find child nodes
			bool bHasOccludedChildren = false;
			const MortonCode FirstChildCode = FNav3DUtils::GetFirstChildMortonCode(ParentCode);

			if (LayerIdx == 1)
			{
				// When creating Layer 1 nodes, link to Layer 0 (leaf) nodes
				for (int32 ChildIdx = 0; ChildIdx < 8; ChildIdx++)
				{
					const MortonCode ChildCode = FirstChildCode + ChildIdx;
					const int32 ChildNodeIdx = GetNodeIndexFromMortonCode(LayerIdx - 1, ChildCode);

					if (ChildNodeIdx != INDEX_NONE)
					{
						auto& ChildNode = Nav3DData.GetLayer(LayerIdx - 1).GetNodes()[ChildNodeIdx];
						if (ChildNode.HasChildren())
						{
							if (!ParentNode.FirstChild.IsValid())
							{
								ParentNode.FirstChild.LayerIndex = LayerIdx - 1;
								ParentNode.FirstChild.NodeIndex = ChildNodeIdx;
							}
							bHasOccludedChildren = true;

							// Set parent reference in child
							ChildNode.Parent.LayerIndex = LayerIdx;
							ChildNode.Parent.NodeIndex = NodeIdx;
						}
					}
				}
			}
			else
			{
				// For higher layers, link to their respective child layers
				for (int32 ChildIdx = 0; ChildIdx < 8; ChildIdx++)
				{
					const MortonCode ChildCode = FirstChildCode + ChildIdx;
					const int32 ChildNodeIdx = GetNodeIndexFromMortonCode(LayerIdx - 1, ChildCode);

					if (ChildNodeIdx != INDEX_NONE)
					{
						auto& ChildNode = Nav3DData.GetLayer(LayerIdx - 1).GetNodes()[ChildNodeIdx];
						if (ChildNode.HasChildren())
						{
							if (!ParentNode.FirstChild.IsValid())
							{
								ParentNode.FirstChild.LayerIndex = LayerIdx - 1;
								ParentNode.FirstChild.NodeIndex = ChildNodeIdx;
							}
							bHasOccludedChildren = true;

							// Set parent reference in child
							ChildNode.Parent.LayerIndex = LayerIdx;
							ChildNode.Parent.NodeIndex = NodeIdx;
						}
					}
				}
			}

			// If this node has no occluded children, invalidate its FirstChild reference
			if (!bHasOccludedChildren)
			{
				ParentNode.FirstChild.Invalidate();
			}
		}

		CurrentLayerModifiedCodes = ParentCodes;

		// Sort nodes in the layer to maintain proper ordering
		Layer.GetNodes().Sort([](const FNav3DNode& A, const FNav3DNode& B)
		{
			return A.MortonCode < B.MortonCode;
		});
	}
}

bool FNav3DVolumeNavigationData::IsNodeInBounds(const FVector& NodePosition, const float NodeExtent, const FBox& Bounds)
{
	const FBox NodeBox = FBox::BuildAABB(NodePosition, FVector(NodeExtent));
	return NodeBox.Intersect(Bounds);
}

void FNav3DVolumeNavigationData::AddDynamicOccluder(const AActor* Occluder)
{
	if (!Occluder)
	{
		return;
	}

	UE_LOG(LogNav3D, Verbose, TEXT("Adding dynamic occluder %s to volume"), *Occluder->GetActorNameOrLabel());

	// Remove any existing weak pointer to this actor
	DynamicOccluders.RemoveAllSwap([Occluder](const TWeakObjectPtr<const AActor>& Existing)
	{
		return !Existing.IsValid() || Existing.Get() == Occluder;
	});

	// Add the new occluder
	DynamicOccluders.Add(Occluder);

	UE_LOG(LogNav3D, Verbose, TEXT("Dynamic occluders count: %d"), DynamicOccluders.Num());
}

void FNav3DVolumeNavigationData::RemoveDynamicOccluder(const AActor* Occluder)
{
	if (!Occluder)
	{
		return;
	}

	const int32 NumRemoved = DynamicOccluders.RemoveAllSwap([Occluder](const TWeakObjectPtr<const AActor>& Existing)
	{
		return !Existing.IsValid() || Existing.Get() == Occluder;
	});

	if (NumRemoved > 0)
	{
		UE_LOG(LogNav3D, Verbose, TEXT("Removed dynamic occluder %s from volume. Remaining occluders: %d"),
		       *Occluder->GetActorNameOrLabel(), DynamicOccluders.Num());
	}
}

MortonCode FNav3DVolumeNavigationData::GetParentMortonCodeAtLayer(const MortonCode ChildCode, const LayerIndex TargetLayer, const LayerIndex ChildLayer)
{
	if (TargetLayer >= ChildLayer)
	{
		return ChildCode;
	}
	LayerIndex Current = ChildLayer;
	MortonCode Code = ChildCode;
	while (Current > TargetLayer)
	{
		Code = FNav3DUtils::GetParentMortonCode(Code);
		--Current;
	}
	return Code;
}

float FNav3DVolumeNavigationData::GetLayerNodeSize(const LayerIndex LayerIndex) const
{
	if (LayerIndex == 0)
	{
		// For layer 0, return leaf node size
		return Nav3DData.GetLeafNodes().GetLeafNodeSize();
	}
	
	if (LayerIndex >= Nav3DData.GetLayerCount())
	{
		UE_LOG(LogNav3D, Warning, TEXT("GetLayerNodeSize: LayerIndex %d out of bounds"), LayerIndex);
		return 0.0f;
	}
	
	return Nav3DData.GetLayer(LayerIndex).GetNodeSize();
}

float FNav3DVolumeNavigationData::GetLayerNodeExtent(const LayerIndex LayerIndex) const
{
	if (LayerIndex == 0)
	{
		// For layer 0, return leaf node extent
		return Nav3DData.GetLeafNodes().GetLeafNodeExtent();
	}
	
	if (LayerIndex >= Nav3DData.GetLayerCount())
	{
		UE_LOG(LogNav3D, Warning, TEXT("GetLayerNodeExtent: LayerIndex %d out of bounds"), LayerIndex);
		return 0.0f;
	}
	
	return Nav3DData.GetLayer(LayerIndex).GetNodeExtent();
}

bool FNav3DVolumeNavigationData::GetNodeAddressFromMortonCode(
	FNav3DNodeAddress& OutNodeAddress,
	const MortonCode MortonCode,
	const LayerIndex LayerIndex) const
{
	if (LayerIndex >= Nav3DData.GetLayerCount())
	{
		UE_LOG(LogNav3D, VeryVerbose, TEXT("GetNodeAddressFromMortonCode: LayerIndex %d out of bounds"), LayerIndex);
		return false;
	}
	
	// Find the node index for this morton code at the specified layer
	const int32 NodeIndex = GetNodeIndexFromMortonCode(LayerIndex, MortonCode);
	if (NodeIndex == INDEX_NONE)
	{
		return false; // Morton code doesn't exist at this layer
	}
	
	OutNodeAddress.LayerIndex = LayerIndex;
	OutNodeAddress.NodeIndex = static_cast<uint32>(NodeIndex);
	OutNodeAddress.SubNodeIndex = 0; // Default sub-node for non-leaf nodes
	
	return true;
}

const TArray<NodeIndex>& FNav3DVolumeNavigationData::GetLayerBlockedNodes(const LayerIndex LayerIndex) const
{
	return Nav3DData.GetLayerBlockedNodes(LayerIndex);
}