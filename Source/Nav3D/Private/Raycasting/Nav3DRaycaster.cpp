#include "Raycasting/Nav3DRaycaster.h"
#include "Nav3DUtils.h"
#include "Nav3DVolumeNavigationData.h"
#include "Nav3D.h"
#include "Misc/ConfigCacheIni.h"
#include "HAL/IConsoleManager.h"

// Console variable to gate VeryVerbose raycaster logs
static TAutoConsoleVariable<int32> CVarNav3DRaycasterVeryVerbose(
    TEXT("nav3d.Raycaster.VeryVerbose"),
    0,
    TEXT("Set to 1 to enable VeryVerbose logs for Nav3D raycaster; 0 to suppress."),
    ECVF_Default);

FNav3DRaycasterProcessor_GenerateDebugInfos::
FNav3DRaycasterProcessor_GenerateDebugInfos(
    FNav3DRaycasterDebugInfos& DebugInfos)
    : DebugInfos(DebugInfos)
{
}

void FNav3DRaycasterProcessor_GenerateDebugInfos::Initialize(
    const FNav3DVolumeNavigationData* NavigationData, const FVector From,
    const FVector To)
{
    DebugInfos.TraversedNodes.Reset();
    DebugInfos.TraversedLeafNodes.Reset();
    DebugInfos.TraversedLeafSubNodes.Reset();
    DebugInfos.RaycastStartLocation = From;
    DebugInfos.RaycastEndLocation = To;
    DebugInfos.NavigationData = NavigationData;
}

void FNav3DRaycasterProcessor_GenerateDebugInfos::SetResult(const bool Result)
{
    DebugInfos.Result = Result;
}

void FNav3DRaycasterProcessor_GenerateDebugInfos::AddTraversedNode(
    FNav3DNodeAddress NodeAddress, bool IsOccluded)
{
    UE_LOG(LogNav3D, VeryVerbose, TEXT("Node Address : %i - %i - %i"),
           NodeAddress.LayerIndex, NodeAddress.NodeIndex,
           NodeAddress.SubNodeIndex);
    DebugInfos.TraversedNodes.Emplace(NodeAddress, IsOccluded);
}

void FNav3DRaycasterProcessor_GenerateDebugInfos::AddTraversedLeafSubNode(
    FNav3DNodeAddress NodeAddress, bool IsOccluded)
{
    UE_LOG(LogNav3D, VeryVerbose, TEXT("SubNode Address : %i - %i - %i"),
           NodeAddress.LayerIndex, NodeAddress.NodeIndex,
           NodeAddress.SubNodeIndex);
    DebugInfos.TraversedLeafSubNodes.Emplace(NodeAddress, IsOccluded);
}

bool UNav3DRaycaster::Trace(
    const FNav3DVolumeNavigationData& VolumeNavigationData,
    const FVector& From,
    const FVector& To) const
{
    if (Processor.IsValid())
    {
       Processor->Initialize(&VolumeNavigationData, From, To);
    }

    FNav3DRaycastHit OutHit;
    const auto Result = Trace(VolumeNavigationData, From, To, OutHit);

    if (Processor.IsValid())
    {
       Processor->SetResult(Result);
    }

    return Result;
}

bool UNav3DRaycaster::Trace(
    const FNav3DVolumeNavigationData& VolumeNavigationData,
    const FVector& From,
    const FVector& To,
    FNav3DRaycastHit& OutHit) const
{
    return TraceInternal(VolumeNavigationData, From, To, OutHit);
}

UWorld* UNav3DRaycaster::GetWorldContext()
{
#if WITH_EDITOR
    return GEditor->GetEditorWorldContext(false).World();
#else
    return GEngine->GetCurrentPlayWorld();
#endif
}

void UNav3DRaycaster::SetProcessor(
    const TSharedPtr<FNav3DRaycasterProcessor>& NewProcessor)
{
    Processor = NewProcessor;
}

bool UNav3DRaycaster::TraceCountingOccludedVoxels(
    const FNav3DVolumeNavigationData& VolumeNavigationData,
    const FVector& From,
    const FVector& To,
    FNav3DRaycastHit& OutHit) const
{
    // Reset the count to make sure we're starting fresh
    OutHit.OccludedVoxelCount = 0;
    
    // Perform the trace with voxel counting enabled
    const bool bHit = TraceInternal(VolumeNavigationData, From, To, OutHit, true);
    
    return bHit;
}

int32 UNav3DRaycaster::CountOccludedVoxelsAlongRay(
    const FNav3DVolumeNavigationData& VolumeNavigationData,
    const FVector& From,
    const FVector& To) const
{
    FNav3DRaycastHit OutHit;
    TraceInternal(VolumeNavigationData, From, To, OutHit, true);
    return OutHit.OccludedVoxelCount;
}

bool UNav3DRaycaster::TraceInternal(
    const FNav3DVolumeNavigationData& VolumeNavigationData,
    const FVector& From,
    const FVector& To,
    FNav3DRaycastHit& OutHit) const
{
    return TraceInternal(VolumeNavigationData, From, To, OutHit, false);
}

bool UNav3DRaycaster::TraceInternal(
    const FNav3DVolumeNavigationData& VolumeNavigationData,
    const FVector& From,
    const FVector& To,
    FNav3DRaycastHit& OutHit,
    const bool bCountAllOccludedVoxels) const
{
    const auto& NavigationBounds = VolumeNavigationData.GetNavigationBounds();
    FVector VolumeCenter;
    FVector VolumeExtent;
    NavigationBounds.GetCenterAndExtents(VolumeCenter, VolumeExtent);

    FRaycastState RayState(From, To);
    FVector RayDirection = RayState.RayDirection;

    // Handle near-zero components
    if (FMath::IsNearlyZero(RayDirection.X, KINDA_SMALL_NUMBER))
    {
        RayDirection.X = KINDA_SMALL_NUMBER;
    }
    if (FMath::IsNearlyZero(RayDirection.Y, KINDA_SMALL_NUMBER))
    {
        RayDirection.Y = KINDA_SMALL_NUMBER;
    }
    if (FMath::IsNearlyZero(RayDirection.Z, KINDA_SMALL_NUMBER))
    {
        RayDirection.Z = KINDA_SMALL_NUMBER;
    }

    FVector RayOrigin = From;

    // Transform ray for octree traversal
    if (RayDirection.X < 0.0f)
    {
        RayOrigin.X = VolumeCenter.X * 2.0f - RayOrigin.X;
        RayDirection.X = -RayDirection.X;
        RayState.A |= 1;
    }
    if (RayDirection.Y < 0.0f)
    {
        RayOrigin.Y = VolumeCenter.Y * 2.0f - RayOrigin.Y;
        RayDirection.Y = -RayDirection.Y;
        RayState.A |= 2;
    }
    if (RayDirection.Z < 0.0f)
    {
        RayOrigin.Z = VolumeCenter.Z * 2.0f - RayOrigin.Z;
        RayDirection.Z = -RayDirection.Z;
        RayState.A |= 4;
    }

    const auto DivX = 1.0f / RayDirection.X;
    const auto DivY = 1.0f / RayDirection.Y;
    const auto DivZ = 1.0f / RayDirection.Z;

    const FOctreeRay OctreeRay(
        (NavigationBounds.Min.X - RayOrigin.X) * DivX,
        (NavigationBounds.Max.X - RayOrigin.X) * DivX,
        (NavigationBounds.Min.Y - RayOrigin.Y) * DivY,
        (NavigationBounds.Max.Y - RayOrigin.Y) * DivY,
        (NavigationBounds.Min.Z - RayOrigin.Z) * DivZ,
        (NavigationBounds.Max.Z - RayOrigin.Z) * DivZ);

    if (!OctreeRay.Intersects())
    {
        return false;
    }

    // Initialize occluded voxel count
    OutHit.OccludedVoxelCount = 0;

    // Start at highest layer and traverse down
    const int32 HighestLayer = VolumeNavigationData.GetData().GetLayerCount() - 1;
    if (CVarNav3DRaycasterVeryVerbose.GetValueOnAnyThread() != 0)
    {
        UE_LOG(LogNav3D, VeryVerbose, TEXT("Raycaster: Starting traversal from layer %d, ray from %s to %s"), 
               HighestLayer, *From.ToString(), *To.ToString());
    }
    
    const auto Result = DoesRayIntersectOccludedNode(
        OctreeRay,
        FNav3DNodeAddress(HighestLayer, 0),
        VolumeNavigationData,
        RayState,
        OutHit,
        bCountAllOccludedVoxels);

    if (Result && OutHit.bBlockingHit)
    {
        // Transform hit point back if ray was transformed
        if (RayState.A & 1)
        {
            OutHit.ImpactPoint.X = VolumeCenter.X * 2.0f - OutHit.ImpactPoint.X;
        }
        if (RayState.A & 2)
        {
            OutHit.ImpactPoint.Y = VolumeCenter.Y * 2.0f - OutHit.ImpactPoint.Y;
        }
        if (RayState.A & 4)
        {
            OutHit.ImpactPoint.Z = VolumeCenter.Z * 2.0f - OutHit.ImpactPoint.Z;
        }
    }

    return Result;
}

bool UNav3DRaycaster::DoesRayIntersectOccludedNode(
    const FOctreeRay& Ray,
    const FNav3DNodeAddress& NodeAddress,
    const FNav3DVolumeNavigationData& Data,
    const FRaycastState& RayState,
    FNav3DRaycastHit& OutHit,
    const bool bCountAllOccludedVoxels) const
{
    if (CVarNav3DRaycasterVeryVerbose.GetValueOnAnyThread() != 0)
    {
        UE_LOG(LogNav3D, VeryVerbose, TEXT("Raycaster: DoesRayIntersectOccludedNode - Layer %d, Node %d"), 
               NodeAddress.LayerIndex, NodeAddress.NodeIndex);
    }
    
    if (!Ray.IsInRange(RayState.RaySize))
    {
        if (CVarNav3DRaycasterVeryVerbose.GetValueOnAnyThread() != 0)
        {
            UE_LOG(LogNav3D, VeryVerbose, TEXT("Raycaster: Node %d is out of range"), NodeAddress.NodeIndex);
        }
        return false;
    }

    // If this is a layer 0 node, test for actual intersection
    if (NodeAddress.LayerIndex == 0)
    {
        if (CVarNav3DRaycasterVeryVerbose.GetValueOnAnyThread() != 0)
        {
            UE_LOG(LogNav3D, VeryVerbose, TEXT("Raycaster: Reached layer 0 node %d, checking for occluded leaf"), NodeAddress.NodeIndex);
        }
        const bool bResult = DoesRayIntersectOccludedLeaf(Ray, NodeAddress, Data, RayState, OutHit, bCountAllOccludedVoxels);
        if (CVarNav3DRaycasterVeryVerbose.GetValueOnAnyThread() != 0)
        {
            UE_LOG(LogNav3D, VeryVerbose, TEXT("Raycaster: Layer 0 node %d result: %s"), NodeAddress.NodeIndex, bResult ? TEXT("HIT") : TEXT("MISS"));
        }
        return bResult;
    }

    // Otherwise use higher layers for traversal optimization
    return DoesRayIntersectOccludedNormalNode(Ray, NodeAddress, Data, RayState, OutHit, bCountAllOccludedVoxels);
}

bool UNav3DRaycaster::DoesRayIntersectOccludedLeaf(
    const FOctreeRay& Ray,
    const FNav3DNodeAddress& NodeAddress,
    const FNav3DVolumeNavigationData& Data,
    const FRaycastState& RayState,
    FNav3DRaycastHit& OutHit,
    const bool bCountAllOccludedVoxels)
{
    if (CVarNav3DRaycasterVeryVerbose.GetValueOnAnyThread() != 0)
    {
        UE_LOG(LogNav3D, VeryVerbose, TEXT("Raycaster: DoesRayIntersectOccludedLeaf - Testing node %d"), NodeAddress.NodeIndex);
    }
    
    const auto& LeafNodes = Data.GetData().GetLeafNodes();
    if (!LeafNodes.GetLeafNodes().IsValidIndex(NodeAddress.NodeIndex))
    {
        if (CVarNav3DRaycasterVeryVerbose.GetValueOnAnyThread() != 0)
        {
            UE_LOG(LogNav3D, VeryVerbose, TEXT("Raycaster: Node index %d is invalid (max: %d)"), 
                   NodeAddress.NodeIndex, LeafNodes.GetLeafNodes().Num() - 1);
        }
        return false;
    }

    const auto& LeafNode = LeafNodes.GetLeafNode(NodeAddress.NodeIndex);
    if (CVarNav3DRaycasterVeryVerbose.GetValueOnAnyThread() != 0)
    {
        UE_LOG(LogNav3D, VeryVerbose, TEXT("Raycaster: Leaf node %d - IsCompletelyFree: %s, IsCompletelyOccluded: %s"), 
               NodeAddress.NodeIndex, 
               LeafNode.IsCompletelyFree() ? TEXT("TRUE") : TEXT("FALSE"),
               LeafNode.IsCompletelyOccluded() ? TEXT("TRUE") : TEXT("FALSE"));
    }
    
    if (LeafNode.IsCompletelyFree())
    {
        if (CVarNav3DRaycasterVeryVerbose.GetValueOnAnyThread() != 0)
        {
            UE_LOG(LogNav3D, VeryVerbose, TEXT("Raycaster: Node %d is completely free, no hit"), NodeAddress.NodeIndex);
        }
        return false;
    }

    // Get LEAF bounds for intersection test (use full leaf AABB, not a single sub-node)
    const FVector LeafCenter = Data.GetNodePositionFromAddress(NodeAddress, false);
    const float LeafExtent = Data.GetData().GetLeafNodes().GetLeafNodeExtent();
    const FBox NodeBox = FBox::BuildAABB(LeafCenter, FVector(LeafExtent));
    
    if (CVarNav3DRaycasterVeryVerbose.GetValueOnAnyThread() != 0)
    {
        UE_LOG(LogNav3D, VeryVerbose, TEXT("Raycaster: Node %d leaf bounds - Center: %s, Extent: %.2f, Box: %s"), 
               NodeAddress.NodeIndex, *LeafCenter.ToString(), LeafExtent, *NodeBox.ToString());
    }

    // Test intersection with node bounds first - use original ray direction
    float TMin, TMax;
    if (!FNav3DUtils::RayBoxIntersection(NodeBox, RayState.RayOrigin, RayState.OriginalRayDirection, RayState.RaySize, TMin,
                                         TMax))
    {
        if (CVarNav3DRaycasterVeryVerbose.GetValueOnAnyThread() != 0)
        {
            UE_LOG(LogNav3D, VeryVerbose, TEXT("Raycaster: Node %d - Ray does not intersect node bounds"), NodeAddress.NodeIndex);
        }
        return false;
    }
    
    if (CVarNav3DRaycasterVeryVerbose.GetValueOnAnyThread() != 0)
    {
        UE_LOG(LogNav3D, VeryVerbose, TEXT("Raycaster: Node %d - Ray intersects bounds at TMin=%.4f, TMax=%.4f"), 
               NodeAddress.NodeIndex, TMin, TMax);
    }

    // If leaf node is completely occluded, it counts as a single voxel hit
    if (LeafNode.IsCompletelyOccluded())
    {
        // Increment occluded voxel count
        OutHit.OccludedVoxelCount++;
        
        if (CVarNav3DRaycasterVeryVerbose.GetValueOnAnyThread() != 0)
        {
            UE_LOG(LogNav3D, VeryVerbose, TEXT("Raycaster: *** HIT! *** Found completely occluded leaf node %d at %s, voxel count now %d"), 
                   NodeAddress.NodeIndex, *LeafCenter.ToString(), OutHit.OccludedVoxelCount);
        }
        
        // Record hit information if this is the first hit we've found
        if (!OutHit.bBlockingHit || TMin < OutHit.Distance)
        {
            OutHit.ImpactPoint = RayState.RayOrigin + RayState.OriginalRayDirection * TMin;
            OutHit.ImpactNormal = CalculateImpactNormal(OutHit.ImpactPoint, LeafCenter);
            OutHit.Distance = TMin;
            OutHit.NodeAddress = NodeAddress;
            OutHit.bBlockingHit = true;
        }
        
        // If we're not counting all voxels, we can return after the first hit
        if (!bCountAllOccludedVoxels)
        {
            return true;
        }
        
        // Otherwise, we signal that we found a hit, but continue traversal
        return true;
    }
    
    // Leaf node has sub-nodes, test them individually
    if (CVarNav3DRaycasterVeryVerbose.GetValueOnAnyThread() != 0)
    {
        UE_LOG(LogNav3D, VeryVerbose, TEXT("Raycaster: Node %d has sub-nodes, testing 64 sub-nodes"), NodeAddress.NodeIndex);
    }
    
    bool bHit = false;
    float ClosestHit = MAX_flt;
    FNav3DNodeAddress ClosestSubNode = NodeAddress;
    int32 OccludedSubNodes = 0;
    int32 IntersectingSubNodes = 0;

    // Test each sub-node
    for (SubNodeIndex SubIdx = 0; SubIdx < 64; SubIdx++)
    {
        if (!LeafNode.IsSubNodeOccluded(SubIdx))
        {
            continue;
        }
        
        OccludedSubNodes++;

        // Calculate sub-node position and bounds
        const auto SubNodePos = LeafCenter + FNav3DUtils::GetSubNodeOffset(SubIdx, LeafExtent);
        const float SubNodeExtent = LeafExtent * 0.25f;
        const FBox SubNodeBox = FBox::BuildAABB(SubNodePos, FVector(SubNodeExtent));

        float SubTMin, SubTMax;
        if (FNav3DUtils::RayBoxIntersection(
            SubNodeBox, RayState.RayOrigin, RayState.OriginalRayDirection, RayState.RaySize, SubTMin, SubTMax))
        {
            IntersectingSubNodes++;
            // Increment occluded voxel count for each hit subnode
            OutHit.OccludedVoxelCount++;
            
            if (CVarNav3DRaycasterVeryVerbose.GetValueOnAnyThread() != 0)
            {
                UE_LOG(LogNav3D, VeryVerbose, TEXT("Raycaster: *** HIT! *** Found occluded sub-node %d at %s (TMin=%.4f), voxel count now %d"), 
                       SubIdx, *SubNodePos.ToString(), SubTMin, OutHit.OccludedVoxelCount);
            }
            
            // Keep track of closest hit for return value
            bHit = true;
            
            // Only update impact information if this is the closest hit so far
            if (SubTMin < ClosestHit)
            {
                ClosestHit = SubTMin;
                ClosestSubNode.SubNodeIndex = SubIdx;

                // Update hit information if this is the first or closest hit
                if (!OutHit.bBlockingHit || SubTMin < OutHit.Distance)
                {
                    OutHit.ImpactPoint = RayState.RayOrigin + RayState.OriginalRayDirection * SubTMin;
                    OutHit.ImpactNormal = CalculateImpactNormal(OutHit.ImpactPoint, SubNodePos);
                    OutHit.Distance = SubTMin;
                    OutHit.NodeAddress = ClosestSubNode;
                    OutHit.bBlockingHit = true;
                }
            }
            
            // If we're not counting all voxels, we can return after the first hit
            if (!bCountAllOccludedVoxels)
            {
                return true;
            }
            
            // Otherwise we continue to count all hits
        }
    }
    
    if (CVarNav3DRaycasterVeryVerbose.GetValueOnAnyThread() != 0)
    {
        UE_LOG(LogNav3D, VeryVerbose, TEXT("Raycaster: Node %d sub-node testing complete - Occluded: %d, Intersecting: %d, Hit: %s"), 
               NodeAddress.NodeIndex, OccludedSubNodes, IntersectingSubNodes, bHit ? TEXT("TRUE") : TEXT("FALSE"));
    }

    return bHit;
}

bool UNav3DRaycaster::DoesRayIntersectOccludedNormalNode(
    const FOctreeRay& Ray,
    const FNav3DNodeAddress& NodeAddress,
    const FNav3DVolumeNavigationData& Data,
    const FRaycastState& RayState,
    FNav3DRaycastHit& OutHit,
    const bool bCountAllOccludedVoxels) const
{
    if (CVarNav3DRaycasterVeryVerbose.GetValueOnAnyThread() != 0)
    {
        UE_LOG(LogNav3D, VeryVerbose, TEXT("Raycaster: DoesRayIntersectOccludedNormalNode - Layer %d, Node %d"), 
               NodeAddress.LayerIndex, NodeAddress.NodeIndex);
    }
    
    // Validate layer and node indices
    const auto& NavData = Data.GetData();
    if (NodeAddress.LayerIndex >= NavData.GetLayerCount())
    {
        if (CVarNav3DRaycasterVeryVerbose.GetValueOnAnyThread() != 0)
        {
            UE_LOG(LogNav3D, VeryVerbose, TEXT("Raycaster: Layer %d >= LayerCount %d"), 
                   NodeAddress.LayerIndex, NavData.GetLayerCount());
        }
        return false;
    }

    const auto& Layer = NavData.GetLayer(NodeAddress.LayerIndex);
    if (!Layer.GetNodes().IsValidIndex(NodeAddress.NodeIndex))
    {
        if (CVarNav3DRaycasterVeryVerbose.GetValueOnAnyThread() != 0)
        {
            UE_LOG(LogNav3D, VeryVerbose, TEXT("Raycaster: Node index %d invalid for layer %d (max: %d)"), 
                   NodeAddress.NodeIndex, NodeAddress.LayerIndex, Layer.GetNodes().Num() - 1);
        }
        return false;
    }

    const auto& Node = Data.GetData()
                           .GetLayer(NodeAddress.LayerIndex)
                           .GetNode(NodeAddress.NodeIndex);

    if (!Node.HasChildren())
    {
        if (CVarNav3DRaycasterVeryVerbose.GetValueOnAnyThread() != 0)
        {
            UE_LOG(LogNav3D, VeryVerbose, TEXT("Raycaster: Node %d has no children"), NodeAddress.NodeIndex);
        }
        return false;
    }

    // Get node bounds for intersection test
    const FVector NodePos = Data.GetNodePositionFromAddress(NodeAddress, true);
    const float NodeExtent = Data.GetNodeExtentFromNodeAddress(NodeAddress);
    const FBox NodeBox = FBox::BuildAABB(NodePos, FVector(NodeExtent));
    
    if (CVarNav3DRaycasterVeryVerbose.GetValueOnAnyThread() != 0)
    {
        UE_LOG(LogNav3D, VeryVerbose, TEXT("Raycaster: Node %d bounds - Center: %s, Extent: %.2f, Box: %s"), 
               NodeAddress.NodeIndex, *NodePos.ToString(), NodeExtent, *NodeBox.ToString());
    }

    // Test intersection with node bounds first
    float TMin, TMax;
    if (!FNav3DUtils::RayBoxIntersection(NodeBox, RayState.RayOrigin, RayState.RayDirection, RayState.RaySize, TMin, TMax))
    {
        if (CVarNav3DRaycasterVeryVerbose.GetValueOnAnyThread() != 0)
        {
            UE_LOG(LogNav3D, VeryVerbose, TEXT("Raycaster: Node %d - Ray does not intersect node bounds"), NodeAddress.NodeIndex);
        }
        return false;
    }
    
    if (CVarNav3DRaycasterVeryVerbose.GetValueOnAnyThread() != 0)
    {
        UE_LOG(LogNav3D, VeryVerbose, TEXT("Raycaster: Node %d - Ray intersects bounds at TMin=%.4f, TMax=%.4f"), 
               NodeAddress.NodeIndex, TMin, TMax);
    }

    const auto& FirstChildAddress = Node.FirstChild;
    if (!FirstChildAddress.IsValid())
    {
        if (CVarNav3DRaycasterVeryVerbose.GetValueOnAnyThread() != 0)
        {
            UE_LOG(LogNav3D, VeryVerbose, TEXT("Raycaster: Node %d has no valid first child"), NodeAddress.NodeIndex);
        }
        return false;
    }

    // Start with the first child node in the traversal
    uint8 ChildIndex = GetFirstNodeIndex(Ray);
    bool bFoundAnyHit = false;
    
    if (CVarNav3DRaycasterVeryVerbose.GetValueOnAnyThread() != 0)
    {
        UE_LOG(LogNav3D, VeryVerbose, TEXT("Raycaster: Node %d - Starting child traversal with ChildIndex=%d, FirstChildAddress=(Layer=%d, Node=%d)"), 
               NodeAddress.NodeIndex, ChildIndex, FirstChildAddress.LayerIndex, FirstChildAddress.NodeIndex);
    }

    do
    {
        // Create the child address with reflected index based on ray direction
        const int32 ReflectedChildNodeIndex = ChildIndex ^ RayState.A;
        const FNav3DNodeAddress NewChildAddress(FirstChildAddress.LayerIndex, 
                                               FirstChildAddress.NodeIndex + ReflectedChildNodeIndex);
        
        if (CVarNav3DRaycasterVeryVerbose.GetValueOnAnyThread() != 0)
        {
            UE_LOG(LogNav3D, VeryVerbose, TEXT("Raycaster: Node %d - Testing child %d (ReflectedIndex=%d, NewAddress=(Layer=%d, Node=%d))"), 
                   NodeAddress.NodeIndex, ChildIndex, ReflectedChildNodeIndex, NewChildAddress.LayerIndex, NewChildAddress.NodeIndex);
        }

        bool bChildHit;
        
        // Process different children with specifically adjusted ray parameters
        switch (ChildIndex)
        {
        case 0:
            bChildHit = DoesRayIntersectOccludedNode(
                FOctreeRay(Ray.Tx0, Ray.Txm, Ray.Ty0, Ray.Tym, Ray.Tz0, Ray.Tzm),
                NewChildAddress, Data, RayState, OutHit, bCountAllOccludedVoxels);
            if (bChildHit)
            {
                bFoundAnyHit = true;
                if (!bCountAllOccludedVoxels)
                {
                    return true;
                }
            }
            ChildIndex = GetNextNodeIndex(Ray.Txm, 1, Ray.Tym, 2, Ray.Tzm, 4);
            break;

        case 1:
            bChildHit = DoesRayIntersectOccludedNode(
                FOctreeRay(Ray.Txm, Ray.Tx1, Ray.Ty0, Ray.Tym, Ray.Tz0, Ray.Tzm),
                NewChildAddress, Data, RayState, OutHit, bCountAllOccludedVoxels);
            if (bChildHit)
            {
                bFoundAnyHit = true;
                if (!bCountAllOccludedVoxels)
                {
                    return true;
                }
            }
            ChildIndex = GetNextNodeIndex(Ray.Tx1, 8, Ray.Tym, 3, Ray.Tzm, 5);
            break;

        case 2:
            bChildHit = DoesRayIntersectOccludedNode(
                FOctreeRay(Ray.Tx0, Ray.Txm, Ray.Tym, Ray.Ty1, Ray.Tz0, Ray.Tzm),
                NewChildAddress, Data, RayState, OutHit, bCountAllOccludedVoxels);
            if (bChildHit)
            {
                bFoundAnyHit = true;
                if (!bCountAllOccludedVoxels)
                {
                    return true;
                }
            }
            ChildIndex = GetNextNodeIndex(Ray.Txm, 3, Ray.Ty1, 8, Ray.Tzm, 6);
            break;

        case 3:
            bChildHit = DoesRayIntersectOccludedNode(
                FOctreeRay(Ray.Txm, Ray.Tx1, Ray.Tym, Ray.Ty1, Ray.Tz0, Ray.Tzm),
                NewChildAddress, Data, RayState, OutHit, bCountAllOccludedVoxels);
            if (bChildHit)
            {
                bFoundAnyHit = true;
                if (!bCountAllOccludedVoxels)
                {
                    return true;
                }
            }
            ChildIndex = GetNextNodeIndex(Ray.Tx1, 8, Ray.Ty1, 8, Ray.Tzm, 7);
            break;

        case 4:
            bChildHit = DoesRayIntersectOccludedNode(
                FOctreeRay(Ray.Tx0, Ray.Txm, Ray.Ty0, Ray.Tym, Ray.Tzm, Ray.Tz1),
                NewChildAddress, Data, RayState, OutHit, bCountAllOccludedVoxels);
            if (bChildHit)
            {
                bFoundAnyHit = true;
                if (!bCountAllOccludedVoxels)
                {
                    return true;
                }
            }
            ChildIndex = GetNextNodeIndex(Ray.Txm, 5, Ray.Tym, 6, Ray.Tz1, 8);
            break;

        case 5:
            bChildHit = DoesRayIntersectOccludedNode(
                FOctreeRay(Ray.Txm, Ray.Tx1, Ray.Ty0, Ray.Tym, Ray.Tzm, Ray.Tz1),
                NewChildAddress, Data, RayState, OutHit, bCountAllOccludedVoxels);
            if (bChildHit)
            {
                bFoundAnyHit = true;
                if (!bCountAllOccludedVoxels)
                {
                    return true;
                }
            }
            ChildIndex = GetNextNodeIndex(Ray.Tx1, 8, Ray.Tym, 7, Ray.Tz1, 8);
            break;

        case 6:
            bChildHit = DoesRayIntersectOccludedNode(
                FOctreeRay(Ray.Tx0, Ray.Txm, Ray.Tym, Ray.Ty1, Ray.Tzm, Ray.Tz1),
                NewChildAddress, Data, RayState, OutHit, bCountAllOccludedVoxels);
            if (bChildHit)
            {
                bFoundAnyHit = true;
                if (!bCountAllOccludedVoxels)
                {
                    return true;
                }
            }
            ChildIndex = GetNextNodeIndex(Ray.Txm, 7, Ray.Ty1, 8, Ray.Tz1, 8);
            break;

        case 7:
            bChildHit = DoesRayIntersectOccludedNode(
                FOctreeRay(Ray.Txm, Ray.Tx1, Ray.Tym, Ray.Ty1, Ray.Tzm, Ray.Tz1),
                NewChildAddress, Data, RayState, OutHit, bCountAllOccludedVoxels);
            if (bChildHit)
            {
                bFoundAnyHit = true;
                if (!bCountAllOccludedVoxels)
                {
                    return true;
                }
            }
            ChildIndex = 8;
            break;
        default: break;
        }
    }
    while (ChildIndex < 8);

    if (CVarNav3DRaycasterVeryVerbose.GetValueOnAnyThread() != 0)
    {
        UE_LOG(LogNav3D, VeryVerbose, TEXT("Raycaster: Node %d - Child traversal complete, FoundAnyHit: %s"), 
               NodeAddress.NodeIndex, bFoundAnyHit ? TEXT("TRUE") : TEXT("FALSE"));
    }
    
    return bFoundAnyHit;
}

UNav3DRaycaster::FOctreeRay::FOctreeRay(
    const float Tx0, const float Tx1, const float Ty0, const float Ty1,
    const float Tz0, const float Tz1)
    : Tx0(Tx0), Tx1(Tx1), Txm(0.5f * (Tx0 + Tx1))
      , Ty0(Ty0), Ty1(Ty1), Tym(0.5f * (Ty0 + Ty1))
      , Tz0(Tz0), Tz1(Tz1), Tzm(0.5f * (Tz0 + Tz1))
{
}

bool UNav3DRaycaster::FOctreeRay::Intersects() const
{
    return FMath::Max3(Tx0, Ty0, Tz0) < FMath::Min3(Tx1, Ty1, Tz1);
}

bool UNav3DRaycaster::FOctreeRay::IsInRange(
    const float MaxSize) const
{
    return Tx1 >= 0.0f && Ty1 >= 0.0f && Tz1 >= 0.0f &&
        Tx0 <= MaxSize && Ty0 <= MaxSize && Tz0 <= MaxSize;
}

uint8 UNav3DRaycaster::GetFirstNodeIndex(const FOctreeRay& Ray)
{
    uint8 Answer = 0;

    if (Ray.Tx0 > Ray.Ty0)
    {
        if (Ray.Tx0 > Ray.Tz0)
        {
            if (Ray.Ty1 < Ray.Tx0)
            {
                Answer |= 2;
            }
            if (Ray.Tz1 < Ray.Tx0)
            {
                Answer |= 4;
            }
            return Answer;
        }
    }
    else
    {
        if (Ray.Ty0 > Ray.Tz0)
        {
            if (Ray.Tx1 < Ray.Ty0)
            {
                Answer |= 1;
            }
            if (Ray.Tz1 < Ray.Ty0)
            {
                Answer |= 4;
            }
            return Answer;
        }
    }

    if (Ray.Tx1 < Ray.Tz0)
    {
        Answer |= 1;
    }
    if (Ray.Ty1 < Ray.Tz0)
    {
        Answer |= 2;
    }
    return Answer;
}

uint8 UNav3DRaycaster::GetNextNodeIndex(
    const float Txm, const int32 X, const float Tym, const int32 Y,
    const float Tzm, const int32 Z)
{
    if (Txm < Tym)
    {
        if (Txm < Tzm)
        {
            return X;
        }
    }
    else
    {
        if (Tym < Tzm)
        {
            return Y;
        }
    }
    return Z;
}

FVector UNav3DRaycaster::CalculateImpactNormal(
    const FVector& ImpactPoint,
    const FVector& NodeCenter)
{
    FVector Normal = FVector::ZeroVector;
    const FVector DirectionToCenter = (NodeCenter - ImpactPoint).GetAbs();

    if (DirectionToCenter.X >= DirectionToCenter.Y && DirectionToCenter.X >= DirectionToCenter.Z)
    {
        Normal.X = ImpactPoint.X > NodeCenter.X ? 1.0f : -1.0f;
    }
    else if (DirectionToCenter.Y >= DirectionToCenter.X && DirectionToCenter.Y >= DirectionToCenter.Z)
    {
        Normal.Y = ImpactPoint.Y > NodeCenter.Y ? 1.0f : -1.0f;
    }
    else
    {
        Normal.Z = ImpactPoint.Z > NodeCenter.Z ? 1.0f : -1.0f;
    }

    return Normal;
}