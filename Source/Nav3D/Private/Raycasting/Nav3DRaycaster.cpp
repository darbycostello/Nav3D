#include "Raycasting/Nav3DRaycaster.h"
#include "Nav3DUtils.h"
#include "Nav3DVolumeNavigationData.h"
#include <DrawDebugHelpers.h>

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
    const auto Result = DoesRayIntersectOccludedNode(
        OctreeRay,
        FNav3DNodeAddress(VolumeNavigationData.GetData().GetLayerCount() - 1, 0),
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
    if (!Ray.IsInRange(RayState.RaySize))
    {
        return false;
    }

    // If this is a layer 0 node, test for actual intersection
    if (NodeAddress.LayerIndex == 0)
    {
        return DoesRayIntersectOccludedLeaf(Ray, NodeAddress, Data, RayState, OutHit, bCountAllOccludedVoxels);
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
    const auto& LeafNodes = Data.GetData().GetLeafNodes();
    if (!LeafNodes.GetLeafNodes().IsValidIndex(NodeAddress.NodeIndex))
    {
        return false;
    }

    const auto& LeafNode = LeafNodes.GetLeafNode(NodeAddress.NodeIndex);
    if (LeafNode.IsCompletelyFree())
    {
        return false;
    }

    // Get node bounds for intersection test
    const FVector NodePos = Data.GetNodePositionFromAddress(NodeAddress, true);
    const float NodeExtent = Data.GetNodeExtentFromNodeAddress(NodeAddress);
    const FBox NodeBox = FBox::BuildAABB(NodePos, FVector(NodeExtent));

    // Test intersection with node bounds first
    float TMin, TMax;
    if (!FNav3DUtils::RayBoxIntersection(NodeBox, RayState.RayOrigin, RayState.RayDirection, RayState.RaySize, TMin,
                                         TMax))
    {
        return false;
    }

    // If leaf node is completely occluded, it counts as a single voxel hit
    if (LeafNode.IsCompletelyOccluded())
    {
        // Increment occluded voxel count
        OutHit.OccludedVoxelCount++;
        
        // Record hit information if this is the first hit we've found
        if (!OutHit.bBlockingHit || TMin < OutHit.Distance)
        {
            OutHit.ImpactPoint = RayState.RayOrigin + RayState.RayDirection * TMin;
            OutHit.ImpactNormal = CalculateImpactNormal(OutHit.ImpactPoint, NodePos);
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
    bool bHit = false;
    float ClosestHit = MAX_flt;
    FNav3DNodeAddress ClosestSubNode = NodeAddress;

    // Test each sub-node
    for (SubNodeIndex SubIdx = 0; SubIdx < 64; SubIdx++)
    {
        if (!LeafNode.IsSubNodeOccluded(SubIdx))
        {
            continue;
        }

        // Calculate sub-node position and bounds
        const auto SubNodePos = NodePos + FNav3DUtils::GetSubNodeOffset(SubIdx, NodeExtent);
        const float SubNodeExtent = NodeExtent * 0.25f;
        const FBox SubNodeBox = FBox::BuildAABB(SubNodePos, FVector(SubNodeExtent));

        float SubTMin, SubTMax;
        if (FNav3DUtils::RayBoxIntersection(
            SubNodeBox, RayState.RayOrigin, RayState.RayDirection, RayState.RaySize, SubTMin, SubTMax))
        {
            // Increment occluded voxel count for each hit subnode
            OutHit.OccludedVoxelCount++;
            
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
                    OutHit.ImpactPoint = RayState.RayOrigin + RayState.RayDirection * SubTMin;
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
    // Validate layer and node indices
    const auto& NavData = Data.GetData();
    if (NodeAddress.LayerIndex >= NavData.GetLayerCount())
    {
        return false;
    }

    const auto& Layer = NavData.GetLayer(NodeAddress.LayerIndex);
    if (!Layer.GetNodes().IsValidIndex(NodeAddress.NodeIndex))
    {
        return false;
    }

    const auto& Node = Data.GetData()
                           .GetLayer(NodeAddress.LayerIndex)
                           .GetNode(NodeAddress.NodeIndex);

    if (!Node.HasChildren())
    {
        return false;
    }

    // Get node bounds for intersection test
    const FVector NodePos = Data.GetNodePositionFromAddress(NodeAddress, true);
    const float NodeExtent = Data.GetNodeExtentFromNodeAddress(NodeAddress);
    const FBox NodeBox = FBox::BuildAABB(NodePos, FVector(NodeExtent));

    // Test intersection with node bounds first
    float TMin, TMax;
    if (!FNav3DUtils::RayBoxIntersection(NodeBox, RayState.RayOrigin, RayState.RayDirection, RayState.RaySize, TMin, TMax))
    {
        return false;
    }

    const auto& FirstChildAddress = Node.FirstChild;
    if (!FirstChildAddress.IsValid())
    {
        return false;
    }

    // Start with the first child node in the traversal
    uint8 ChildIndex = GetFirstNodeIndex(Ray);
    bool bFoundAnyHit = false;

    do
    {
        // Create the child address with reflected index based on ray direction
        const int32 ReflectedChildNodeIndex = ChildIndex ^ RayState.A;
        const FNav3DNodeAddress NewChildAddress(FirstChildAddress.LayerIndex, 
                                               FirstChildAddress.NodeIndex + ReflectedChildNodeIndex);

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