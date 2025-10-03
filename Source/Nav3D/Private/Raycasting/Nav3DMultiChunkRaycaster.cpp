#include "Raycasting/Nav3DMultiChunkRaycaster.h"

#include "Nav3D.h"
#include "Nav3DData.h"
#include "Nav3DDataChunkActor.h"
#include "Nav3DVolumeNavigationData.h"

UNav3DMultiChunkRaycaster::UNav3DMultiChunkRaycaster()
{
}

bool UNav3DMultiChunkRaycaster::HasLineOfTraversal(
    const ANav3DData* Nav3DData,
    const FVector& From,
    const FVector& To,
    float AgentRadius,
    FNav3DRaycastHit& OutHit)
{
    if (!Nav3DData)
    {
        UE_LOG(LogNav3D, Verbose, TEXT("HasLineOfTraversal: Failed - No Nav3DData provided"));
        return false;
    }

    UE_LOG(LogNav3D, Verbose, TEXT("HasLineOfTraversal: Checking traversal from %s to %s (AgentRadius=%.2f)"), 
        *From.ToString(), *To.ToString(), AgentRadius);

    TArray<FChunkRaySegment> Segments;
    if (!BuildChunkSegments(Nav3DData, From, To, Segments))
    {
        UE_LOG(LogNav3D, Verbose, TEXT("HasLineOfTraversal: Failed - No intersecting chunks found"));
        return false;
    }

    UE_LOG(LogNav3D, Verbose, TEXT("HasLineOfTraversal: Found %d intersecting chunks"), Segments.Num());

    // Test each segment with 5-ray corridor
    for (int32 i = 0; i < Segments.Num(); ++i)
    {
        const FChunkRaySegment& Segment = Segments[i];
        UE_LOG(LogNav3D, Verbose, TEXT("HasLineOfTraversal: Testing segment %d/%d in chunk %s"), 
            i + 1, Segments.Num(), 
            Segment.ChunkActor ? *Segment.ChunkActor->GetName() : TEXT("NULL"));
        
        if (!TraceCorridorInChunk(Segment, AgentRadius, OutHit))
        {
            UE_LOG(LogNav3D, Verbose, TEXT("HasLineOfTraversal: Failed - Corridor blocked in segment %d"), i + 1);
            return false;
        }
    }

    UE_LOG(LogNav3D, Verbose, TEXT("HasLineOfTraversal: SUCCESS - All segments clear"));
    return true;
}

bool UNav3DMultiChunkRaycaster::BuildChunkSegments(
    const ANav3DData* Nav3DData,
    const FVector& From,
    const FVector& To,
    TArray<FChunkRaySegment>& OutSegments)
{
    OutSegments.Reset();

    TArray<ANav3DDataChunkActor*> AllChunks = Nav3DData->GetAllChunkActors();
    const FVector RayDirection = (To - From).GetSafeNormal();
    const float RayLength = FVector::Dist(From, To);

    UE_LOG(LogNav3D, Verbose, TEXT("BuildChunkSegments: Checking %d chunks for intersection"), AllChunks.Num());

    for (ANav3DDataChunkActor* ChunkActor : AllChunks)
    {
        if (!ChunkActor) continue;

        FVector IntersectStart, IntersectEnd;
        if (RayIntersectsBox(From, RayDirection, RayLength, 
            ChunkActor->DataChunkActorBounds, IntersectStart, IntersectEnd))
        {
            FChunkRaySegment Segment;
            Segment.ChunkActor = ChunkActor;
            Segment.SegmentStart = FMath::ClosestPointOnSegment(IntersectStart, From, To);
            Segment.SegmentEnd = FMath::ClosestPointOnSegment(IntersectEnd, From, To);
            
            if (!Segment.SegmentStart.Equals(Segment.SegmentEnd, KINDA_SMALL_NUMBER))
            {
                UE_LOG(LogNav3D, Verbose, TEXT("BuildChunkSegments: Added segment for chunk %s (%.2f to %.2f)"), 
                    *ChunkActor->GetName(), 
                    FVector::Dist(From, Segment.SegmentStart),
                    FVector::Dist(From, Segment.SegmentEnd));
                OutSegments.Add(Segment);
            }
        }
    }

    OutSegments.Sort([&From](const FChunkRaySegment& A, const FChunkRaySegment& B)
    {
        return FVector::DistSquared(From, A.SegmentStart) < FVector::DistSquared(From, B.SegmentStart);
    });

    UE_LOG(LogNav3D, Verbose, TEXT("BuildChunkSegments: Found %d intersecting segments"), OutSegments.Num());
    return OutSegments.Num() > 0;
}

bool UNav3DMultiChunkRaycaster::TraceCorridorInChunk(
    const FChunkRaySegment& Segment,
    float AgentRadius,
    FNav3DRaycastHit& OutHit)
{
    const FNav3DVolumeNavigationData* VolumeData = nullptr;
    if (Segment.ChunkActor && Segment.ChunkActor->Nav3DChunks.Num() > 0)
    {
        VolumeData = Segment.ChunkActor->Nav3DChunks[0]->GetVolumeNavigationData();
    }
    if (!VolumeData) 
    {
        UE_LOG(LogNav3D, Verbose, TEXT("TraceCorridorInChunk: Failed - No volume data for chunk %s"), 
            Segment.ChunkActor ? *Segment.ChunkActor->GetName() : TEXT("NULL"));
        return false;
    }

    UE_LOG(LogNav3D, Verbose, TEXT("TraceCorridorInChunk: Testing 5-ray corridor in chunk %s (AgentRadius=%.2f)"), 
        *Segment.ChunkActor->GetName(), AgentRadius);

    UNav3DRaycaster* Raycaster = NewObject<UNav3DRaycaster>();

    // Center ray
    UE_LOG(LogNav3D, Verbose, TEXT("TraceCorridorInChunk: Testing center ray"));
    if (Raycaster->Trace(*VolumeData, Segment.SegmentStart, Segment.SegmentEnd, OutHit))
    {
        UE_LOG(LogNav3D, Verbose, TEXT("TraceCorridorInChunk: Center ray blocked at distance %.2f"), OutHit.Distance);
        return false;
    }

    // Corner rays
    FVector Direction = (Segment.SegmentEnd - Segment.SegmentStart).GetSafeNormal();
    FVector Right = FVector::CrossProduct(Direction, FVector::UpVector).GetSafeNormal();
    FVector Up = FVector::CrossProduct(Direction, Right).GetSafeNormal();

    TArray<FVector> CornerOffsets = {
        (Right + Up).GetSafeNormal() * AgentRadius,
        (Right - Up).GetSafeNormal() * AgentRadius,
        (-Right + Up).GetSafeNormal() * AgentRadius,
        (-Right - Up).GetSafeNormal() * AgentRadius
    };

    UE_LOG(LogNav3D, Verbose, TEXT("TraceCorridorInChunk: Testing %d corner rays"), CornerOffsets.Num());

    for (int32 i = 0; i < CornerOffsets.Num(); ++i)
    {
        const FVector& Offset = CornerOffsets[i];
        FNav3DRaycastHit CornerHit;
        UE_LOG(LogNav3D, Verbose, TEXT("TraceCorridorInChunk: Testing corner ray %d (offset: %s)"), 
            i + 1, *Offset.ToString());
        
        if (Raycaster->Trace(*VolumeData, 
            Segment.SegmentStart + Offset, 
            Segment.SegmentEnd + Offset, 
            CornerHit))
        {
            UE_LOG(LogNav3D, Verbose, TEXT("TraceCorridorInChunk: Corner ray %d blocked at distance %.2f"), 
                i + 1, CornerHit.Distance);
            OutHit = CornerHit;
            return false;
        }
    }

    UE_LOG(LogNav3D, Verbose, TEXT("TraceCorridorInChunk: SUCCESS - All 5 rays clear"));
    return true;
}

bool UNav3DMultiChunkRaycaster::RayIntersectsBox(
    const FVector& RayOrigin,
    const FVector& RayDirection,
    float RayLength,
    const FBox& Box,
    FVector& OutIntersectStart,
    FVector& OutIntersectEnd)
{
    constexpr float BigNumber = 1e10f;
    const FVector InvDir(
        FMath::IsNearlyZero(RayDirection.X) ? BigNumber : 1.0f / RayDirection.X,
        FMath::IsNearlyZero(RayDirection.Y) ? BigNumber : 1.0f / RayDirection.Y,
        FMath::IsNearlyZero(RayDirection.Z) ? BigNumber : 1.0f / RayDirection.Z
    );

    const FVector t0 = (Box.Min - RayOrigin) * InvDir;
    const FVector t1 = (Box.Max - RayOrigin) * InvDir;

    const FVector tmin(FMath::Min(t0.X, t1.X), FMath::Min(t0.Y, t1.Y), FMath::Min(t0.Z, t1.Z));
    const FVector tmax(FMath::Max(t0.X, t1.X), FMath::Max(t0.Y, t1.Y), FMath::Max(t0.Z, t1.Z));

    const float tEnter = FMath::Max3(tmin.X, tmin.Y, tmin.Z);
    const float tExit = FMath::Min3(tmax.X, tmax.Y, tmax.Z);

    if (tEnter > tExit || tExit < 0.0f || tEnter > RayLength)
    {
        return false;
    }

    OutIntersectStart = RayOrigin + RayDirection * FMath::Max(0.0f, tEnter);
    OutIntersectEnd = RayOrigin + RayDirection * FMath::Min(RayLength, tExit);

    return true;
}
