#pragma once

#include "CoreMinimal.h"
#include "Nav3DTypes.h"
#include "Nav3DRaycaster.h"
#include "Nav3DMultiChunkRaycaster.generated.h"

class ANav3DData;
class ANav3DDataChunkActor;

UCLASS(NotBlueprintable, EditInlineNew)
class NAV3D_API UNav3DMultiChunkRaycaster : public UObject
{
    GENERATED_BODY()

public:
    UNav3DMultiChunkRaycaster();

    /**
     * Check if there's a clear line of traversal between two points across multiple chunks
     * @param Nav3DData The navigation data containing all chunks
     * @param From Starting position
     * @param To Ending position
     * @param AgentRadius Radius of the agent for corridor testing
     * @param OutHit Hit result if traversal is blocked
     * @return true if traversal is clear, false if blocked
     */
    static bool HasLineOfTraversal(
        const ANav3DData* Nav3DData,
        const FVector& From,
        const FVector& To,
        float AgentRadius,
        FNav3DRaycastHit& OutHit);

private:
    struct FChunkRaySegment
    {
        ANav3DDataChunkActor* ChunkActor;
        FVector SegmentStart;
        FVector SegmentEnd;
    };

    /**
     * Build segments for each chunk that intersects with the ray
     * @param Nav3DData The navigation data
     * @param From Starting position
     * @param To Ending position
     * @param OutSegments Array to fill with chunk segments
     * @return true if segments were built successfully
     */
    static bool BuildChunkSegments(
        const ANav3DData* Nav3DData,
        const FVector& From,
        const FVector& To,
        TArray<FChunkRaySegment>& OutSegments);

    /**
     * Trace a corridor within a single chunk using 5-ray pattern
     * @param Segment The chunk segment to test
     * @param AgentRadius Radius of the agent
     * @param OutHit Hit result if blocked
     * @return true if corridor is clear, false if blocked
     */
    static bool TraceCorridorInChunk(
        const FChunkRaySegment& Segment,
        float AgentRadius,
        FNav3DRaycastHit& OutHit);

    /**
     * Check if a ray intersects with a bounding box
     * @param RayOrigin Origin of the ray
     * @param RayDirection Direction of the ray (normalized)
     * @param RayLength Length of the ray
     * @param Box Bounding box to test against
     * @param OutIntersectStart Start of intersection segment
     * @param OutIntersectEnd End of intersection segment
     * @return true if ray intersects the box
     */
    static bool RayIntersectsBox(
        const FVector& RayOrigin,
        const FVector& RayDirection,
        float RayLength,
        const FBox& Box,
        FVector& OutIntersectStart,
        FVector& OutIntersectEnd);
};
