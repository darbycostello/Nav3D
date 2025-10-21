#pragma once

#include "CoreMinimal.h"
#include "Nav3DTypes.h"
#include "Nav3DMultiChunkConeCaster.generated.h"

class ANav3DData;
class ANav3DDataChunkActor;

/**
 * Parameters for cone casting in Nav3D
 */
USTRUCT(BlueprintType)
struct NAV3D_API FNav3DConeCastParams
{
	GENERATED_BODY()

	/** Origin point of the cone */
	UPROPERTY(BlueprintReadWrite, Category = "ConeCast")
	FVector Origin = FVector::ZeroVector;

	/** Direction the cone is pointing */
	UPROPERTY(BlueprintReadWrite, Category = "ConeCast")
	FVector Direction = FVector::ForwardVector;

	/** Cone angle in degrees (full angle, not half) */
	UPROPERTY(BlueprintReadWrite, Category = "ConeCast")
	float ConeAngleDeg = 120.0f;

	/** Maximum distance to search */
	UPROPERTY(BlueprintReadWrite, Category = "ConeCast")
	float MaxDistance = 3000.0f;

	/** Target octree layer to sample (1 = Layer 1, larger voxels, faster) */
	LayerIndex TargetLayer = 1;

	/** Maximum number of occluded voxels to return (for performance) */
	UPROPERTY(BlueprintReadWrite, Category = "ConeCast")
	int32 MaxResults = 32;
};

/**
 * Result structure for an occluded voxel found by cone cast
 */
USTRUCT(BlueprintType)
struct NAV3D_API FNav3DOccludedVoxel
{
	GENERATED_BODY()

	/** World position of the voxel center */
	UPROPERTY(BlueprintReadOnly, Category = "ConeCast")
	FVector Position = FVector::ZeroVector;

	/** Size of the voxel (extent) */
	UPROPERTY(BlueprintReadOnly, Category = "ConeCast")
	float VoxelSize = 0.0f;

	/** Navigation node address */
	FNav3DNodeAddress Address;

	FNav3DOccludedVoxel() = default;

	FNav3DOccludedVoxel(const FVector& InPosition, float InVoxelSize, const FNav3DNodeAddress& InAddress)
		: Position(InPosition), VoxelSize(InVoxelSize), Address(InAddress)
	{
	}
};

/**
 * Multi-chunk cone caster for finding occluded voxels within a cone frustum
 * Uses octree traversal for efficient spatial queries across chunk boundaries
 */
UCLASS(NotBlueprintable)
class NAV3D_API UNav3DMultiChunkConeCaster : public UObject
{
	GENERATED_BODY()

public:
	/**
	 * Find all occluded voxels within a cone across multiple chunks
	 * Uses hierarchical octree traversal with frustum culling for efficiency
	 * 
	 * @param Nav3DData The navigation data containing all chunks
	 * @param Params Cone cast parameters
	 * @param OutOccludedVoxels Array to fill with occluded voxel positions
	 * @return Number of occluded voxels found
	 */
	static int32 FindOccludedVoxelsInCone(
		const ANav3DData* Nav3DData,
		const FNav3DConeCastParams& Params,
		TArray<FNav3DOccludedVoxel>& OutOccludedVoxels);

private:
	struct FChunkConeSegment
	{
		ANav3DDataChunkActor* ChunkActor;
		FVector SegmentStart;
		FVector SegmentEnd;
	};

	/**
	 * Build segments for each chunk that intersects with the cone
	 * @param Nav3DData The navigation data
	 * @param Params Cone parameters
	 * @param OutSegments Array to fill with chunk segments
	 * @return true if segments were built successfully
	 */
	static bool BuildChunkSegments(
		const ANav3DData* Nav3DData,
		const FNav3DConeCastParams& Params,
		TArray<FChunkConeSegment>& OutSegments);

	/**
	 * Traverse octree nodes within cone frustum in a single chunk
	 * @param Segment The chunk segment to process
	 * @param Params Cone parameters
	 * @param OutOccludedVoxels Array to add found voxels to
	 */
	static void TraverseConeInChunk(
		const FChunkConeSegment& Segment,
		const FNav3DConeCastParams& Params,
		TArray<FNav3DOccludedVoxel>& OutOccludedVoxels);

	/**
	 * Recursively traverse octree nodes with frustum culling
	 * @param VolumeData The volume navigation data
	 * @param NodeAddress Current node address
	 * @param Params Cone parameters
	 * @param OutOccludedVoxels Array to add found voxels to
	 */
	static void TraverseNodeRecursive(
		const FNav3DVolumeNavigationData* VolumeData,
		const FNav3DNodeAddress& NodeAddress,
		const FNav3DConeCastParams& Params,
		TArray<FNav3DOccludedVoxel>& OutOccludedVoxels);

	/**
	 * Check if an AABB intersects with a cone frustum
	 * @param Box The bounding box to test
	 * @param ConeOrigin Origin of the cone
	 * @param ConeDirection Direction of the cone (normalized)
	 * @param ConeAngleDeg Full cone angle in degrees
	 * @param MaxDistance Maximum cone distance
	 * @return true if AABB intersects the cone
	 */
	static bool AABBIntersectsCone(
		const FBox& Box,
		const FVector& ConeOrigin,
		const FVector& ConeDirection,
		float ConeAngleDeg,
		float MaxDistance);

	/**
	 * Check if a sphere intersects with a cone
	 * @param SphereCenter Center of the sphere
	 * @param SphereRadius Radius of the sphere
	 * @param ConeOrigin Origin of the cone
	 * @param ConeDirection Direction of the cone (normalized)
	 * @param CosHalfAngle Cosine of half the cone angle
	 * @param MaxDistance Maximum cone distance
	 * @return true if sphere intersects the cone
	 */
	static bool SphereIntersectsCone(
		const FVector& SphereCenter,
		float SphereRadius,
		const FVector& ConeOrigin,
		const FVector& ConeDirection,
		float CosHalfAngle,
		float MaxDistance);
};

