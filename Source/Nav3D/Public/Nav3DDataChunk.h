#pragma once

#include "Nav3DTypes.h"
#include "Nav3DVolumeNavigationData.h"
#include <AI/Navigation/NavigationDataChunk.h>
#include <CoreMinimal.h>
#include "Nav3DDataChunk.generated.h"

USTRUCT()
struct NAV3D_API FNav3DEdgeVoxel
{
	GENERATED_BODY()

	MortonCode Morton;
	int32 VolumeIndex = 0;
	LayerIndex LayerIndex = 0;  // Layer this boundary voxel belongs to
	TArray<MortonCode> AdjacentChunkVoxels;
	uint8 bIsNavigable : 1;
	
	// Boundary face flags (bitfield)
	uint8 bOnMinXFace : 1;  // -X face
	uint8 bOnMaxXFace : 1;  // +X face
	uint8 bOnMinYFace : 1;  // -Y face
	uint8 bOnMaxYFace : 1;  // +Y face
	uint8 bOnMinZFace : 1;  // -Z face
	uint8 bOnMaxZFace : 1;  // +Z face
};

UCLASS()
class NAV3D_API UNav3DDataChunk final : public UNavigationDataChunk
{
	GENERATED_BODY()

public:
	virtual void Serialize(FArchive& Archive) override;
	void AddNavigationData(FNav3DVolumeNavigationData& NavData);
	void ReleaseNavigationData();
	const FNav3DVolumeNavigationData* GetVolumeNavigationData() const;
	
	// Get bounds of this chunk
	FBox GetBounds() const;

	// Octree navigation data blocks belonging to this chunk
	TArray<FNav3DVolumeNavigationData> NavigationData;

	// Precomputed boundary voxels (Morton-coded)
	TArray<FNav3DEdgeVoxel> BoundaryVoxels;

	// Transient lookup for quick Morton->index mapping; rebuilt on load
	TMap<MortonCode, int32> MortonToBoundaryIndex;
};
