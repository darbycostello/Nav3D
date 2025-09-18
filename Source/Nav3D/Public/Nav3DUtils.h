#pragma once

#include "Nav3DTypes.h"
#include <CoreMinimal.h>
#include <GraphAStar.h>
#include "Nav3DDataChunk.h"
#include "Pathfinding/Search/Nav3DQueryFilter.h"

class ANav3DData;
class UNav3DDataChunk;

class NAV3D_API FNav3DUtils
{
public:
	static MortonCode GetMortonCodeFromVector(const FVector& Vector);
	static MortonCode GetMortonCodeFromIntVector(const FIntVector& Vector);
	static FVector GetVectorFromMortonCode(const MortonCode MortonCode);
	static FIntVector GetIntVectorFromMortonCode(const MortonCode MortonCode);
	static MortonCode GetParentMortonCode(const MortonCode ChildMortonCode);
	static MortonCode GetFirstChildMortonCode(const MortonCode ParentMortonCode);
	static FVector GetSubNodeOffset(SubNodeIndex SubIdx, float NodeExtent);
	static ENavigationQueryResult::Type GraphAStarResultToNavigationTypeResult(const EGraphAStarResult Result);
	static bool RayBoxIntersection(const FBox& Box, const FVector& RayOrigin, const FVector& RayDir, float RayLength,
	                               float& OutTMin, float& OutTMax);
	static FNavAgentProperties GetNavAgentPropsFromQuerier(const UObject* Querier);

	// Boundary voxel extraction for a chunk (populates Morton-coded boundary voxels)
	static void IdentifyBoundaryVoxels(UNav3DDataChunk* Chunk);
	static float GetDefaultVoxelSize(const ANav3DData* NavData);
	// Build adjacency between two chunks by proximity of boundary voxels
	static void BuildAdjacencyBetweenChunks(UNav3DDataChunk* ChunkA, UNav3DDataChunk* ChunkB, float VoxelSize,
	                                        float ConnectionThresholdMultiplier = 1.5f);

	// Utilities for adjacency workflow
	static FBox ComputeChunkBounds(const UNav3DDataChunk* Chunk);
	static bool AreChunksAdjacent(const UNav3DDataChunk* ChunkA, const UNav3DDataChunk* ChunkB, float Threshold);
	static float GetChunkLeafNodeSize(const UNav3DDataChunk* Chunk);
	static void BuildAdjacencyForChunk(UNav3DDataChunk* Chunk, const TArray<UNav3DDataChunk*>& OtherChunks,
	                                   float VoxelSize, float ConnectionThresholdMultiplier = 1.5f);

	// Navigation data access
	static ANav3DData* GetNav3DData(const UWorld* World);

	// Shared color palette utilities
	static FLinearColor GetChunkColorByIndex(int32 ChunkIndex);


	// Endpoint projection utilities for cross-volume pathfinding
	struct FEndpointProjectionResult
	{
		bool bSuccess = false;
		FVector ProjectedPosition;
		FNav3DNodeAddress NodeAddress;
		LayerIndex ResolvedLayer = 0;
		FString FailureReason;

		FEndpointProjectionResult() = default;

		FEndpointProjectionResult(bool bInSuccess, const FVector& InPosition, const FNav3DNodeAddress& InAddress,
		                          LayerIndex InLayer, const FString& InReason = TEXT(""))
			: bSuccess(bInSuccess), ProjectedPosition(InPosition), NodeAddress(InAddress), ResolvedLayer(InLayer),
			  FailureReason(InReason)
		{
		}
	};

	static float GetMaxSearchDistance();

	// Validate and sanitize portal connections
	static bool ValidatePortalConnection(
		const FNav3DVoxelConnection& Connection,
		const FNav3DVolumeNavigationData& LocalVolume,
		const FNav3DVolumeNavigationData& RemoteVolume,
		FString& OutValidationError);

	static bool CheckVoxelFaceAdjacency(
		const FNav3DEdgeVoxel& VoxelA,
		const FNav3DEdgeVoxel& VoxelB,
		const FNav3DVolumeNavigationData* VolumeA,
		const FNav3DVolumeNavigationData* VolumeB,
		uint8 FaceA,
		uint8 FaceB,
		float AdjacencyClearance);

	static FSharedConstNavQueryFilter GetNav3DQueryFilter(
		const ANav3DData* Nav3DData,
		const TSubclassOf<UNavigationQueryFilter>& NavigationQueryFilter,
		const UObject* Querier);

	static bool IsNodeFreeSpace(
		const FNav3DVolumeNavigationData& VolumeData,
		const FNav3DNodeAddress& NodeAddress);
};
