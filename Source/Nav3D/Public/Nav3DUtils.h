#pragma once

#include "Nav3DTypes.h"
#include <CoreMinimal.h>
#include <GraphAStar.h>

class UNav3DDataChunk;

class NAV3D_API FNav3DUtils
{
public:
	static MortonCode GetMortonCodeFromVector(const FVector& Vector);
	static MortonCode GetMortonCodeFromVector(const FIntVector& Vector);
	static FVector GetVectorFromMortonCode(const MortonCode MortonCode);
	static MortonCode GetParentMortonCode(const MortonCode ChildMortonCode);
	static MortonCode GetFirstChildMortonCode(const MortonCode ParentMortonCode);
	static FVector GetSubNodeOffset(SubNodeIndex SubIdx, float NodeExtent);
	static ENavigationQueryResult::Type GraphAStarResultToNavigationTypeResult(const EGraphAStarResult Result);
	static bool RayBoxIntersection(const FBox& Box, const FVector& RayOrigin, const FVector& RayDir, float RayLength,
	                               float& OutTMin, float& OutTMax);
	static FNavAgentProperties GetNavAgentPropsFromQuerier(const UObject* Querier);

	// Boundary voxel extraction for a chunk (populates Morton-coded boundary voxels)
	static void IdentifyBoundaryVoxels(UNav3DDataChunk* Chunk);

	// Build adjacency between two chunks by proximity of boundary voxels
	static void BuildAdjacencyBetweenChunks(UNav3DDataChunk* ChunkA, UNav3DDataChunk* ChunkB, float VoxelSize, float ConnectionThresholdMultiplier = 1.5f);

	// Utilities for adjacency workflow
	static FBox ComputeChunkBounds(const UNav3DDataChunk* Chunk);
	static bool AreChunksAdjacent(const UNav3DDataChunk* ChunkA, const UNav3DDataChunk* ChunkB, float Threshold);
	static float GetChunkLeafNodeSize(const UNav3DDataChunk* Chunk);
	static void BuildAdjacencyForChunk(UNav3DDataChunk* Chunk, const TArray<UNav3DDataChunk*>& OtherChunks, float VoxelSize, float ConnectionThresholdMultiplier = 1.5f);

	// Navigation data access
	static class ANav3DData* GetNav3DData(const UWorld* World);

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
		FEndpointProjectionResult(bool bInSuccess, const FVector& InPosition, const FNav3DNodeAddress& InAddress, LayerIndex InLayer, const FString& InReason = TEXT(""))
			: bSuccess(bInSuccess), ProjectedPosition(InPosition), NodeAddress(InAddress), ResolvedLayer(InLayer), FailureReason(InReason) {}
	};

	// Cross-volume adjacency utilities (used by unified cross-volume graph)
	static TArray<struct FNav3DEdgeVoxel> ExtractCrossVolumeBoundaryVoxels(class UNav3DDataChunk* Chunk, int32 ChunkIndex);
	static FVector GetVoxelWorldPosition(const struct FNav3DVoxelID& VoxelID, const TArray<class ANav3DDataChunkActor*>& ChunkActors);
	static float GetVoxelExtent(const struct FNav3DVoxelID& VoxelID, const TArray<class ANav3DDataChunkActor*>& ChunkActors);
	static FBox GetVoxelWorldBounds(const struct FNav3DVoxelID& VoxelID, const TArray<class ANav3DDataChunkActor*>& ChunkActors);
	static bool AreFaceAdjacent(const struct FNav3DVoxelID& A, const struct FNav3DVoxelID& B, const TArray<class ANav3DDataChunkActor*>& ChunkActors);
	static FVector CalculateSharedFacePortal(const struct FNav3DVoxelID& A, const struct FNav3DVoxelID& B, const TArray<class ANav3DDataChunkActor*>& ChunkActors);

	// Find best cross-actor portal between two chunk actors by scanning free boundary voxels
	static bool FindCrossActorBoundaryPortal(const class ANav3DDataChunkActor* FromActor,
		const class ANav3DDataChunkActor* ToActor,
		FVector& OutLocalPortal,
		FVector& OutRemotePortal,
		FVector& OutPortalLocation);

	// Project an arbitrary point to the nearest free voxel within a specific volume
	static FEndpointProjectionResult ProjectPointToFreeVoxel(
		const FNav3DVolumeNavigationData& VolumeData,
		const FVector& InputPosition,
		const FNavAgentProperties& AgentProperties,
		LayerIndex MinLayerIndex = 0,
		float MaxSearchRadius = 1000.0f,
		int32 MaxSearchIterations = 10);

	// Project a portal position to a guaranteed free voxel with layer fallback
	static FEndpointProjectionResult ProjectPortalToFreeVoxel(
		const FNav3DVolumeNavigationData& VolumeData,
		const FNav3DVoxelConnection& Connection,
		bool bUseLocal,
		const FNavAgentProperties& AgentProperties,
		LayerIndex MinLayerIndex = 0);

	// Project boundary points to navigable positions within volume bounds
	static FEndpointProjectionResult ProjectBoundaryToNavigable(
		const FNav3DVolumeNavigationData& VolumeData,
		const FVector& BoundaryPosition,
		const FNavAgentProperties& AgentProperties,
		LayerIndex MinLayerIndex = 0);

	// Validate and sanitize portal connections
	static bool ValidatePortalConnection(
		const FNav3DVoxelConnection& Connection,
		const FNav3DVolumeNavigationData& LocalVolume,
		const FNav3DVolumeNavigationData& RemoteVolume,
		FString& OutValidationError);

	// Find nearest free voxel within a bounded search area
	static FEndpointProjectionResult FindNearestFreeVoxel(
		const FNav3DVolumeNavigationData& VolumeData,
		const FVector& SearchCenter,
		float SearchRadius,
		const FNavAgentProperties& AgentProperties,
		LayerIndex MinLayerIndex = 0,
		int32 MaxSearchIterations = 20);
};
