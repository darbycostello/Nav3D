#pragma once

#include "CoreMinimal.h"
#include "Nav3DTypes.h"

class ANav3DDataChunkActor;

// Identifier of a voxel across volumes/chunks within a tactical region.
// NOTE: ChunkIndex here refers to the index of the chunk ACTOR within the
// array passed to BuildGraph (CachedChunkActors). It is NOT the index of
// UNav3DDataChunk inside an actor.
struct FNav3DVoxelID
{
    int32 VolumeIndex = INDEX_NONE;
    int32 ChunkIndex = INDEX_NONE;
    LayerIndex Layer = 0;
    MortonCode Morton = 0;

    bool operator==(const FNav3DVoxelID& Other) const
    {
        return VolumeIndex == Other.VolumeIndex &&
               ChunkIndex == Other.ChunkIndex &&
               Layer == Other.Layer &&
               Morton == Other.Morton;
    }
};

FORCEINLINE uint32 GetTypeHash(const FNav3DVoxelID& VoxelID)
{
    return HashCombine(
        HashCombine(GetTypeHash(VoxelID.VolumeIndex), GetTypeHash(VoxelID.ChunkIndex)),
        HashCombine(GetTypeHash(VoxelID.Layer), GetTypeHash((uint64)VoxelID.Morton))
    );
}

struct FNav3DCrossVolumeConnection
{
    FNav3DVoxelID RemoteVoxel;
    FVector PortalLocation = FVector::ZeroVector;
    float ConnectionCost = 0.0f;

    FNav3DCrossVolumeConnection() = default;
    FNav3DCrossVolumeConnection(const FNav3DVoxelID& InRemoteVoxel, const FVector& InPortalLocation, float InCost)
        : RemoteVoxel(InRemoteVoxel), PortalLocation(InPortalLocation), ConnectionCost(InCost)
    {}
};

class FNav3DCrossVolumeGraph
{
public:
    void Serialize(FArchive& Ar);
    void BuildGraph(const TArray<ANav3DDataChunkActor*>& ChunkActors);
    void RebuildForChunk(const ANav3DDataChunkActor* ModifiedChunk);
    void GetNeighbors(const FNav3DVoxelID& VoxelID, TArray<FNav3DCrossVolumeConnection>& OutConnections) const;
    void Clear();
#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST
    int32 GetConnectionCount() const;
    void LogGraphStatistics() const;
#endif

    // Utility accessors
    int32 FindChunkIndexForPosition(const FVector& Position) const;
    const TArray<ANav3DDataChunkActor*>& GetCachedChunkActors() const { return CachedChunkActors; }
    const TMap<int64, TArray<FNav3DVoxelID>>& GetBuckets() const { return Buckets; }

    // Persistence helpers (used by tactical actor serialization)
    void SetCachedChunkActors(const TArray<ANav3DDataChunkActor*>& InActors);
    void SetBuckets(const TMap<int64, TArray<FNav3DVoxelID>>& InBuckets);

private:
    // On-demand adjacency cache (populated per-voxel when queried)
    mutable TMap<FNav3DVoxelID, TArray<FNav3DCrossVolumeConnection>> Adjacency;

    // Cached input actors used to interpret FNav3DVoxelID::ChunkIndex
    TArray<ANav3DDataChunkActor*> CachedChunkActors;

    // Face-plane buckets to accelerate on-demand neighbor queries
    TMap<int64, TArray<FNav3DVoxelID>> Buckets;
    static int64 MakeFaceKey(int32 Axis, float Pos);
};

FORCEINLINE FArchive& operator<<(FArchive& Ar, FNav3DVoxelID& V)
{
    Ar << V.VolumeIndex;
    Ar << V.ChunkIndex;
    Ar << V.Layer;
    Ar << V.Morton;
    return Ar;
}


