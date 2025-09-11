#include "Pathfinding/Nav3DCrossVolumeGraph.h"
#include "Nav3D.h"
#include "Nav3DDataChunkActor.h"
#include "Nav3DDataChunk.h"
#include "Nav3DUtils.h"

void FNav3DCrossVolumeGraph::Serialize(FArchive& Ar)
{
    // Serialize buckets and cached actors by name (actors will be resolved on load by owner)
    Ar << Buckets;
    // CachedChunkActors are not directly serializable here; owner actor should restore them.
    // Adjacency is on-demand and not serialized.
}

void FNav3DCrossVolumeGraph::Clear()
{
    Adjacency.Reset();
    CachedChunkActors.Reset();
    Buckets.Reset();
}

int32 FNav3DCrossVolumeGraph::GetConnectionCount() const
{
    int32 Count = 0;
    for (const auto& Pair : Adjacency)
    {
        Count += Pair.Value.Num();
    }
    return Count;
}

void FNav3DCrossVolumeGraph::LogGraphStatistics() const
{
#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST
    int32 BucketEntryCount = 0;
    for (const auto& Pair : Buckets)
    {
        BucketEntryCount += Pair.Value.Num();
    }
    UE_LOG(LogNav3D, Log, TEXT("CrossVolumeGraph: cache=%d voxels, connections=%d, buckets=%d keys, %d entries"),
           Adjacency.Num(), GetConnectionCount(), Buckets.Num(), BucketEntryCount);
#endif
}

int32 FNav3DCrossVolumeGraph::FindChunkIndexForPosition(const FVector& Position) const
{
    for (int32 i = 0; i < CachedChunkActors.Num(); ++i)
    {
        const ANav3DDataChunkActor* Actor = CachedChunkActors[i];
        if (Actor && Actor->DataChunkActorBounds.IsInside(Position))
        {
            return i;
        }
    }
    return INDEX_NONE;
}

void FNav3DCrossVolumeGraph::SetCachedChunkActors(const TArray<ANav3DDataChunkActor*>& InActors)
{
    CachedChunkActors = InActors;
}

void FNav3DCrossVolumeGraph::SetBuckets(const TMap<int64, TArray<FNav3DVoxelID>>& InBuckets)
{
    Buckets = InBuckets;
}

int64 FNav3DCrossVolumeGraph::MakeFaceKey(const int32 Axis, const float Pos)
{
    const int32 PosQ = FMath::RoundToInt(Pos * 100.0f);
    return (static_cast<int64>(Axis) << 32) ^ (static_cast<int64>(PosQ) & 0x00000000FFFFFFFFLL);
}

void FNav3DCrossVolumeGraph::BuildGraph(const TArray<ANav3DDataChunkActor*>& ChunkActors)
{
    Clear();
    CachedChunkActors = ChunkActors;

    TArray<FNav3DVoxelID> BoundaryIDs;
    BoundaryIDs.Reserve(1024);

    // Extract all boundary voxels from each actor/chunk (free-only handled in utility)
    for (int32 ChunkIdx = 0; ChunkIdx < ChunkActors.Num(); ++ChunkIdx)
    {
        ANav3DDataChunkActor* Actor = ChunkActors[ChunkIdx];
        if (!Actor) { continue; }
        if (Actor->Nav3DChunks.Num() == 0) { continue; }

        UNav3DDataChunk* Chunk = Actor->Nav3DChunks[0];
        if (!Chunk) { continue; }

        const TArray<FNav3DEdgeVoxel> Edges = FNav3DUtils::ExtractCrossVolumeBoundaryVoxels(Chunk, ChunkIdx);
        for (const FNav3DEdgeVoxel& E : Edges)
        {
            FNav3DVoxelID Id;
            Id.VolumeIndex = E.VolumeIndex;
            Id.ChunkIndex = ChunkIdx;
            Id.Layer = E.LayerIndex;
            Id.Morton = E.Morton;
            BoundaryIDs.Add(Id);
        }
    }

    UE_LOG(LogNav3D, Log, TEXT("FNav3DCrossVolumeGraph::BuildGraph: %d boundary voxels (free-only)"), BoundaryIDs.Num());

    // Populate face-plane buckets for on-demand queries
    Buckets.Reserve(BoundaryIDs.Num());
    for (const FNav3DVoxelID& Id : BoundaryIDs)
    {
        const FBox Box = FNav3DUtils::GetVoxelWorldBounds(Id, CachedChunkActors);
        Buckets.FindOrAdd(MakeFaceKey(0, Box.Min.X)).Add(Id);
        Buckets.FindOrAdd(MakeFaceKey(0, Box.Max.X)).Add(Id);
        Buckets.FindOrAdd(MakeFaceKey(1, Box.Min.Y)).Add(Id);
        Buckets.FindOrAdd(MakeFaceKey(1, Box.Max.Y)).Add(Id);
        Buckets.FindOrAdd(MakeFaceKey(2, Box.Min.Z)).Add(Id);
        Buckets.FindOrAdd(MakeFaceKey(2, Box.Max.Z)).Add(Id);
    }

    // Do not precompute connections; they will be computed on demand and cached
    LogGraphStatistics();
}

void FNav3DCrossVolumeGraph::RebuildForChunk(const ANav3DDataChunkActor* ModifiedChunk)
{
    if (!ModifiedChunk) { return; }
    // For simplicity, rebuild fully for now
    BuildGraph(CachedChunkActors);
}

void FNav3DCrossVolumeGraph::GetNeighbors(const FNav3DVoxelID& VoxelID, TArray<FNav3DCrossVolumeConnection>& OutConnections) const
{
    OutConnections.Reset();

    if (const TArray<FNav3DCrossVolumeConnection>* Cached = Adjacency.Find(VoxelID))
    {
        OutConnections = *Cached;
        return;
    }

    // Compute neighbors on-demand: cross-actor only, face-adjacent only, choose nearest per face plane
    TArray<FNav3DCrossVolumeConnection> Computed;
    const FBox Box = FNav3DUtils::GetVoxelWorldBounds(VoxelID, CachedChunkActors);

    const struct { int32 Axis; float Pos; } Faces[6] = {
        {0, static_cast<float>(Box.Min.X)}, {0, static_cast<float>(Box.Max.X)}, 
        {1, static_cast<float>(Box.Min.Y)}, {1, static_cast<float>(Box.Max.Y)}, 
        {2, static_cast<float>(Box.Min.Z)}, {2, static_cast<float>(Box.Max.Z)}
    };

    for (const auto& Face : Faces)
    {
        const int64 Key = MakeFaceKey(Face.Axis, Face.Pos);
        const TArray<FNav3DVoxelID>* Bucket = Buckets.Find(Key);
        if (!Bucket) { continue; }

        float BestDist2 = TNumericLimits<float>::Max();
        const FNav3DVoxelID* Best = nullptr;
        const FVector PosA = FNav3DUtils::GetVoxelWorldPosition(VoxelID, CachedChunkActors);

        for (const FNav3DVoxelID& Candidate : *Bucket)
        {
            // cross-actor only
            if (Candidate.ChunkIndex == VoxelID.ChunkIndex) { continue; }

            if (!FNav3DUtils::AreFaceAdjacent(VoxelID, Candidate, CachedChunkActors)) { continue; }

            const FVector PosB = FNav3DUtils::GetVoxelWorldPosition(Candidate, CachedChunkActors);
            const float Dist2 = FVector::DistSquared(PosA, PosB);
            if (Dist2 < BestDist2)
            {
                BestDist2 = Dist2;
                Best = &Candidate;
            }
        }

        if (Best)
        {
            const FVector Portal = FNav3DUtils::CalculateSharedFacePortal(VoxelID, *Best, CachedChunkActors);
            const float Cost = FMath::Sqrt(BestDist2);
            Computed.Add(FNav3DCrossVolumeConnection(*Best, Portal, Cost));
        }
    }

    Adjacency.Add(VoxelID, Computed);
    OutConnections = MoveTemp(Computed);
}


