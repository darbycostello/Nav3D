#include "Pathfinding/Core/Nav3DVolumePathfinder.h"
#include "Pathfinding/Core/Nav3DPath.h"
#include "Pathfinding/Core/INav3DPathfinder.h"
#include "Nav3D.h"
#include "Nav3DData.h"
#include "Nav3DDataChunkActor.h"
#include "Nav3DBoundsVolume.h"
#include "Nav3DWorldSubsystem.h"
#include "Nav3DDataChunk.h"
#include "Nav3DVolumeNavigationData.h"
#include "EngineUtils.h"
#include "Nav3DSettings.h"
#include "Raycasting/Nav3DMultiChunkRaycaster.h"

namespace
{
static FString ChunkLabel(const ANav3DDataChunkActor* Chunk)
{
	if (!Chunk)
	{
		return TEXT("None");
	}
	return Chunk->GetActorNameOrLabel();
}
}

FNav3DVolumePathfinder::FNav3DVolumePathfinder() {}
FNav3DVolumePathfinder::~FNav3DVolumePathfinder() {}

ENavigationQueryResult::Type FNav3DVolumePathfinder::FindPath(
	FNav3DPath& OutPath,
	const FNav3DPathingRequest& Request,
	INav3DPathfinder* Algorithm)
{
	CurrentRequest = Request;
	CurrentNavData = Request.NavData;

	// PHASE 1: SPATIAL ANALYSIS
	const ANav3DDataChunkActor* StartChunk = FindChunkContaining(Request.StartLocation);
	const ANav3DDataChunkActor* EndChunk = FindChunkContaining(Request.EndLocation);
	const ANav3DBoundsVolume* StartVolume = FindVolumeContaining(Request.StartLocation);
	const ANav3DBoundsVolume* EndVolume = FindVolumeContaining(Request.EndLocation);

	LogSpatialAnalysis(Request.StartLocation, Request.EndLocation, StartChunk, EndChunk, StartVolume, EndVolume);

	// PHASE 2: TRACE DECISION TREE
	UE_LOG(LogNav3D, Verbose, TEXT("DecisionTree: StartChunk=%s EndChunk=%s StartVolume=%s EndVolume=%s"),
		StartChunk ? *StartChunk->GetActorNameOrLabel() : TEXT("None"),
		EndChunk ? *EndChunk->GetActorNameOrLabel() : TEXT("None"),
		StartVolume ? *StartVolume->GetActorNameOrLabel() : TEXT("None"),
		EndVolume ? *EndVolume->GetActorNameOrLabel() : TEXT("None"));

	// PHASE 2: ROUTING DECISION TREE
	if (StartChunk == nullptr && EndChunk == nullptr)
	{
		UE_LOG(LogNav3D, Verbose, TEXT("DecisionTree: No start/end chunk → direct path"));
		return CreateDirectPath(OutPath, Request.StartLocation, Request.EndLocation);
	}
	else if (StartChunk == EndChunk && StartChunk != nullptr)
	{
		UE_LOG(LogNav3D, Verbose, TEXT("DecisionTree: Same chunk (%s)"), *StartChunk->GetActorNameOrLabel());
		if (IsChunkEmpty(StartChunk))
		{
			UE_LOG(LogNav3D, Verbose, TEXT("DecisionTree: Chunk empty → direct path"));
			return CreateDirectPath(OutPath, Request.StartLocation, Request.EndLocation);
		}
		else
		{
			UE_LOG(LogNav3D, Verbose, TEXT("DecisionTree: Chunk has nav → FindPathInChunk"));
			return FindPathInChunk(OutPath, Request.StartLocation, Request.EndLocation, StartChunk, Algorithm);
		}
	}
    else if (StartVolume == EndVolume && StartVolume != nullptr)
    {
    UE_LOG(LogNav3D, Verbose, TEXT("DecisionTree: Same volume (%s) → FindPathWithinVolume"), *StartVolume->GetActorNameOrLabel());
    UE_LOG(LogNav3D, Verbose, TEXT("FindPathWithinVolume: Start=%s End=%s"), *Request.StartLocation.ToString(), *Request.EndLocation.ToString());
        return FindPathWithinVolume(OutPath, Request.StartLocation, Request.EndLocation, StartVolume, Algorithm, StartChunk, EndChunk);
    }
	else
	{
		UE_LOG(LogNav3D, Verbose, TEXT("DecisionTree: Cross-volume (StartVol=%s, EndVol=%s) → FindPathCrossVolume"),
			StartVolume ? *StartVolume->GetActorNameOrLabel() : TEXT("None"),
			EndVolume ? *EndVolume->GetActorNameOrLabel() : TEXT("None"));
		return FindPathCrossVolume(OutPath, Request.StartLocation, Request.EndLocation, StartVolume, EndVolume, Algorithm);
	}
}

const ANav3DDataChunkActor* FNav3DVolumePathfinder::FindChunkContaining(const FVector& Location) const
{
	if (!CurrentNavData || !CurrentNavData->GetWorld())
	{
		return nullptr;
	}

	if (const UNav3DWorldSubsystem* Subsystem = CurrentNavData->GetWorld()->GetSubsystem<UNav3DWorldSubsystem>())
	{
		TArray<ANav3DDataChunkActor*> Candidates;
		Subsystem->QueryActorsInBounds(FBox(Location, Location), Candidates);
		for (const ANav3DDataChunkActor* Candidate : Candidates)
		{
			if (Candidate && Candidate->DataChunkActorBounds.IsInside(Location))
			{
				return Candidate;
			}
		}
	}

	for (TActorIterator<ANav3DDataChunkActor> It(CurrentNavData->GetWorld()); It; ++It)
	{
		const ANav3DDataChunkActor* ChunkActor = *It;
		if (ChunkActor && ChunkActor->DataChunkActorBounds.IsInside(Location))
		{
			return ChunkActor;
		}
	}

	return nullptr;
}

const ANav3DBoundsVolume* FNav3DVolumePathfinder::FindVolumeContaining(const FVector& Location) const
{
	if (!CurrentNavData || !CurrentNavData->GetWorld())
	{
		return nullptr;
	}

	for (TActorIterator<ANav3DBoundsVolume> It(CurrentNavData->GetWorld()); It; ++It)
	{
		if (const ANav3DBoundsVolume* Volume = *It)
		{
			const FBoxSphereBounds Bounds = Volume->GetBounds();
			if (const FBox BoundsBox(Bounds.Origin - Bounds.BoxExtent, Bounds.Origin + Bounds.BoxExtent);
				BoundsBox.IsInside(Location))
			{
				return Volume;
			}
		}
	}

	return nullptr;
}

bool FNav3DVolumePathfinder::IsChunkEmpty(const ANav3DDataChunkActor* Chunk)
{
	if (!Chunk)
	{
		return true;
	}

	for (const UNav3DDataChunk* Nav3DChunk : Chunk->Nav3DChunks)
	{
		if (!Nav3DChunk)
		{
			continue;
		}

		const FNav3DVolumeNavigationData* VolumeData = Nav3DChunk->GetVolumeNavigationData();
		if (!VolumeData)
		{
			continue;
		}

		for (LayerIndex LayerIdx = 0; LayerIdx < VolumeData->GetData().GetLayerCount(); LayerIdx++)
		{
			const auto& Layer = VolumeData->GetData().GetLayer(LayerIdx);
			if (Layer.GetNodeCount() > 0)
			{
				return false;
			}
		}
	}

	return true;
}

ENavigationQueryResult::Type FNav3DVolumePathfinder::FindPathWithinVolume(
	FNav3DPath& OutPath,
	const FVector& StartLocation,
	const FVector& EndLocation,
	const ANav3DBoundsVolume* Volume,
	INav3DPathfinder* Algorithm) const
{
	// Delegate to the overload that accepts resolved chunks to avoid code duplication
	const ANav3DDataChunkActor* StartChunk = FindChunkContaining(StartLocation);
	const ANav3DDataChunkActor* EndChunk = FindChunkContaining(EndLocation);
	return FindPathWithinVolume(OutPath, StartLocation, EndLocation, Volume, Algorithm, StartChunk, EndChunk);
}

ENavigationQueryResult::Type FNav3DVolumePathfinder::FindPathWithinVolume(
    FNav3DPath& OutPath,
    const FVector& StartLocation,
    const FVector& EndLocation,
    const ANav3DBoundsVolume* Volume,
    INav3DPathfinder* Algorithm,
    const ANav3DDataChunkActor* ResolvedStartChunk,
    const ANav3DDataChunkActor* ResolvedEndChunk) const
{
    // Use provided chunks directly to avoid re-query divergence
    const ANav3DDataChunkActor* StartChunk = ResolvedStartChunk;
    const ANav3DDataChunkActor* EndChunk = ResolvedEndChunk;

    if (StartChunk == EndChunk)
    {
        if (StartChunk)
        {
            return FindPathInChunk(OutPath, StartLocation, EndLocation, StartChunk, Algorithm);
        }
        return CreateDirectPath(OutPath, StartLocation, EndLocation);
    }

    TArray<const ANav3DDataChunkActor*> ChunkPath = FindChunkPathWithinVolume(StartChunk, EndChunk);
    if (ChunkPath.Num() == 0)
    {
        UE_LOG(LogNav3D, Verbose, TEXT("FindPathWithinVolume: No adjacency path between %s and %s"),
               StartChunk ? *StartChunk->GetActorNameOrLabel() : TEXT("None"),
               EndChunk ? *EndChunk->GetActorNameOrLabel() : TEXT("None"));

        // Geometric fallback: choose the first chunk intersected by the ray from Start to End
        const FVector RayDir = (EndLocation - StartLocation).GetSafeNormal();
        const float RayLen = FVector::Dist(StartLocation, EndLocation);

        const ANav3DDataChunkActor* FirstStep = nullptr;
        float BestT = TNumericLimits<float>::Max();

        for (TActorIterator<ANav3DDataChunkActor> It(CurrentNavData->GetWorld()); It; ++It)
        {
            const ANav3DDataChunkActor* Candidate = *It;
            if (!Candidate || Candidate == StartChunk) { continue; }

            const FBox& Box = Candidate->DataChunkActorBounds;
            // Ray-box intersection (slab method)
            FVector InvDir( RayDir.X != 0.f ? 1.f/RayDir.X : BIG_NUMBER,
                            RayDir.Y != 0.f ? 1.f/RayDir.Y : BIG_NUMBER,
                            RayDir.Z != 0.f ? 1.f/RayDir.Z : BIG_NUMBER);
            const FVector t0 = (Box.Min - StartLocation) * InvDir;
            const FVector t1 = (Box.Max - StartLocation) * InvDir;
            const FVector tminVec(FMath::Min(t0.X, t1.X), FMath::Min(t0.Y, t1.Y), FMath::Min(t0.Z, t1.Z));
            const FVector tmaxVec(FMath::Max(t0.X, t1.X), FMath::Max(t0.Y, t1.Y), FMath::Max(t0.Z, t1.Z));
            const float tEnter = FMath::Max3(tminVec.X, tminVec.Y, tminVec.Z);
            const float tExit  = FMath::Min3(tmaxVec.X, tmaxVec.Y, tmaxVec.Z);
            if (tEnter <= tExit && tExit >= 0.f && tEnter <= RayLen)
            {
                if (tEnter > 0.f && tEnter < BestT)
                {
                    BestT = tEnter;
                    FirstStep = Candidate;
                }
            }
        }

        if (!FirstStep)
        {
            UE_LOG(LogNav3D, Warning, TEXT("FindPathWithinVolume: Geometric fallback found no next chunk; returning Fail"));
            return ENavigationQueryResult::Fail;
        }

        // Build segments using a first direct segment into the first navigable chunk along the ray
        TArray<FPathSegment> Segments;
        const FVector EntryPoint = StartLocation + RayDir * BestT;
        Segments.Add({StartLocation, EntryPoint, nullptr, false, StartChunk});

        // Continue from FirstStep toward EndChunk using adjacency path if possible
        TArray<const ANav3DDataChunkActor*> TailPath = FindChunkPathWithinVolume(FirstStep, EndChunk);
        FVector CurrentPoint = EntryPoint;
        if (TailPath.Num() == 0)
        {
            // Last resort: single within-chunk segment from entry to End
            const FNav3DVolumeNavigationData* VD = (FirstStep->Nav3DChunks.Num() > 0 && FirstStep->Nav3DChunks[0])
                ? FirstStep->Nav3DChunks[0]->GetVolumeNavigationData() : nullptr;
            Segments.Add({CurrentPoint, EndLocation, VD, false, FirstStep});
            return ProcessPathSegments(OutPath, Segments, Algorithm);
        }

        for (int32 i = 0; i < TailPath.Num(); ++i)
        {
            const bool bLast = (i == TailPath.Num() - 1);
            const ANav3DDataChunkActor* PlannedChunk = TailPath[i];

            // Resolve actual start chunk for VolumeData
            const ANav3DDataChunkActor* ActualStartChunk = FindChunkContaining(CurrentPoint);
            const FNav3DVolumeNavigationData* StartVD = (ActualStartChunk && ActualStartChunk->Nav3DChunks.Num() > 0 && ActualStartChunk->Nav3DChunks[0])
                ? ActualStartChunk->Nav3DChunks[0]->GetVolumeNavigationData() : nullptr;

            if (bLast)
            {
                Segments.Add({CurrentPoint, EndLocation, StartVD, false, ActualStartChunk});
                CurrentPoint = EndLocation;
            }
            else
            {
                const ANav3DDataChunkActor* NextChunk = TailPath[i + 1];
                FVector LocalPortal, RemotePortal;
                if (GetPortalPositions(PlannedChunk, NextChunk, LocalPortal, RemotePortal))
                {
                    Segments.Add({CurrentPoint, LocalPortal, StartVD, false, ActualStartChunk});
                    CurrentPoint = RemotePortal;
                }
                else
                {
                    const FVector FallbackTarget = FindPortalBetweenChunks(PlannedChunk, NextChunk);
                    Segments.Add({CurrentPoint, FallbackTarget, StartVD, false, ActualStartChunk});
                    CurrentPoint = FallbackTarget;
                }
            }
        }

        return ProcessPathSegments(OutPath, Segments, Algorithm);
    }
    
    TArray<FPathSegment> Segments;
    FVector CurrentPoint = StartLocation;

	for (int32 i = 0; i < ChunkPath.Num(); ++i)
    {
		const bool bLast = (i == ChunkPath.Num() - 1);

        // Determine the actual chunk from the current point for VolumeData
		const ANav3DDataChunkActor* ActualStartChunk = FindChunkContaining(CurrentPoint);
		const FNav3DVolumeNavigationData* StartVolumeData = ActualStartChunk && ActualStartChunk->Nav3DChunks.Num() > 0 && ActualStartChunk->Nav3DChunks[0]
			? ActualStartChunk->Nav3DChunks[0]->GetVolumeNavigationData()
			: nullptr;

        // Use the planned chunk path only to decide the target portal
		const ANav3DDataChunkActor* PlannedChunk = ChunkPath[i];
		if (bLast)
		{
			Segments.Add({CurrentPoint, EndLocation, StartVolumeData, false, ActualStartChunk});
			CurrentPoint = EndLocation;
		}
		else
		{
			const ANav3DDataChunkActor* NextChunk = ChunkPath[i + 1];
			FVector LocalPortal;
			FVector RemotePortal;
			if (GetPortalPositions(PlannedChunk, NextChunk, LocalPortal, RemotePortal))
			{
				Segments.Add({CurrentPoint, LocalPortal, StartVolumeData, false, ActualStartChunk});
				CurrentPoint = RemotePortal;
			}
			else
			{
				const FVector FallbackTarget = FindPortalBetweenChunks(PlannedChunk, NextChunk);
				Segments.Add({CurrentPoint, FallbackTarget, StartVolumeData, false, ActualStartChunk});
				CurrentPoint = FallbackTarget;
			}
		}
    }

    return ProcessPathSegments(OutPath, Segments, Algorithm);
}

ENavigationQueryResult::Type FNav3DVolumePathfinder::FindPathCrossVolume(
	FNav3DPath& OutPath,
	const FVector& StartLocation,
	const FVector& EndLocation,
	const ANav3DBoundsVolume* StartVolume,
	const ANav3DBoundsVolume* EndVolume,
	INav3DPathfinder* Algorithm) const
{
	TArray<FPathSegment> Segments;
	FVector CurrentPoint = StartLocation;

	if (StartVolume)
	{
		const FVector ExitPoint = FindVolumeExitPoint(StartVolume, StartLocation, EndLocation);
		FNav3DPath StartPath;
		if (FindPathWithinVolume(StartPath, StartLocation, ExitPoint, StartVolume, Algorithm) == ENavigationQueryResult::Success)
		{
			Segments.Add({StartLocation, ExitPoint, nullptr, true, nullptr});
			CurrentPoint = ExitPoint;
		}
	}

	if (EndVolume)
	{
		const FVector EntryPoint = FindVolumeEntryPoint(EndVolume, CurrentPoint, EndLocation);
		Segments.Add({CurrentPoint, EntryPoint, nullptr, false, nullptr});
		CurrentPoint = EntryPoint;
		Segments.Add({EntryPoint, EndLocation, nullptr, true, nullptr});
	}
	else
	{
		Segments.Add({CurrentPoint, EndLocation, nullptr, false, nullptr});
	}

	// Final segment inside end volume
	if (EndVolume)
	{
		FNav3DPath EndPath;
		FindPathWithinVolume(EndPath, CurrentPoint, EndLocation, EndVolume, Algorithm);
	}

	return CombinePathSegments(OutPath, Segments);
}

ENavigationQueryResult::Type FNav3DVolumePathfinder::FindPathInChunk(
	FNav3DPath& OutPath,
	const FVector& StartLocation,
	const FVector& EndLocation,
	const ANav3DDataChunkActor* Chunk,
	INav3DPathfinder* Algorithm) const
{
	if (!Algorithm || !Chunk)
	{
		return ENavigationQueryResult::Error;
	}

	if (IsChunkEmpty(Chunk))
	{
		UE_LOG(LogNav3D, Verbose, TEXT("FindPathInChunk: Chunk %s is empty - using direct path"), Chunk ? *Chunk->GetName() : TEXT("<null>"));
		return CreateDirectPath(OutPath, StartLocation, EndLocation);
	}

	const FNav3DVolumeNavigationData* VolumeNavData = nullptr;
	if (Chunk->Nav3DChunks.Num() > 0 && Chunk->Nav3DChunks[0])
	{
		VolumeNavData = Chunk->Nav3DChunks[0]->GetVolumeNavigationData();
	}

	if (!VolumeNavData)
	{
		UE_LOG(LogNav3D, Warning, TEXT("FindPathInChunk: No volume navigation data in chunk %s - using direct path"), *Chunk->GetName());
		return CreateDirectPath(OutPath, StartLocation, EndLocation);
	}

    UE_LOG(LogNav3D, Verbose, TEXT("FindPathInChunk: Using algorithm for pathfinding in chunk %s"), *Chunk->GetName());
    if (VolumeNavData)
    {
        const FBox& NavBounds = VolumeNavData->GetNavigationBounds();
        const FBox& VolBounds = VolumeNavData->GetVolumeBounds();
        const bool bStartInsideNav = NavBounds.IsInside(StartLocation);
        const bool bEndInsideNav = NavBounds.IsInside(EndLocation);
        const bool bStartInsideVol = VolBounds.IsInside(StartLocation);
        const bool bEndInsideVol = VolBounds.IsInside(EndLocation);
        UE_LOG(LogNav3D, VeryVerbose, TEXT("FindPathInChunk: NavBounds Min=%s Max=%s | VolBounds Min=%s Max=%s | StartIn{Nav=%d Vol=%d} EndIn{Nav=%d Vol=%d}"),
               *NavBounds.Min.ToString(), *NavBounds.Max.ToString(), *VolBounds.Min.ToString(), *VolBounds.Max.ToString(),
               bStartInsideNav, bStartInsideVol, bEndInsideNav, bEndInsideVol);
    }
	const ENavigationQueryResult::Type Result = Algorithm->FindPath(OutPath, CurrentRequest, VolumeNavData);
	if (Result != ENavigationQueryResult::Success)
	{
		UE_LOG(LogNav3D, Warning, TEXT("FindPathInChunk: Algorithm failed - falling back to direct path"));
		return CreateDirectPath(OutPath, StartLocation, EndLocation);
	}

	return Result;
}

TArray<const ANav3DDataChunkActor*> FNav3DVolumePathfinder::FindChunkPathWithinVolume(
	const ANav3DDataChunkActor* StartChunk,
	const ANav3DDataChunkActor* EndChunk)
{
	TArray<const ANav3DDataChunkActor*> Result;
	if (!StartChunk || !EndChunk)
	{
		return Result;
	}
	if (StartChunk == EndChunk)
	{
		Result.Add(StartChunk);
		return Result;
	}

	TQueue<const ANav3DDataChunkActor*> Queue;
	TMap<const ANav3DDataChunkActor*, const ANav3DDataChunkActor*> CameFrom;
	TSet<const ANav3DDataChunkActor*> Visited;

	Queue.Enqueue(StartChunk);
	Visited.Add(StartChunk);
	CameFrom.Add(StartChunk, nullptr);

	while (!Queue.IsEmpty())
	{
		const ANav3DDataChunkActor* Current = nullptr;
		Queue.Dequeue(Current);
		if (Current == EndChunk)
		{
			TArray<const ANav3DDataChunkActor*> Path;
			const ANav3DDataChunkActor* Node = EndChunk;
			while (Node)
			{
				Path.Add(Node);
				Node = CameFrom.FindRef(Node);
			}
			Algo::Reverse(Path);
			return Path;
		}

		for (const FNav3DChunkAdjacency& Adjacency : Current->ChunkAdjacency)
		{
			const ANav3DDataChunkActor* Neighbor = Adjacency.OtherChunkActor.Get();
			if (Neighbor && !Visited.Contains(Neighbor))
			{
				Queue.Enqueue(Neighbor);
				Visited.Add(Neighbor);
				CameFrom.Add(Neighbor, Current);
			}
		}
	}

	return Result;
}

FVector FNav3DVolumePathfinder::FindPortalBetweenChunks(
	const ANav3DDataChunkActor* FromChunk,
	const ANav3DDataChunkActor* ToChunk)
{
	if (!FromChunk || !ToChunk)
	{
		return FVector::ZeroVector;
	}

	const FNav3DChunkAdjacency* Adjacency = FromChunk->ChunkAdjacency.FindByPredicate(
		[ToChunk](const FNav3DChunkAdjacency& Adj)
		{
			return Adj.OtherChunkActor.Get() == ToChunk;
		});

	auto SnapToNavigable = [](const FNav3DVolumeNavigationData* VD, const FVector& Pos) -> TOptional<FVector>
	{
		if (!VD) { return TOptional<FVector>(); }
		FNav3DNodeAddress Addr;
		if (VD->GetNodeAddressFromPosition(Addr, Pos, /*MinLayerIndex*/0))
		{
			const FNav3DData& Data = VD->GetData();
			bool bNavigable = false;
			if (Addr.LayerIndex == 0)
			{
				const auto& LeafNodes = Data.GetLeafNodes();
				if (LeafNodes.GetLeafNodes().IsValidIndex(Addr.NodeIndex))
				{
					const auto& Leaf = LeafNodes.GetLeafNode(Addr.NodeIndex);
					bNavigable = !Leaf.IsSubNodeOccluded(Addr.SubNodeIndex);
				}
			}
			else
			{
				const auto& Node = VD->GetNodeFromAddress(Addr);
				bNavigable = !Node.HasChildren();
			}
			if (bNavigable)
			{
				return VD->GetNodePositionFromAddress(Addr, /*TryGetSubNodePosition*/true);
			}
		}
		FNav3DNodeAddress Best;
		if (VD->FindNearestNavigableNode(Pos, Best, /*MinLayerIndex*/0))
		{
			return VD->GetNodePositionFromAddress(Best, /*TryGetSubNodePosition*/true);
		}
		return TOptional<FVector>();
	};

	if (Adjacency && Adjacency->CompactPortals.Num() > 0)
	{
		const FCompactPortal& Portal = Adjacency->CompactPortals[0];
		if (FromChunk->Nav3DChunks.Num() > 0 && FromChunk->Nav3DChunks[0])
		{
			if (const FNav3DVolumeNavigationData* VD = FromChunk->Nav3DChunks[0]->GetVolumeNavigationData())
			{
				const FVector Guess = VD->GetLeafNodePositionFromMortonCode(Portal.Local);
				if (const TOptional<FVector> Fixed = SnapToNavigable(VD, Guess))
				{
					return Fixed.GetValue();
				}
			}
		}
	}

	const FVector FallbackGuess = FromChunk->DataChunkActorBounds.GetClosestPointTo(ToChunk->DataChunkActorBounds.GetCenter());
	const FNav3DVolumeNavigationData* FromVD = (FromChunk->Nav3DChunks.Num() > 0 && FromChunk->Nav3DChunks[0]) ? FromChunk->Nav3DChunks[0]->GetVolumeNavigationData() : nullptr;
	if (const TOptional<FVector> Fixed = SnapToNavigable(FromVD, FallbackGuess))
	{
		return Fixed.GetValue();
	}
	return FallbackGuess;
}

bool FNav3DVolumePathfinder::GetPortalPositions(
    const ANav3DDataChunkActor* FromChunk,
    const ANav3DDataChunkActor* ToChunk,
    FVector& OutLocalInFrom,
    FVector& OutRemoteInTo)
{
    if (!FromChunk || !ToChunk) { return false; }
    const FNav3DChunkAdjacency* Adjacency = FromChunk->ChunkAdjacency.FindByPredicate(
        [ToChunk](const FNav3DChunkAdjacency& Adj)
        {
            return Adj.OtherChunkActor.Get() == ToChunk;
        });
    if (!Adjacency || Adjacency->CompactPortals.Num() == 0)
    {
        return false;
    }
    const FCompactPortal& Portal = Adjacency->CompactPortals[0];
    const FNav3DVolumeNavigationData* FromVD = (FromChunk->Nav3DChunks.Num() > 0 && FromChunk->Nav3DChunks[0]) ? FromChunk->Nav3DChunks[0]->GetVolumeNavigationData() : nullptr;
    const FNav3DVolumeNavigationData* ToVD = (ToChunk->Nav3DChunks.Num() > 0 && ToChunk->Nav3DChunks[0]) ? ToChunk->Nav3DChunks[0]->GetVolumeNavigationData() : nullptr;
    if (!FromVD || !ToVD) { return false; }

    // Initial portal centers from stored morton codes
    const FVector LocalPos  = FromVD->GetLeafNodePositionFromMortonCode(Portal.Local);
    const FVector RemotePos = ToVD->GetLeafNodePositionFromMortonCode(Portal.Remote);

    auto SnapToNavigable = [](const FNav3DVolumeNavigationData* VD, const FVector& Pos) -> TOptional<FVector>
    {
        if (!VD) { return TOptional<FVector>(); }
        FNav3DNodeAddress Addr;
        if (VD->GetNodeAddressFromPosition(Addr, Pos, /*MinLayerIndex*/0))
        {
            const FNav3DData& Data = VD->GetData();
            bool bNavigable = false;
            if (Addr.LayerIndex == 0)
            {
                const auto& LeafNodes = Data.GetLeafNodes();
                if (LeafNodes.GetLeafNodes().IsValidIndex(Addr.NodeIndex))
                {
                    const auto& Leaf = LeafNodes.GetLeafNode(Addr.NodeIndex);
                    bNavigable = !Leaf.IsSubNodeOccluded(Addr.SubNodeIndex);
                }
            }
            else
            {
                const auto& Node = VD->GetNodeFromAddress(Addr);
                bNavigable = !Node.HasChildren();
            }
            if (bNavigable)
            {
                return VD->GetNodePositionFromAddress(Addr, /*TryGetSubNodePosition*/true);
            }
        }
        FNav3DNodeAddress Best;
        if (VD->FindNearestNavigableNode(Pos, Best, /*MinLayerIndex*/0))
        {
            return VD->GetNodePositionFromAddress(Best, /*TryGetSubNodePosition*/true);
        }
        return TOptional<FVector>();
    };

    const TOptional<FVector> FixedLocal  = SnapToNavigable(FromVD, LocalPos);
    const TOptional<FVector> FixedRemote = SnapToNavigable(ToVD,   RemotePos);
    if (!FixedLocal.IsSet() || !FixedRemote.IsSet())
    {
        return false;
    }

    OutLocalInFrom = FixedLocal.GetValue();
    OutRemoteInTo  = FixedRemote.GetValue();
    return true;
}

ENavigationQueryResult::Type FNav3DVolumePathfinder::ProcessPathSegments(
	FNav3DPath& OutPath,
	const TArray<FPathSegment>& Segments,
	INav3DPathfinder* Algorithm) const
{
	if (Segments.Num() == 0)
	{
		return ENavigationQueryResult::Fail;
	}

	TArray<FNavPathPoint> CombinedPoints;
	CombinedPoints.Reset();

	bool bAllSegmentsSucceeded = true;

	// Determine the starting volume for exit checks
	const ANav3DBoundsVolume* StartVolume = FindVolumeContaining(Segments[0].StartPoint);

	for (int32 SegmentIndex = 0; SegmentIndex < Segments.Num(); ++SegmentIndex)
	{
		const FPathSegment& Segment = Segments[SegmentIndex];
		// Verbose: segment intent and spatial context
		const ANav3DDataChunkActor* SegmentStartChunk = FindChunkContaining(Segment.StartPoint);
		const ANav3DDataChunkActor* SegmentEndChunkPlanned = FindChunkContaining(Segment.EndPoint);
		UE_LOG(LogNav3D, Verbose, TEXT("ProcessPathSegments: Segment %d/%d from %s -> %s | StartChunk=%s PlannedEndChunk=%s%s"),
			SegmentIndex + 1, Segments.Num(),
			*Segment.StartPoint.ToString(), *Segment.EndPoint.ToString(),
			*ChunkLabel(SegmentStartChunk),
			*ChunkLabel(SegmentEndChunkPlanned),
			(SegmentStartChunk != SegmentEndChunkPlanned ? TEXT(" | portal transition planned") : TEXT("")));
		FNav3DPath SegmentPath;
		FNav3DPathingRequest SegmentRequest = CurrentRequest;
		SegmentRequest.StartLocation = Segment.StartPoint;
		SegmentRequest.EndLocation = Segment.EndPoint;

        ENavigationQueryResult::Type SegmentResult = ENavigationQueryResult::Fail;
        if (Algorithm && Segment.VolumeData)
        {
            SegmentResult = Algorithm->FindPath(SegmentPath, SegmentRequest, Segment.VolumeData);
        }
        else
        {
            // Fallback: if start chunk is empty, treat as clear space and use direct segment
            if (SegmentStartChunk && IsChunkEmpty(SegmentStartChunk))
            {
                SegmentPath.ResetForRepath();
                TArray<FNavPathPoint>& Pts = SegmentPath.GetPathPoints();
                Pts.Add(FNavPathPoint(Segment.StartPoint));
                Pts.Add(FNavPathPoint(Segment.EndPoint));
                SegmentResult = ENavigationQueryResult::Success;
            }
        }

		if (SegmentResult == ENavigationQueryResult::Success)
		{
			const TArray<FNavPathPoint>& SegmentPoints = SegmentPath.GetPathPoints();
			UE_LOG(LogNav3D, Verbose, TEXT("ProcessPathSegments: Segment %d succeeded with %d points (first=%s, last=%s)"),
				SegmentIndex + 1,
				SegmentPoints.Num(),
				SegmentPoints.Num() > 0 ? *SegmentPoints[0].Location.ToString() : TEXT("None"),
				SegmentPoints.Num() > 0 ? *SegmentPoints.Last().Location.ToString() : TEXT("None"));

			// VeryVerbose: enumerate points and detect portal enter/exit between chunks
			for (int32 P = 0; P < SegmentPoints.Num(); ++P)
			{
				const FVector& Pt = SegmentPoints[P].Location;
				const ANav3DDataChunkActor* PtChunk = FindChunkContaining(Pt);
				const FString PtChunkName = ChunkLabel(PtChunk);
				if (P > 0)
				{
					const FVector& PrevPt = SegmentPoints[P - 1].Location;
					const ANav3DDataChunkActor* PrevChunk = FindChunkContaining(PrevPt);
					if (PrevChunk != PtChunk)
					{
						UE_LOG(LogNav3D, VeryVerbose, TEXT("  Point %d: %s | %s (ENTER portal from %s)"),
							P, *Pt.ToString(), *PtChunkName, *ChunkLabel(PrevChunk));
					}
					else
					{
						UE_LOG(LogNav3D, VeryVerbose, TEXT("  Point %d: %s | %s"),
							P, *Pt.ToString(), *PtChunkName);
					}
				}
				else
				{
					UE_LOG(LogNav3D, VeryVerbose, TEXT("  Point %d: %s | %s"),
						P, *Pt.ToString(), *PtChunkName);
				}
			}

            // Deduplicate consecutive identical points to avoid log/path bloat
            if (SegmentPoints.Num() > 0)
            {
                if (CombinedPoints.Num() == 0)
                {
                    CombinedPoints.Add(SegmentPoints[0]);
                }
                for (int32 S = 1; S < SegmentPoints.Num(); ++S)
                {
                    if (!SegmentPoints[S].Location.Equals(CombinedPoints.Last().Location, KINDA_SMALL_NUMBER))
                    {
                        CombinedPoints.Add(SegmentPoints[S]);
                    }
                }
            }

			// Cross-chunk validation for non-final segments, unless exiting volume
			if (SegmentIndex < Segments.Num() - 1)
			{
				const ANav3DBoundsVolume* TargetVolume = FindVolumeContaining(Segment.EndPoint);
				if (TargetVolume == StartVolume)
				{
					const FVector ActualEndPoint = SegmentPath.GetEndLocation();
					const ANav3DDataChunkActor* ExpectedTargetChunk = FindChunkContaining(Segment.EndPoint);
					const ANav3DDataChunkActor* ActualTargetChunk = FindChunkContaining(ActualEndPoint);
					UE_LOG(LogNav3D, Verbose, TEXT("ProcessPathSegments: Segment %d end at %s | ActualChunk=%s ExpectedChunk=%s"),
						SegmentIndex + 1,
						*ActualEndPoint.ToString(),
						*ChunkLabel(ActualTargetChunk),
						*ChunkLabel(ExpectedTargetChunk));
					if (ExpectedTargetChunk != ActualTargetChunk)
					{
						UE_LOG(LogNav3D, Warning, TEXT("Segment %d failed to reach target chunk - stopping here"), SegmentIndex);
						bAllSegmentsSucceeded = false;
						break;
					}
				}
				else
				{
					UE_LOG(LogNav3D, Verbose, TEXT("ProcessPathSegments: Segment %d exits volume - skipping chunk validation"), SegmentIndex + 1);
				}
			}
		}
        else
		{
			UE_LOG(LogNav3D, Warning, TEXT("ProcessPathSegments: Segment %d FAILED | Start=%s End=%s StartChunk=%s PlannedEndChunk=%s"),
				SegmentIndex + 1,
				*Segment.StartPoint.ToString(), *Segment.EndPoint.ToString(),
				*ChunkLabel(SegmentStartChunk),
				*ChunkLabel(SegmentEndChunkPlanned));
			bAllSegmentsSucceeded = false;
			break;
		}
	}

    // Optional pruning pass using direct traversal (before smoothing)
    if (const UNav3DSettings* Settings = UNav3DSettings::Get())
    {
        if (Settings->bPrunePaths && CombinedPoints.Num() > 2)
        {
            UNav3DMultiChunkRaycaster* Raycaster = NewObject<UNav3DMultiChunkRaycaster>();
            TArray<FNavPathPoint> Pruned;
            Pruned.Reserve(CombinedPoints.Num());
            Pruned.Add(CombinedPoints[0]);

            const float AgentRadius = CurrentRequest.AgentProperties.AgentRadius;
            // Pruning budgets (configurable via cvars)
            static TAutoConsoleVariable<int32> CVarPruneMaxLOS(
                TEXT("nav3d.Prune.MaxLOSChecks"), 512,
                TEXT("Max direct-traversal checks during path pruning. Prevents quadratic blowups."),
                ECVF_Default);
            static TAutoConsoleVariable<int32> CVarPruneMaxBackscan(
                TEXT("nav3d.Prune.MaxBackscan"), 64,
                TEXT("Max number of points to scan backwards per anchor during pruning."),
                ECVF_Default);

            const int32 MaxLOSChecks = FMath::Max(0, CVarPruneMaxLOS.GetValueOnAnyThread());
            const int32 MaxBackscan  = FMath::Max(1, CVarPruneMaxBackscan.GetValueOnAnyThread());
            int32 LOSChecksUsed = 0;

            int32 i = 0;
            while (i < CombinedPoints.Num() - 1)
            {
                int32 best = i + 1;
                const int32 jMax = CombinedPoints.Num() - 1;
                const int32 window = FMath::Min(MaxBackscan, jMax - (i + 1));
                for (int32 j = i + 1 + window; j > i + 1; --j)
                {
                    if (MaxLOSChecks > 0 && LOSChecksUsed >= MaxLOSChecks)
                    {
                        // Budget exhausted: append remaining as-is and stop pruning
                        for (int32 k = i + 1; k < CombinedPoints.Num(); ++k)
                        {
                            Pruned.Add(CombinedPoints[k]);
                        }
                        CombinedPoints = MoveTemp(Pruned);
                        goto PruneDone;
                    }
                    FNav3DRaycastHit Hit;
                    if (Raycaster->HasLineOfTraversal(CurrentRequest.NavData,
                        CombinedPoints[i].Location,
                        CombinedPoints[j].Location,
                        AgentRadius,
                        Hit))
                    {
                        best = j;
                        break;
                    }
                    ++LOSChecksUsed;
                }
                Pruned.Add(CombinedPoints[best]);
                i = best;
            }
            CombinedPoints = MoveTemp(Pruned);
        }
    }

PruneDone:
    OutPath.GetPathPoints() = MoveTemp(CombinedPoints);
    OutPath.MarkReady();

    if (bAllSegmentsSucceeded)
	{
		return ENavigationQueryResult::Success;
	}
    // Only report success if path actually reaches final segment end
    const bool bReachedEnd = OutPath.GetPathPoints().Num() > 0 && OutPath.GetPathPoints().Last().Location.Equals(Segments.Last().EndPoint, 1.0f);
    return bReachedEnd ? ENavigationQueryResult::Success : ENavigationQueryResult::Fail;
}

FVector FNav3DVolumePathfinder::FindVolumeExitPoint(const ANav3DBoundsVolume* Volume, const FVector& StartLocation, const FVector& TowardLocation)
{
	if (!Volume)
	{
		return StartLocation;
	}

	const FBoxSphereBounds Bounds = Volume->GetBounds();
	const FBox VolumeBounds(Bounds.Origin - Bounds.BoxExtent, Bounds.Origin + Bounds.BoxExtent);
	const FVector Direction = (TowardLocation - StartLocation).GetSafeNormal();
	if (Direction.IsNearlyZero())
	{
		return StartLocation;
	}

	const float MaxDistance = FVector::Dist(StartLocation, TowardLocation) * 2.0f;
	const FVector RayEnd = StartLocation + Direction * MaxDistance;
	// Fallback to closest point since intersection point API may vary
	return VolumeBounds.GetClosestPointTo(RayEnd);
}

FVector FNav3DVolumePathfinder::FindVolumeEntryPoint(const ANav3DBoundsVolume* Volume, const FVector& FromLocation, const FVector& EndLocation)
{
	if (!Volume)
	{
		return EndLocation;
	}

	const FBoxSphereBounds Bounds = Volume->GetBounds();
	const FBox VolumeBounds(Bounds.Origin - Bounds.BoxExtent, Bounds.Origin + Bounds.BoxExtent);
	const FVector Direction = (EndLocation - FromLocation).GetSafeNormal();
	if (Direction.IsNearlyZero())
	{
		return VolumeBounds.GetClosestPointTo(FromLocation);
	}
	const float MaxDistance = FVector::Dist(FromLocation, EndLocation) * 2.0f;
	const FVector RayEnd = FromLocation + Direction * MaxDistance;
	return VolumeBounds.GetClosestPointTo(RayEnd);
}

ENavigationQueryResult::Type FNav3DVolumePathfinder::CreateDirectPath(FNav3DPath& OutPath, const FVector& Start, const FVector& End)
{
	OutPath.ResetForRepath();
	TArray<FNavPathPoint>& Points = OutPath.GetPathPoints();
	Points.Reset();
	Points.Add(FNavPathPoint(Start));
	Points.Add(FNavPathPoint(End));
	return ENavigationQueryResult::Success;
}

ENavigationQueryResult::Type FNav3DVolumePathfinder::CombinePathSegments(FNav3DPath& OutPath, const TArray<FPathSegment>& Segments)
{
	OutPath.ResetForRepath();
	TArray<FNavPathPoint>& Points = OutPath.GetPathPoints();
	Points.Reset();
	for (const FPathSegment& Segment : Segments)
	{
		Points.Add(FNavPathPoint(Segment.StartPoint));
		Points.Add(FNavPathPoint(Segment.EndPoint));
	}
	return ENavigationQueryResult::Success;
}

void FNav3DVolumePathfinder::LogSpatialAnalysis(const FVector& Start, const FVector& End,
	const ANav3DDataChunkActor* StartChunk, const ANav3DDataChunkActor* EndChunk,
	const ANav3DBoundsVolume* StartVolume, const ANav3DBoundsVolume* EndVolume) const
{
	if (CurrentRequest.LogVerbosity < ENav3DPathingLogVerbosity::Standard)
	{
		return;
	}

	UE_LOG(LogNav3D, Log, TEXT("=== SPATIAL ANALYSIS ==="));
	UE_LOG(LogNav3D, Log, TEXT("Start: %s"), *Start.ToString());
	UE_LOG(LogNav3D, Log, TEXT("End: %s"), *End.ToString());

    UE_LOG(LogNav3D, Log, TEXT("Start Chunk: %s"), StartChunk ? *StartChunk->GetActorNameOrLabel() : TEXT("None"));
    UE_LOG(LogNav3D, Log, TEXT("End Chunk: %s"), EndChunk ? *EndChunk->GetActorNameOrLabel() : TEXT("None"));
    UE_LOG(LogNav3D, Log, TEXT("Start Volume: %s"), StartVolume ? *StartVolume->GetActorNameOrLabel() : TEXT("None"));
    UE_LOG(LogNav3D, Log, TEXT("End Volume: %s"), EndVolume ? *EndVolume->GetActorNameOrLabel() : TEXT("None"));

	if (StartChunk)
	{
		UE_LOG(LogNav3D, Log, TEXT("Start Chunk Empty: %s"), IsChunkEmpty(StartChunk) ? TEXT("Yes") : TEXT("No"));
	}
	if (EndChunk)
	{
		UE_LOG(LogNav3D, Log, TEXT("End Chunk Empty: %s"), IsChunkEmpty(EndChunk) ? TEXT("Yes") : TEXT("No"));
	}
}


