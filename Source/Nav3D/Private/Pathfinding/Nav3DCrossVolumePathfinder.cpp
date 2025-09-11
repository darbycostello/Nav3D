#include "Pathfinding/Nav3DCrossVolumePathfinder.h"

#include "EngineUtils.h"
#include "Nav3D.h"
#include "Nav3DWorldSubsystem.h"
#include "Nav3DDataChunkActor.h"
#include "Nav3DData.h"
#include "Nav3DTacticalActor.h"
#include "Nav3DUtils.h"
#include "Engine/World.h"
#include "Pathfinding/Nav3DCrossVolumeGraph.h"

static float Heuristic(const FBox& A, const FBox& B)
{
	return FVector::Dist(A.GetCenter(), B.GetCenter());
}

bool FNav3DCrossVolumePathfinder::FindActorPath(const UWorld* World,
	const FVector& Start,
	const FVector& End,
	TArray<ANav3DDataChunkActor*>& OutActorPath,
	TArray<FNav3DActorPortal>& OutPortals)
{
	UE_LOG(LogNav3D, Verbose, TEXT("FindActorPath: Starting cross-volume actor pathfinding from %s to %s"), 
	       *Start.ToString(), *End.ToString());
	
	OutActorPath.Reset();
	OutPortals.Reset();
	if (!World) 
	{
		UE_LOG(LogNav3D, Warning, TEXT("FindActorPath: No world provided"));
		return false;
	}
	
	const UNav3DWorldSubsystem* Subsystem = World->GetSubsystem<UNav3DWorldSubsystem>();
	if (!Subsystem) 
	{
		UE_LOG(LogNav3D, Warning, TEXT("FindActorPath: No Nav3DWorldSubsystem found"));
		return false;
	}

	UE_LOG(LogNav3D, Verbose, TEXT("FindActorPath: Found Nav3DWorldSubsystem, querying for start/goal actors"));
	
	// Get Nav3DData early so it's available throughout the function
	ANav3DData* Nav3DData = nullptr;
	for (TActorIterator<ANav3DData> ActorItr(World); ActorItr;)
	{
		Nav3DData = *ActorItr;
		break; // Take the first one
	}
	
	// Let's check what's actually in the spatial subsystem
	UE_LOG(LogNav3D, Verbose, TEXT("FindActorPath: Checking spatial subsystem state"));
	
	// Try to get all actors in a very large bounds to see if the subsystem has any data
	FBox LargeBounds(FVector(-100000.0f), FVector(100000.0f));
	TArray<ANav3DDataChunkActor*> AllActors;
	Subsystem->QueryActorsInBounds(LargeBounds, AllActors);
	UE_LOG(LogNav3D, Verbose, TEXT("FindActorPath: Spatial subsystem contains %d total actors in large bounds"), AllActors.Num());

	// Find start/goal actors
	TArray<ANav3DDataChunkActor*> StartCandidates;
	FBox StartBounds(Start, Start);
	UE_LOG(LogNav3D, Verbose, TEXT("FindActorPath: Querying spatial subsystem for start bounds: %s"), *StartBounds.ToString());
	Subsystem->QueryActorsInBounds(StartBounds, StartCandidates);
	
	TArray<ANav3DDataChunkActor*> GoalCandidates;
	FBox GoalBounds(End, End);
	UE_LOG(LogNav3D, Verbose, TEXT("FindActorPath: Querying spatial subsystem for goal bounds: %s"), *GoalBounds.ToString());
	Subsystem->QueryActorsInBounds(GoalBounds, GoalCandidates);
	
	// Let's also try with slightly larger bounds to see if it's a precision issue
	if (StartCandidates.Num() == 0)
	{
		UE_LOG(LogNav3D, Verbose, TEXT("FindActorPath: No start candidates with point bounds, trying expanded bounds"));
		FBox ExpandedStartBounds(Start - FVector(100.0f), Start + FVector(100.0f));
		UE_LOG(LogNav3D, Verbose, TEXT("FindActorPath: Querying spatial subsystem for expanded start bounds: %s"), *ExpandedStartBounds.ToString());
		Subsystem->QueryActorsInBounds(ExpandedStartBounds, StartCandidates);
	}
	
	if (GoalCandidates.Num() == 0)
	{
		UE_LOG(LogNav3D, Verbose, TEXT("FindActorPath: No goal candidates with point bounds, trying expanded bounds"));
		FBox ExpandedGoalBounds(End - FVector(100.0f), End + FVector(100.0f));
		UE_LOG(LogNav3D, Verbose, TEXT("FindActorPath: Querying spatial subsystem for expanded goal bounds: %s"), *ExpandedGoalBounds.ToString());
		Subsystem->QueryActorsInBounds(ExpandedGoalBounds, GoalCandidates);
	}
	
	UE_LOG(LogNav3D, Verbose, TEXT("FindActorPath: Found %d start candidates, %d goal candidates"), 
	       StartCandidates.Num(), GoalCandidates.Num());
	
	if (StartCandidates.Num() == 0 || GoalCandidates.Num() == 0) 
	{
		UE_LOG(LogNav3D, Verbose, TEXT("FindActorPath: No valid start or goal candidates found from spatial subsystem"));
		
		// Fallback: Try to get the Nav3DData and use its linear search method
		UE_LOG(LogNav3D, Verbose, TEXT("FindActorPath: Attempting fallback to linear search via Nav3DData"));
		
		if (Nav3DData)
		{
			UE_LOG(LogNav3D, Verbose, TEXT("FindActorPath: Found Nav3DData, using linear search fallback"));
			
			// Use the linear search method from Nav3DData
			TArray<ANav3DDataChunkActor*> AllChunkActors = Nav3DData->GetAllChunkActors();
			UE_LOG(LogNav3D, Verbose, TEXT("FindActorPath: Nav3DData has %d chunk actors"), AllChunkActors.Num());
			
			// Find start actor
			for (ANav3DDataChunkActor* ChunkActor : AllChunkActors)
			{
				if (ChunkActor && ChunkActor->ContainsPoint(Start))
				{
					StartCandidates.Add(ChunkActor);
					UE_LOG(LogNav3D, Verbose, TEXT("FindActorPath: Found start actor via linear search: %s"), *ChunkActor->GetName());
					break;
				}
			}
			
			// Find goal actor
			for (ANav3DDataChunkActor* ChunkActor : AllChunkActors)
			{
				if (ChunkActor && ChunkActor->ContainsPoint(End))
				{
					GoalCandidates.Add(ChunkActor);
					UE_LOG(LogNav3D, Verbose, TEXT("FindActorPath: Found goal actor via linear search: %s"), *ChunkActor->GetName());
					break;
				}
			}
			
			UE_LOG(LogNav3D, Verbose, TEXT("FindActorPath: Linear search fallback found %d start candidates, %d goal candidates"), 
			       StartCandidates.Num(), GoalCandidates.Num());
		}
		else
		{
			UE_LOG(LogNav3D, Warning, TEXT("FindActorPath: No Nav3DData found in world for fallback"));
		}
		
		if (StartCandidates.Num() == 0 || GoalCandidates.Num() == 0) 
		{
			UE_LOG(LogNav3D, Warning, TEXT("FindActorPath: Still no valid start or goal candidates found after fallback"));
			return false;
		}
	}
	
	ANav3DDataChunkActor* StartActor = StartCandidates[0];
	ANav3DDataChunkActor* GoalActor = GoalCandidates[0];
	
	UE_LOG(LogNav3D, Verbose, TEXT("FindActorPath: Using start actor: %s, goal actor: %s"), 
	       *StartActor->GetName(), *GoalActor->GetName());

	// A* over actors using baked adjacency
	struct FNode { ANav3DDataChunkActor* Actor; float G=0; float F=0; FNode* Parent=nullptr; FNav3DActorPortal PortalFromParent; bool bHasPortal=false; };
	TMap<ANav3DDataChunkActor*, FNode*> Nodes;
	TSet<ANav3DDataChunkActor*> Closed;
	TArray<FNode*> Open;

	auto GetOrAddNode = [&](ANav3DDataChunkActor* A)->FNode*
	{
		if (FNode** Found = Nodes.Find(A)) return *Found;
		FNode* N = new FNode(); N->Actor = A; Nodes.Add(A, N); return N;
	};

	FNode* StartNode = GetOrAddNode(StartActor);
	StartNode->G = 0;
	StartNode->F = Heuristic(StartActor->DataChunkActorBounds, GoalActor->DataChunkActorBounds);
	Open.Add(StartNode);

	UE_LOG(LogNav3D, Verbose, TEXT("FindActorPath: Starting A* search with start node F=%f"), StartNode->F);

	auto PopBest = [&]() -> FNode*
	{
		int32 BestIdx = 0; float BestF = Open[0]->F;
		for (int32 i=1;i<Open.Num();++i){ if (Open[i]->F < BestF){ BestF=Open[i]->F; BestIdx=i; } }
		FNode* Best = Open[BestIdx];
		Open.RemoveAtSwap(BestIdx);
		return Best;
	};

	int32 IterationCount = 0;
	constexpr int32 MaxIterations = 1000; // Prevent infinite loops
	
	while (Open.Num() > 0 && IterationCount < MaxIterations)
	{
		IterationCount++;
		FNode* Current = PopBest();
		
		UE_LOG(LogNav3D, Verbose, TEXT("FindActorPath: Iteration %d - Processing actor: %s (F=%f, G=%f)"), 
		       IterationCount, *Current->Actor->GetName(), Current->F, Current->G);
		
		if (Current->Actor == GoalActor)
		{
			UE_LOG(LogNav3D, Verbose, TEXT("FindActorPath: Found goal actor! Reconstructing path"));
			// Reconstruct path
			TArray<FNode*> Rev; for (FNode* N = Current; N; N = N->Parent) Rev.Add(N);
			Algo::Reverse(Rev);
			for (FNode* N : Rev) OutActorPath.Add(N->Actor);
			// Collect portals along edges
			for (int32 k=1; k<Rev.Num(); ++k)
			{
				if (FNode* Child = Rev[k]; Child->bHasPortal)
				{
					OutPortals.Add(Child->PortalFromParent);
				}
			}
			UE_LOG(LogNav3D, Verbose, TEXT("FindActorPath: Successfully found path with %d actors and %d portals"), 
			       OutActorPath.Num(), OutPortals.Num());
			
			// Clean up allocated nodes
			for (auto& Pair : Nodes)
			{
				delete Pair.Value;
			}
			
			return true;
		}
		Closed.Add(Current->Actor);

		// Expand neighbors via baked adjacency ONLY - no on-the-fly building
		UE_LOG(LogNav3D, Verbose, TEXT("FindActorPath: Expanding %d adjacent actors for %s"), 
		       Current->Actor->ChunkAdjacency.Num(), *Current->Actor->GetName());
		
		// Check for missing adjacency data - this is a critical error
		if (Current->Actor->ChunkAdjacency.Num() == 0)
		{
			UE_LOG(LogNav3D, Error, TEXT("FindActorPath: Actor %s has NO adjacency data - this indicates missing serialization or incomplete generation"), 
			       *Current->Actor->GetName());
			UE_LOG(LogNav3D, Error, TEXT("FindActorPath: Actor bounds: %s"), *Current->Actor->DataChunkActorBounds.ToString());
			
			// Diagnostic: Show which actors SHOULD be adjacent based on bounds (for debugging only)
			if (Nav3DData)
			{
				TArray<ANav3DDataChunkActor*> AllChunkActors = Nav3DData->GetAllChunkActors();
				int32 NearbyCount = 0;
				for (ANav3DDataChunkActor* OtherActor : AllChunkActors)
				{
					if (OtherActor && OtherActor != Current->Actor)
					{
						// Check if bounds are close (for diagnostic purposes only)
						if (Current->Actor->DataChunkActorBounds.ExpandBy(200.0f).Intersect(OtherActor->DataChunkActorBounds))
						{
							UE_LOG(LogNav3D, Warning, TEXT("FindActorPath: Nearby actor (should be adjacent): %s"), *OtherActor->GetName());
							NearbyCount++;
						}
					}
				}
				UE_LOG(LogNav3D, Error, TEXT("FindActorPath: Found %d nearby actors that should have adjacency data"), NearbyCount);
			}
			
			UE_LOG(LogNav3D, Error, TEXT("FindActorPath: Pathfinding failed due to missing adjacency data. Fix: ensure adjacency is built during generation and properly serialized"));
			
			// Clean up allocated nodes
			for (auto& Pair : Nodes)
			{
				delete Pair.Value;
			}
			
			return false;
		}
		
		int32 ValidNeighbors = 0;
		
		// Prefer cross-volume graph from tactical actors; fallback to baked adjacency
		bool bUsedCrossVolumeGraph = false;

		// Try to find a tactical actor covering current actor
		if (ANav3DData* NavDataOwner = Nav3DData)
		{
			for (ANav3DTacticalActor* Ta : NavDataOwner->GetAllTacticalActors())
			{
				if (!Ta) continue;
				if (!Ta->ContainsPoint(Current->Actor->DataChunkActorBounds.GetCenter())) continue;
				const FNav3DCrossVolumeGraph& Cvg = Ta->GetCrossVolumeGraph();
				const TArray<ANav3DDataChunkActor*>& Actors = Cvg.GetCachedChunkActors();
				int32 CurChunkIdx = INDEX_NONE;
				for (int32 i = 0; i < Actors.Num(); ++i) { if (Actors[i] == Current->Actor) { CurChunkIdx = i; break; } }
				if (CurChunkIdx == INDEX_NONE) { continue; }
				if (Current->Actor->Nav3DChunks.Num() == 0 || !Current->Actor->Nav3DChunks[0]) { continue; }
				for (const UNav3DDataChunk* CurChunk = Current->Actor->Nav3DChunks[0];
					const FNav3DEdgeVoxel& Edge : CurChunk->BoundaryVoxels)
				{
					FNav3DVoxelID Id; Id.ChunkIndex = CurChunkIdx; Id.VolumeIndex = Edge.VolumeIndex; Id.Layer = Edge.LayerIndex; Id.Morton = Edge.Morton;
					TArray<FNav3DCrossVolumeConnection> Connections; Cvg.GetNeighbors(Id, Connections);
					for (const FNav3DCrossVolumeConnection& CvConn : Connections)
					{
						if (!Actors.IsValidIndex(CvConn.RemoteVoxel.ChunkIndex)) { continue; }
						ANav3DDataChunkActor* NextActor = Actors[CvConn.RemoteVoxel.ChunkIndex];
						if (!NextActor || NextActor == Current->Actor) { continue; }
						if (Closed.Contains(NextActor)) { continue; }

						ValidNeighbors++;
						const float EdgeCost = CvConn.ConnectionCost;
						FNode* Next = GetOrAddNode(NextActor);
						const float TentativeG = Current->G + EdgeCost;
						const float H = Heuristic(NextActor->DataChunkActorBounds, GoalActor->DataChunkActorBounds);
						const float TentativeF = TentativeG + H;

						if (Next->Parent == nullptr || TentativeF < Next->F)
						{
							Next->Parent = Current;
							Next->G = TentativeG;
							Next->F = TentativeF;
							Next->bHasPortal = true;
							Next->PortalFromParent.From = Current->Actor;
							Next->PortalFromParent.To = NextActor;
							if (!Open.Contains(Next)) { Open.Add(Next); }
						}
					}
				}
				bUsedCrossVolumeGraph = true;
				break;
			}
		}

		if (!bUsedCrossVolumeGraph)
			// ReSharper disable once CppUseStructuredBinding
			for (const FNav3DChunkAdjacency& Adj : Current->Actor->ChunkAdjacency)
		{
			ANav3DDataChunkActor* NextActor = Adj.OtherChunkActor.Get();
			if (!NextActor)
			{
				UE_LOG(LogNav3D, Verbose, TEXT("FindActorPath: Skipping null adjacent actor (soft reference not resolved)"));
				continue;
			}
			if (Closed.Contains(NextActor))
			{
				UE_LOG(LogNav3D, Verbose, TEXT("FindActorPath: Skipping already closed actor: %s"), *NextActor->GetName());
				continue;
			}
			
			ValidNeighbors++;
			UE_LOG(LogNav3D, Verbose, TEXT("FindActorPath: Processing adjacent actor: %s with %d connections"), 
			       *NextActor->GetName(), Adj.Connections.Num());
			
			// Choose a representative (minimum-distance) portal connection between actors
			float MinPortalDist = FLT_MAX;
			FNav3DVoxelConnection BestConn{};
			int32 ValidConnections = 0;
			
			for (const FNav3DVoxelConnection& Conn : Adj.Connections)
			{
				// Validate the portal connection
				FString ValidationError;
				bool bIsValid = true;
				
				// Get volume data for validation
				const FNav3DVolumeNavigationData* LocalVolume = nullptr;
				const FNav3DVolumeNavigationData* RemoteVolume = nullptr;
				
				if (Current->Actor->Nav3DChunks.Num() > 0 && Current->Actor->Nav3DChunks[0])
				{
					LocalVolume = Current->Actor->Nav3DChunks[0]->GetVolumeNavigationData();
				}
				
				if (NextActor->Nav3DChunks.Num() > 0 && NextActor->Nav3DChunks[0])
				{
					RemoteVolume = NextActor->Nav3DChunks[0]->GetVolumeNavigationData();
				}
				
				if (LocalVolume && RemoteVolume)
				{
					bIsValid = FNav3DUtils::ValidatePortalConnection(Conn, *LocalVolume, *RemoteVolume, ValidationError);
				}
				
				if (!bIsValid)
				{
					UE_LOG(LogNav3D, Warning, TEXT("FindActorPath: Invalid portal connection between %s and %s: %s"), 
					       *Current->Actor->GetName(), *NextActor->GetName(), *ValidationError);
					continue;
				}
				
				ValidConnections++;
				if (Conn.Distance < MinPortalDist)
				{
					MinPortalDist = Conn.Distance;
					BestConn = Conn;
				}
			}
			
			if (ValidConnections == 0)
			{
				UE_LOG(LogNav3D, Warning, TEXT("FindActorPath: No valid portal connections between %s and %s"), 
				       *Current->Actor->GetName(), *NextActor->GetName());
				continue;
			}
			
			UE_LOG(LogNav3D, VeryVerbose, TEXT("FindActorPath: Found %d valid connections out of %d total between %s and %s"), 
			       ValidConnections, Adj.Connections.Num(), *Current->Actor->GetName(), *NextActor->GetName());
			const float EdgeCost = MinPortalDist < FLT_MAX ? MinPortalDist : 1.0f;
			FNode* Next = GetOrAddNode(NextActor);
			const float TentativeG = Current->G + EdgeCost;
			const float H = Heuristic(NextActor->DataChunkActorBounds, GoalActor->DataChunkActorBounds);
			const float TentativeF = TentativeG + H;
			
			UE_LOG(LogNav3D, Verbose, TEXT("FindActorPath: Neighbor %s - EdgeCost=%f, TentativeG=%f, H=%f, TentativeF=%f"), 
			       *NextActor->GetName(), EdgeCost, TentativeG, H, TentativeF);
			
			if (Next->Parent == nullptr || TentativeF < Next->F)
			{
				Next->Parent = Current;
				Next->G = TentativeG;
				Next->F = TentativeF;
				// Store chosen portal for reconstruction
				Next->bHasPortal = MinPortalDist < FLT_MAX;
				Next->PortalFromParent.From = Current->Actor;
				Next->PortalFromParent.To = NextActor;
				Next->PortalFromParent.Connection = BestConn;
				
				// Add to open list if not already there
				if (!Open.Contains(Next))
				{
					Open.Add(Next);
				}
				
				UE_LOG(LogNav3D, Verbose, TEXT("FindActorPath: Added/updated neighbor %s to open list"), *NextActor->GetName());
			}
		}
		
		UE_LOG(LogNav3D, Verbose, TEXT("FindActorPath: Processed %d valid neighbors, open list now has %d nodes"), 
		       ValidNeighbors, Open.Num());
	}
	
	if (IterationCount >= MaxIterations)
	{
		UE_LOG(LogNav3D, Warning, TEXT("FindActorPath: A* search exceeded maximum iterations (%d)"), MaxIterations);
	}
	else
	{
		UE_LOG(LogNav3D, Warning, TEXT("FindActorPath: A* search exhausted open list after %d iterations without finding goal"), IterationCount);
	}

	// Clean up allocated nodes
	for (auto& Pair : Nodes)
	{
		delete Pair.Value;
	}

	return false;
}