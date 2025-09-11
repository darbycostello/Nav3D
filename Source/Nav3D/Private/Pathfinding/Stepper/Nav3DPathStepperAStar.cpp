#include "Pathfinding/Stepper/Nav3DPathStepperAStar.h"
#include "Nav3DUtils.h"
#include "Nav3DVolumeNavigationData.h"
#include "Nav3DDataChunkActor.h"
#include "Nav3DData.h"
#include "Nav3DTacticalActor.h"
#include "Nav3DTypes.h"
#include "Pathfinding/Nav3DCrossVolumeGraph.h"
#include "Pathfinding/Nav3DPathBuilder.h"
#include "Pathfinding/Nav3DQueryFilterSettings.h"
// Search algorithm implementation is in Search/Nav3DAStar.cpp

FNav3DPathStepperAStar::FNav3DPathStepperAStar(
	const FNav3DPathFindingParameters& Parameters)
	: FNav3DPathStepper(Parameters)
	  , ConsideredNodeIndex(INDEX_NONE)
	  , BestNodeIndex(INDEX_NONE)
	  , BestNodeCost(-1.0f)
	  , NeighbourIndex(INDEX_NONE)
{
}

bool FNav3DPathStepperAStar::FillNodeAddresses(
	TArray<FNav3DPathFinderNodeAddress>& NodeAddresses) const
{
	int32 SearchNodeIndex = BestNodeIndex;
	int32 PathLength = 0;
	do
	{
		PathLength++;
		SearchNodeIndex = Graph.NodePool[SearchNodeIndex].ParentNodeIndex;
	}
	while (Graph.NodePool.IsValidIndex(SearchNodeIndex) &&
		Graph.NodePool[SearchNodeIndex].NodeRef !=
		Parameters.StartNodeAddress &&
		ensure(PathLength < FGraphAStarDefaultPolicy::FatalPathLength));

	if (PathLength >= FGraphAStarDefaultPolicy::FatalPathLength)
	{
		return false;
	}

	// Same as FGraphAStar except we add the start node address as the first node,
	// since it is different from where the start location is
	NodeAddresses.Reset(PathLength + 1);
	NodeAddresses.AddZeroed(PathLength + 1);

	SearchNodeIndex = BestNodeIndex;
	int32 ResultNodeIndex = PathLength;
	do
	{
		const auto& Node = Graph.NodePool[SearchNodeIndex];
		NodeAddresses[ResultNodeIndex--] = {Node.NodeRef, Node.TraversalCost};
		SearchNodeIndex = Node.ParentNodeIndex;
	}
	while (ResultNodeIndex >= 0 && SearchNodeIndex != INDEX_NONE);

	NodeAddresses[0] = {Parameters.StartNodeAddress, 0.0f};

	return true;
}

ENav3DPathStepperStatus
FNav3DPathStepperAStar::Init(EGraphAStarResult& Result)
{
	if (!(Graph.Graph.IsValidRef(Parameters.StartNodeAddress) &&
		Graph.Graph.IsValidRef(Parameters.EndNodeAddress)))
	{
		Result = SearchFail;
		return ENav3DPathStepperStatus::IsStopped;
	}

	if (Parameters.StartNodeAddress == Parameters.EndNodeAddress)
	{
		Result = SearchSuccess;
		return ENav3DPathStepperStatus::IsStopped;
	}

	Graph.NodePool.Reset();
	Graph.OpenList.Reset();

	// kick off the search with the first node
	auto& StartNode = Graph.NodePool.Add(
		FNav3DGraphAStar::FSearchNode(Parameters.StartNodeAddress));
	StartNode.ParentRef.Invalidate();
	StartNode.TraversalCost = 0;
	StartNode.TotalCost =
		GetHeuristicCost(Parameters.StartNodeAddress, Parameters.EndNodeAddress);

	Graph.OpenList.Push(StartNode);

	BestNodeIndex = StartNode.SearchNodeIndex;
	BestNodeCost = StartNode.TotalCost;

	if (Graph.OpenList.Num() == 0)
	{
		SetState(ENav3DPathFindingState::Ended);
	}
	else
	{
		SetState(ENav3DPathFindingState::ProcessNode);
	}

	return ENav3DPathStepperStatus::MustContinue;
}

void FNav3DPathStepperAStar::FillNodeAddressNeighbours(
	const FNav3DNodeAddress& NodeAddress)
{
	Neighbours.Reset();
	Graph.Graph.GetNodeNeighbours(Neighbours, NodeAddress);

	// Append baked cross-actor portal neighbours when on a boundary leaf
	const FNav3DNode& Node = Graph.Graph.GetNodeFromAddress(NodeAddress);
	const FVector NodeWorldPos = Graph.Graph.GetNodePositionFromAddress(NodeAddress, true);
	const FBox& CurrentBounds = Graph.Graph.GetData().GetNavigationBounds();
	const float Tolerance = Graph.Graph.GetData().GetLeafNodes().GetLeafNodeExtent() * 0.5f;

	const bool bOnBoundary =
		FMath::Abs(NodeWorldPos.X - CurrentBounds.Min.X) < Tolerance ||
		FMath::Abs(NodeWorldPos.X - CurrentBounds.Max.X) < Tolerance ||
		FMath::Abs(NodeWorldPos.Y - CurrentBounds.Min.Y) < Tolerance ||
		FMath::Abs(NodeWorldPos.Y - CurrentBounds.Max.Y) < Tolerance ||
		FMath::Abs(NodeWorldPos.Z - CurrentBounds.Min.Z) < Tolerance ||
		FMath::Abs(NodeWorldPos.Z - CurrentBounds.Max.Z) < Tolerance;

	if (bOnBoundary && NodeAddress.LayerIndex == 0)
	{
		if (const UWorld* World = Graph.Graph.GetDataGenerationSettings().World)
		{
			if (const ANav3DData* NavData = FNav3DUtils::GetNav3DData(World))
			{
				const ANav3DTacticalActor* TacticalActorForPos = nullptr;
				for (const ANav3DTacticalActor* Ta : NavData->GetAllTacticalActors())
				{
					if (Ta && Ta->ContainsPoint(NodeWorldPos)) { TacticalActorForPos = Ta; break; }
				}
				if (TacticalActorForPos)
				{
					const FNav3DCrossVolumeGraph& Cvg = TacticalActorForPos->GetCrossVolumeGraph();
					if (const int32 ChunkIdx = Cvg.FindChunkIndexForPosition(NodeWorldPos);
						ChunkIdx != INDEX_NONE && Cvg.GetCachedChunkActors().IsValidIndex(ChunkIdx))
					{
						int32 VolumeIdx = INDEX_NONE;
						if (const ANav3DDataChunkActor* ChunkActor = Cvg.GetCachedChunkActors()[ChunkIdx])
						{
							if (ChunkActor->Nav3DChunks.Num() > 0)
							{
								const UNav3DDataChunk* Chunk = ChunkActor->Nav3DChunks[0];
								for (int32 V = 0; V < Chunk->NavigationData.Num(); ++V)
								{
									if (Chunk->NavigationData[V].GetData().GetNavigationBounds() == CurrentBounds) { VolumeIdx = V; break; }
								}
							}
						}
						if (VolumeIdx != INDEX_NONE)
						{
							const MortonCode CurrentMorton = Node.MortonCode;
							FNav3DVoxelID ThisVoxel; ThisVoxel.ChunkIndex = ChunkIdx; ThisVoxel.VolumeIndex = VolumeIdx; ThisVoxel.Layer = NodeAddress.LayerIndex; ThisVoxel.Morton = CurrentMorton;
							TArray<FNav3DCrossVolumeConnection> Connections; Cvg.GetNeighbors(ThisVoxel, Connections);
							for (const FNav3DCrossVolumeConnection& C : Connections)
							{
								const FNav3DVoxelID& Remote = C.RemoteVoxel;
								if (!Cvg.GetCachedChunkActors().IsValidIndex(Remote.ChunkIndex)) { continue; }
								const ANav3DDataChunkActor* RemoteActor = Cvg.GetCachedChunkActors()[Remote.ChunkIndex];
								if (!RemoteActor || RemoteActor->Nav3DChunks.Num() == 0) { continue; }
								const UNav3DDataChunk* RemoteChunk = RemoteActor->Nav3DChunks[0];
								if (!RemoteChunk || !RemoteChunk->NavigationData.IsValidIndex(Remote.VolumeIndex)) { continue; }
								const FNav3DVolumeNavigationData& RemoteVol = RemoteChunk->NavigationData[Remote.VolumeIndex];
								const FVector RemotePos = FNav3DUtils::GetVoxelWorldPosition(Remote, Cvg.GetCachedChunkActors());
								if (FNav3DNodeAddress RemoteAddr; RemoteVol.GetNodeAddressFromPosition(RemoteAddr, RemotePos, Remote.Layer))
								{
									Neighbours.Add(RemoteAddr);
								}
							}
						}
					}
				}
			}
		}
	}

	NeighbourIndex = 0;
}

float FNav3DPathStepperAStar::
AdjustTotalCostWithNodeSizeCompensation(
	const float TotalCost,
	const FNav3DNodeAddress NeighbourNodeAddress) const
{
	if (!Parameters.QueryFilterSettings.bUseNodeSizeCompensation)
	{
		return TotalCost;
	}

	return TotalCost * Parameters.VolumeNavigationData.GetLayerInverseRatio(
		NeighbourNodeAddress.LayerIndex);
}

ENav3DPathStepperStatus
FNav3DPathStepperAStar::ProcessSingleNode(EGraphAStarResult& Result)
{
	if (Graph.OpenList.Num() == 0)
	{
		State = ENav3DPathFindingState::Ended;
		Result = SearchFail;
		return ENav3DPathStepperStatus::MustContinue;
	}

	ConsideredNodeIndex = Graph.OpenList.PopIndex();
	auto& ConsideredNodeUnsafe = Graph.NodePool[ConsideredNodeIndex];
	ConsideredNodeUnsafe.MarkClosed();

	if (ConsideredNodeUnsafe.NodeRef == Parameters.EndNodeAddress)
	{
		BestNodeIndex = ConsideredNodeUnsafe.SearchNodeIndex;
		BestNodeCost = 0.0f;
		State = ENav3DPathFindingState::Ended;
		Result = SearchSuccess;
	}
	else
	{
		FillNodeAddressNeighbours(ConsideredNodeUnsafe.NodeRef);
		State = ENav3DPathFindingState::ProcessNeighbour;

		for (const auto& Processor : Processors)
		{
			Processor->ProcessNode(ConsideredNodeUnsafe);
		}
	}

	return ENav3DPathStepperStatus::MustContinue;
}

ENav3DPathStepperStatus
FNav3DPathStepperAStar::ProcessNeighbour(
	EGraphAStarResult& Result)
{
	FNeighbourIndexIncrement NeighbourIndexIncrement(Neighbours, NeighbourIndex,
	                                                 State);

	if (!Neighbours.IsValidIndex(NeighbourIndex))
	{
		Result = SearchFail;
		return ENav3DPathStepperStatus::IsStopped;
	}

	const auto NeighbourAddress = Neighbours[NeighbourIndex];

	// Check if the neighbour node is navigable using stored occlusion data
	bool bIsNavigable = false;
	if (NeighbourAddress.LayerIndex == 0)
	{
		// For leaf nodes, check if the specific subnode is free
		const auto& LeafNodes = Parameters.VolumeNavigationData.GetData().GetLeafNodes();
		if (LeafNodes.GetLeafNodes().IsValidIndex(NeighbourAddress.NodeIndex))
		{
			const auto& LeafNode = LeafNodes.GetLeafNode(NeighbourAddress.NodeIndex);
			bIsNavigable = !LeafNode.IsSubNodeOccluded(NeighbourAddress.SubNodeIndex);
		}
	}
	else
	{
		// For non-leaf nodes, check if they don't have children (meaning they're free)
		const auto& Node = Parameters.VolumeNavigationData.GetNodeFromAddress(NeighbourAddress);
		bIsNavigable = !Node.HasChildren();
	}
	
	if (!bIsNavigable)
	{
		return ENav3DPathStepperStatus::MustContinue;
	}

	if (!Graph.Graph.IsValidRef(NeighbourAddress) ||
		NeighbourAddress == Graph.NodePool[ConsideredNodeIndex].ParentRef ||
		NeighbourAddress == Graph.NodePool[ConsideredNodeIndex].NodeRef)
	{
		return ENav3DPathStepperStatus::MustContinue;
	}

	auto& NeighbourNode = Graph.NodePool.FindOrAdd(NeighbourAddress);

	if (NeighbourNode.bIsClosed)
	{
		return ENav3DPathStepperStatus::MustContinue;
	}

	const auto NewTraversalCost =
		GetTraversalCost(Graph.NodePool[ConsideredNodeIndex].NodeRef,
		                 NeighbourNode.NodeRef) +
		Graph.NodePool[ConsideredNodeIndex].TraversalCost;
	const auto NewHeuristicCost =
		NeighbourNode.NodeRef != Parameters.EndNodeAddress
			? GetHeuristicCost(NeighbourNode.NodeRef, Parameters.EndNodeAddress)
			: 0.f;
	const auto NewTotalCost = AdjustTotalCostWithNodeSizeCompensation(
		NewTraversalCost + NewHeuristicCost, NeighbourAddress);

	const auto& ConsideredNodeUnsafe = Graph.NodePool[ConsideredNodeIndex];

	if (NewTotalCost >= NeighbourNode.TotalCost)
	{
		for (const auto& Processor : Processors)
		{
			Processor->ProcessNeighborEvaluation(ConsideredNodeUnsafe, NeighbourNode, NewTotalCost);
		}
		return ENav3DPathStepperStatus::MustContinue;
	}

	NeighbourNode.TraversalCost = NewTraversalCost;
	ensure(NewTraversalCost > 0);
	NeighbourNode.TotalCost = NewTotalCost;
	NeighbourNode.ParentRef = Graph.NodePool[ConsideredNodeIndex].NodeRef;
	NeighbourNode.ParentNodeIndex =
		Graph.NodePool[ConsideredNodeIndex].SearchNodeIndex;
	NeighbourNode.MarkNotClosed();

	if (NeighbourNode.IsOpened() == false)
	{
		Graph.OpenList.Push(NeighbourNode);
	}

	for (const auto& Processor : Processors)
	{
		Processor->ProcessNeighborSelection(NeighbourNode);
	}

	if (NewHeuristicCost < BestNodeCost)
	{
		BestNodeCost = NewHeuristicCost;
		BestNodeIndex = NeighbourNode.SearchNodeIndex;
	}

	return ENav3DPathStepperStatus::MustContinue;
}

ENav3DPathStepperStatus
FNav3DPathStepperAStar::Ended(EGraphAStarResult& Result)
{
	if (BestNodeCost != 0.f)
	{
		Result = GoalUnreachable;
	}

	if (Result == SearchSuccess)
	{
		TArray<FNav3DPathFinderNodeAddress> NodeAddresses;

		if (!FillNodeAddresses(NodeAddresses))
		{
			Result = InfiniteLoop;
		}

		for (const auto& Processor : Processors)
		{
			Processor->ProcessFinalPath(NodeAddresses);
		}
	}

	return ENav3DPathStepperStatus::IsStopped;
}

FNav3DPathStepperAStar::FNeighbourIndexIncrement::
FNeighbourIndexIncrement(TArray<FNav3DNodeAddress>& Neighbours,
                         int& NeighbourIndex,
                         ENav3DPathFindingState& State)
	: Neighbours(Neighbours), NeighbourIndex(NeighbourIndex), State(State)
{
}

FNav3DPathStepperAStar::FNeighbourIndexIncrement::
~FNeighbourIndexIncrement()
{
	NeighbourIndex++;

	if (NeighbourIndex >= Neighbours.Num())
	{
		State = ENav3DPathFindingState::ProcessNode;
	}
	else
	{
		State = ENav3DPathFindingState::ProcessNeighbour;
	}
}

// UNav3DAStar::GetPath moved to Search/Nav3DAStar.cpp to match other algorithms