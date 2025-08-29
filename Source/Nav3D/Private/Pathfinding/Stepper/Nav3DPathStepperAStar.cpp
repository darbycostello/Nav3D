#include "Pathfinding/Stepper/Nav3DPathStepperAStar.h"
#include "Nav3DUtils.h"
#include "Nav3DVolumeNavigationData.h"
#include "Pathfinding/Search/Nav3DAStar.h"
#include "Pathfinding/Nav3DPathBuilder.h"
#include "Pathfinding/Nav3DQueryFilterSettings.h"
#include "Pathfinding/Search/Nav3DGraphAStar.h"
#include "Pathfinding/Stepper/Nav3DPathDebug.h"

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

	const auto NeighbourPosition = Parameters.VolumeNavigationData.GetNodePositionFromAddress(NeighbourAddress, true);
	const float NeighbourExtent = Parameters.VolumeNavigationData.GetNodeExtentFromNodeAddress(NeighbourAddress);
	if (Parameters.VolumeNavigationData.IsPositionOccluded(NeighbourPosition, NeighbourExtent))
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

ENavigationQueryResult::Type UNav3DAStar::GetPath(
	FNav3DPath& NavigationPath,
	const FNav3DPathFindingParameters& Params) const
{
	FNav3DPathStepperAStar Stepper(Params);
	const auto PathBuilder = MakeShared<FNav3DPathBuilder>(
		NavigationPath, Stepper);

	Stepper.AddProcessor(PathBuilder);

	int Iterations = 0;

	EGraphAStarResult Result = SearchFail;
	while (Stepper.Step(Result) ==
		ENav3DPathStepperStatus::MustContinue)
	{
		Iterations++;
	}

	return FNav3DUtils::GraphAStarResultToNavigationTypeResult(Result);
}

TSharedPtr<FNav3DPathStepper> UNav3DAStar::GetDebugPathStepper(
	FNav3DPathFinderDebugData& DebugData,
	const FNav3DPathFindingParameters Params) const
{
	auto Stepper = MakeShared<FNav3DPathStepperAStar>(Params);
	const auto DebugPath =
		MakeShared<FNav3DPathDebug>(DebugData, Stepper.Get());
	Stepper->AddProcessor(DebugPath);

	return Stepper;
}
