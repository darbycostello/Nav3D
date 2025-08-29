#include "Pathfinding/Stepper/Nav3DPathStepperLazyThetaStar.h"
#include "Pathfinding/Search/Nav3DPathFindingSearch.h"
#include "Pathfinding/Stepper/Nav3DPathStepper.h"

FNav3DPathStepperLazyThetaStar::FNav3DPathStepperLazyThetaStar(
	const FNav3DPathFindingParameters& Parameters,
	const FNav3DPathStepperThetaStarParameters
	& ThetaStarParameters)
	: FNav3DPathStepperThetaStar(Parameters, ThetaStarParameters)
{
}

ENav3DPathStepperStatus FNav3DPathStepperLazyThetaStar::ProcessSingleNode(
	EGraphAStarResult& Result)
{
	if (Graph.OpenList.Num() == 0)
	{
		State = ENav3DPathFindingState::Ended;
		Result = SearchFail;
		return ENav3DPathStepperStatus::MustContinue;
	}

	ConsideredNodeIndex = Graph.OpenList.PopIndex();
	auto* ConsideredNodeUnsafe = &Graph.NodePool[ConsideredNodeIndex];
	ConsideredNodeUnsafe->MarkClosed();

	FillNodeAddressNeighbours(ConsideredNodeUnsafe->NodeRef);

	if (ConsideredNodeUnsafe->ParentNodeIndex != INDEX_NONE &&
		!HasLineOfSight(ConsideredNodeUnsafe->ParentRef,
		                ConsideredNodeUnsafe->NodeRef))
	{
		auto MinTraversalCost = TNumericLimits<float>::Max();
		for (auto Index = 0; Index < Neighbours.Num(); ++Index)
		{
			const auto NeighbourAddress = Neighbours[Index];
			auto& NeighbourNode = Graph.NodePool.FindOrAdd(NeighbourAddress);
			ConsideredNodeUnsafe = &Graph.NodePool[ConsideredNodeIndex];

			if (!NeighbourNode.bIsClosed)
			{
				continue;
			}

			const auto TraversalCost =
				NeighbourNode.TraversalCost +
				GetTraversalCost(NeighbourNode.NodeRef, ConsideredNodeUnsafe->NodeRef);
			const float HeuristicCost =
				NeighbourNode.NodeRef != Parameters.EndNodeAddress
					? GetHeuristicCost(NeighbourNode.NodeRef,
					                   Parameters.EndNodeAddress)
					: 0.f;
			if (MinTraversalCost > TraversalCost)
			{
				MinTraversalCost = TraversalCost;
				ConsideredNodeUnsafe->TraversalCost = TraversalCost;
				ConsideredNodeUnsafe->TotalCost = TraversalCost + HeuristicCost;
				ConsideredNodeUnsafe->ParentRef = NeighbourAddress;
				ConsideredNodeUnsafe->ParentNodeIndex = NeighbourNode.SearchNodeIndex;
			}
		}
	}

	if (ConsideredNodeUnsafe->NodeRef == Parameters.EndNodeAddress)
	{
		BestNodeIndex = ConsideredNodeUnsafe->SearchNodeIndex;
		BestNodeCost = 0.0f;
		State = ENav3DPathFindingState::Ended;
		Result = SearchSuccess;
	}
	else
	{
		NeighbourIndex = 0;

		State = ENav3DPathFindingState::ProcessNeighbour;

		for (const auto Processor : Processors)
		{
			Processor->ProcessNode(*ConsideredNodeUnsafe);
		}
	}

	return ENav3DPathStepperStatus::MustContinue;
}

ENav3DPathStepperStatus
FNav3DPathStepperLazyThetaStar::ProcessNeighbour(
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

	// Again, let's take a pointer as we call FindOrAdd below
	const auto* CurrentNode = &Graph.NodePool[ConsideredNodeIndex];

	if (!Graph.Graph.IsValidRef(NeighbourAddress) ||
		NeighbourAddress == CurrentNode->ParentRef ||
		NeighbourAddress == CurrentNode->NodeRef)
	{
		return ENav3DPathStepperStatus::MustContinue;
	}

	auto& NeighbourNode = Graph.NodePool.FindOrAdd(NeighbourAddress);

	if (NeighbourNode.bIsClosed)
	{
		return ENav3DPathStepperStatus::MustContinue;
	}

	// Time to refresh
	CurrentNode = &Graph.NodePool[ConsideredNodeIndex];

	float NewTraversalCost;
	const auto NewHeuristicCost =
		NeighbourNode.NodeRef != Parameters.EndNodeAddress
			? GetHeuristicCost(NeighbourNode.NodeRef, Parameters.EndNodeAddress)
			: 0.f;

	const auto CurrentNodeSearchIndex = CurrentNode->SearchNodeIndex;
	const auto& CurrentNodeAddress = CurrentNode->NodeRef;
	const auto ParentNodeIndex = CurrentNode->ParentNodeIndex;
	const auto ParentIndex = ParentNodeIndex == INDEX_NONE ? 0 : ParentNodeIndex;
	const auto& ParentNode = Graph.NodePool[ParentIndex];

	FNav3DNodeAddress NeighbourParentAddress;
	int32 NeighbourParentSearchNodeIndex;

	if (ParentNodeIndex != INDEX_NONE)
	{
		NewTraversalCost =
			ParentNode.TraversalCost +
			GetTraversalCost(ParentNode.NodeRef, NeighbourNode.NodeRef);
		NeighbourParentAddress = ParentNode.NodeRef;
		NeighbourParentSearchNodeIndex = ParentNode.SearchNodeIndex;
	}
	else
	{
		NewTraversalCost =
			CurrentNode->TraversalCost +
			GetTraversalCost(CurrentNodeAddress, NeighbourNode.NodeRef);
		NeighbourParentAddress = CurrentNodeAddress;
		NeighbourParentSearchNodeIndex = CurrentNodeSearchIndex;
	}

	const auto NewTotalCost = AdjustTotalCostWithNodeSizeCompensation(
		NewTraversalCost + NewHeuristicCost, NeighbourAddress);

	if (NewTotalCost >= NeighbourNode.TotalCost)
	{
		for (const auto Processor : Processors)
		{
			Processor->ProcessNeighborEvaluation(*CurrentNode, NeighbourAddress, NewTotalCost);
		}

		return ENav3DPathStepperStatus::MustContinue;
	}

	NeighbourNode.TraversalCost = NewTraversalCost;
	ensure(NewTraversalCost > 0);
	NeighbourNode.TotalCost = NewTotalCost;
	NeighbourNode.ParentRef = NeighbourParentAddress;
	NeighbourNode.ParentNodeIndex = NeighbourParentSearchNodeIndex;
	NeighbourNode.MarkNotClosed();

	if (NeighbourNode.IsOpened() == false)
	{
		Graph.OpenList.Push(NeighbourNode);
	}

	for (const auto Processor : Processors)
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
