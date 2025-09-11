#include "Pathfinding/Stepper/Nav3DPathStepperThetaStar.h"
#include "Pathfinding/Nav3DPathFindingTypes.h"
#include "Pathfinding/Stepper/Nav3DPathStepperAStar.h"
#include "Raycasting/Nav3DRaycaster.h"
#include "Nav3D.h"

FNav3DPathStepperThetaStarParameters::FNav3DPathStepperThetaStarParameters()
	: Raycaster(nullptr)
{
}

FNav3DPathStepperThetaStar::FNav3DPathStepperThetaStar(
	const FNav3DPathFindingParameters& Parameters,
	const FNav3DPathStepperThetaStarParameters
	& ThetaStarParameters)
	: FNav3DPathStepperAStar(Parameters),
	  ThetaStarParameters(ThetaStarParameters), LOSCheckCount(0)
{
}

ENav3DPathStepperStatus FNav3DPathStepperThetaStar::Init(EGraphAStarResult& Result)
{
	LOSCheckCount = 0;
	return FNav3DPathStepperAStar::Init(Result);
}

ENav3DPathStepperStatus FNav3DPathStepperThetaStar::ProcessSingleNode(EGraphAStarResult& Result)
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

ENav3DPathStepperStatus FNav3DPathStepperThetaStar::ProcessNeighbour(EGraphAStarResult& Result)
{
	FNeighbourIndexIncrement NeighbourIndexIncrement(Neighbours, NeighbourIndex,
	                                                 State);

	if (!Neighbours.IsValidIndex(NeighbourIndex))
	{
		Result = SearchFail;
		return ENav3DPathStepperStatus::IsStopped;
	}

	const auto NeighbourNodeAddress = Neighbours[NeighbourIndex];

	if (!Graph.Graph.IsValidRef(NeighbourNodeAddress) ||
		NeighbourNodeAddress == Graph.NodePool[ConsideredNodeIndex].ParentRef ||
		NeighbourNodeAddress == Graph.NodePool[ConsideredNodeIndex].NodeRef)
	{
		return ENav3DPathStepperStatus::MustContinue;
	}

	auto& NeighbourNode = Graph.NodePool.FindOrAdd(NeighbourNodeAddress);

	if (NeighbourNode.bIsClosed)
	{
		return ENav3DPathStepperStatus::MustContinue;
	}

	const auto& CurrentNode = Graph.NodePool[ConsideredNodeIndex];
	const auto& CurrentNodeAddress = CurrentNode.NodeRef;
	const auto& ParentNodeAddress = CurrentNode.ParentRef;
	const auto ParentSearchNodeIndex = CurrentNode.ParentNodeIndex;

	const auto HasLOS =
		ParentSearchNodeIndex == INDEX_NONE
			? false
			: HasLineOfSight(ParentNodeAddress, NeighbourNode.NodeRef);

	float NewTraversalCost;
	const auto NewHeuristicCost =
		NeighbourNode.NodeRef != Parameters.EndNodeAddress
			? GetHeuristicCost(NeighbourNode.NodeRef, Parameters.EndNodeAddress)
			: 0.f;

	const auto ParentIndex =
		ParentSearchNodeIndex == INDEX_NONE ? 0 : ParentSearchNodeIndex;

	const auto& ParentNode = Graph.NodePool[ParentIndex];

	FNav3DNodeAddress NeighbourParentNodeAddress;
	int32 NeighbourParentNodeIndex;

	if (HasLOS)
	{
		NewTraversalCost =
			ParentNode.TraversalCost +
			GetTraversalCost(ParentNode.NodeRef, NeighbourNode.NodeRef);
		NeighbourParentNodeAddress = ParentNode.NodeRef;
		NeighbourParentNodeIndex = ParentNode.SearchNodeIndex;
	}
	else
	{
		NewTraversalCost =
			CurrentNode.TraversalCost +
			GetTraversalCost(CurrentNodeAddress, NeighbourNode.NodeRef);
		NeighbourParentNodeAddress = CurrentNodeAddress;
		NeighbourParentNodeIndex = CurrentNode.SearchNodeIndex;
	}

	const auto NewTotalCost = AdjustTotalCostWithNodeSizeCompensation(
		NewTraversalCost + NewHeuristicCost, NeighbourNodeAddress);

	const auto& ConsideredNodeUnsafe = Graph.NodePool[ConsideredNodeIndex];

	if (NewTotalCost >= NeighbourNode.TotalCost)
	{
		for (const auto Processor : Processors)
		{
			Processor->ProcessNeighborEvaluation(ConsideredNodeUnsafe, NeighbourNodeAddress,
			                                     NewTotalCost);
		}

		return ENav3DPathStepperStatus::MustContinue;
	}

	NeighbourNode.TraversalCost = NewTraversalCost;
	ensure(NewTraversalCost > 0);
	NeighbourNode.TotalCost = NewTotalCost;
	NeighbourNode.ParentRef = NeighbourParentNodeAddress;
	NeighbourNode.ParentNodeIndex = NeighbourParentNodeIndex;
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

ENav3DPathStepperStatus FNav3DPathStepperThetaStar::Ended(EGraphAStarResult& Result)
{
	UE_LOG(LogNav3D, Verbose, TEXT("LOSCheckCount : %i"), LOSCheckCount);
	return FNav3DPathStepperAStar::Ended(Result);
}

bool FNav3DPathStepperThetaStar::HasLineOfSight(
	const FNav3DNodeAddress From, const FNav3DNodeAddress To) const
{
	const auto* Raycaster = ThetaStarParameters.Raycaster;

	if (Raycaster == nullptr)
	{
		Raycaster = NewObject<UNav3DRaycaster>();
	}

	const auto GetAdjustedPosition = [this](const FNav3DNodeAddress& Address)
	{
		if (Address == Parameters.StartNodeAddress)
		{
			return Parameters.StartLocation;
		}
		if (Address == Parameters.EndNodeAddress)
		{
			return Parameters.EndLocation;
		}
		return Parameters.VolumeNavigationData.GetNodePositionFromAddress(Address, true);
	};

	const auto FromPosition = GetAdjustedPosition(From);
	const auto ToPosition = GetAdjustedPosition(To);

	// Check if either endpoint is navigable using stored occlusion data
	bool bFromNavigable = false;
	bool bToNavigable = false;
	
	// Check From node
	if (From.LayerIndex == 0)
	{
		const auto& LeafNodes = Parameters.VolumeNavigationData.GetData().GetLeafNodes();
		if (LeafNodes.GetLeafNodes().IsValidIndex(From.NodeIndex))
		{
			const auto& LeafNode = LeafNodes.GetLeafNode(From.NodeIndex);
			bFromNavigable = !LeafNode.IsSubNodeOccluded(From.SubNodeIndex);
		}
	}
	else
	{
		const auto& Node = Parameters.VolumeNavigationData.GetNodeFromAddress(From);
		bFromNavigable = !Node.HasChildren();
	}
	
	// Check To node
	if (To.LayerIndex == 0)
	{
		const auto& LeafNodes = Parameters.VolumeNavigationData.GetData().GetLeafNodes();
		if (LeafNodes.GetLeafNodes().IsValidIndex(To.NodeIndex))
		{
			const auto& LeafNode = LeafNodes.GetLeafNode(To.NodeIndex);
			bToNavigable = !LeafNode.IsSubNodeOccluded(To.SubNodeIndex);
		}
	}
	else
	{
		const auto& Node = Parameters.VolumeNavigationData.GetNodeFromAddress(To);
		bToNavigable = !Node.HasChildren();
	}
	
	if (!bFromNavigable || !bToNavigable)
	{
		return false;
	}

	// Use raycaster to check for occlusion along the path
	FNav3DRaycastHit Hit;
	const bool bHit = Raycaster->Trace(Parameters.VolumeNavigationData,
	                                   FromPosition, ToPosition, Hit);

	// If we hit something, there's no line of sight
	return !bHit;
}
