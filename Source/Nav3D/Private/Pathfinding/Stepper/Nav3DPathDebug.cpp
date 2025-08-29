#include "Pathfinding/Stepper/Nav3DPathDebug.h"

#include "Nav3D.h"
#include "Pathfinding/Stepper/Nav3DPathStepper.h"
#include "Pathfinding/Nav3DPathFinder.h"
#include "Pathfinding/Search/Nav3DPathFindingSearch.h"
#include "Pathfinding/Nav3DPathSmoothing.h"
#include "Pathfinding/Nav3DQueryFilterSettings.h"

FNav3DPathDebug::FNav3DPathDebug(
	FNav3DPathFinderDebugData& DebugData,
	const FNav3DPathStepper& Stepper)
	: FNav3DPathFindingProcessor(Stepper)
	  , DebugData(DebugData)
{
}

void FNav3DPathDebug::ProcessNode(
	const FGraphAStarDefaultNode<FNav3DVolumeNavigationData>& Node)
{
	UpdateProcessedNodeData(Node);
	UpdateDebugIterationData();
}

void FNav3DPathDebug::ProcessNeighborEvaluation(
	const FGraphAStarDefaultNode<FNav3DVolumeNavigationData>& Parent,
	const FGraphAStarDefaultNode<FNav3DVolumeNavigationData>& Neighbor,
	float Cost)
{
	const auto& NavData = Stepper.GetParameters().VolumeNavigationData;

	DebugData.ProcessedNeighbours.Emplace(
		FNav3DNodeAddressWithLocation(Parent.NodeRef, NavData),
		FNav3DNodeAddressWithLocation(Neighbor.NodeRef, NavData),
		Cost,
		true);

	DebugData.VisitedNodes++;

	UE_LOG(LogNav3D, Verbose,
	       TEXT("Processing Neighbor Evaluation: From [%s] To [%s] Cost [%s]"),
	       *Parent.NodeRef.ToString(),
	       *Neighbor.NodeRef.ToString(),
	       *FString::SanitizeFloat(Cost));
}

void FNav3DPathDebug::ProcessNeighborSelection(
	const FGraphAStarDefaultNode<FNav3DVolumeNavigationData>& Neighbor)
{
	const auto& NavData = Stepper.GetParameters().VolumeNavigationData;

	DebugData.ProcessedNeighbours.Emplace(
		FNav3DNodeAddressWithLocation(Neighbor.ParentRef, NavData),
		FNav3DNodeAddressWithLocation(Neighbor.NodeRef, NavData),
		Neighbor.TotalCost,
		false);

	DebugData.VisitedNodes++;

	UE_LOG(LogNav3D, Verbose,
	       TEXT("Processing Neighbor Selection: From [%s] To [%s] Cost [%s]"),
	       *Neighbor.ParentRef.ToString(),
	       *Neighbor.NodeRef.ToString(),
	       *FString::SanitizeFloat(Neighbor.TotalCost));
}

void FNav3DPathDebug::ProcessFinalPath(
	const TArray<FNav3DPathFinderNodeAddress>& NodeAddresses)
{
	UpdateCurrentBestPath(NodeAddresses, true);
	UpdatePathMetrics();
}

void FNav3DPathDebug::UpdateProcessedNodeData(
	const FGraphAStarDefaultNode<FNav3DVolumeNavigationData>& Node) const
{
	const auto& Params = Stepper.GetParameters();

	if (Node.ParentRef.IsValid())
	{
		DebugData.LastProcessedSingleNode.From = FNav3DNodeAddressWithLocation(
			Node.ParentRef, Params.VolumeNavigationData);
	}
	else
	{
		DebugData.LastProcessedSingleNode.From = FNav3DNodeAddressWithLocation(
			FNav3DNodeAddress::InvalidAddress,
			Params.StartLocation);
	}

	DebugData.LastProcessedSingleNode.To = FNav3DNodeAddressWithLocation(
		Node.NodeRef, Params.VolumeNavigationData);
	DebugData.LastProcessedSingleNode.Cost = Node.TotalCost;

	UE_LOG(LogNav3D, Verbose,
	       TEXT("Processing Node: From [%s] To [%s] Cost [%s]"),
	       *Node.NodeRef.ToString(),
	       *DebugData.LastProcessedSingleNode.To.NodeAddress.ToString(),
	       *FString::SanitizeFloat(Node.TotalCost));

	DebugData.ProcessedNeighbours.Reset();
}

void FNav3DPathDebug::UpdateDebugIterationData() const
{
	DebugData.Iterations++;

	TArray<FNav3DPathFinderNodeAddress> NodeAddresses;
	Stepper.FillNodeAddresses(NodeAddresses);
	UpdateCurrentBestPath(NodeAddresses, false);
}

void FNav3DPathDebug::UpdateCurrentBestPath(
	const TArray<FNav3DPathFinderNodeAddress>& NodeAddresses,
	bool AddEndLocation) const
{
	DebugData.CurrentBestPath.ResetForRepath();

	const auto& Params = Stepper.GetParameters();
	FNav3DPathFinder::BuildPath(DebugData.CurrentBestPath, Params, NodeAddresses, AddEndLocation);

	if (AddEndLocation && Params.QueryFilterSettings.bSmoothPaths)
	{
		FNav3DPathSmoothing::SmoothPath(
			DebugData.CurrentBestPath,
			Params.QueryFilterSettings.SmoothingSubdivisions);
	}

	DebugData.CurrentBestPath.MarkReady();
}

void FNav3DPathDebug::UpdatePathMetrics() const
{
	auto& NavPathPoints = DebugData.CurrentBestPath.GetPathPoints();
	DebugData.PathSegmentCount = NavPathPoints.Num();

	float PathLength = 0.0f;
	for (int32 Index = 1; Index < NavPathPoints.Num(); ++Index)
	{
		PathLength += FVector::Dist(
			NavPathPoints[Index].Location,
			NavPathPoints[Index - 1].Location);
	}

	DebugData.PathLength = PathLength;
}
