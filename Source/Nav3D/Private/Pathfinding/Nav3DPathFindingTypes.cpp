#include "../../Public/Pathfinding/Nav3DPathFindingTypes.h"
#include "Nav3DVolumeNavigationData.h"
#include "Pathfinding/Nav3DQueryFilter.h"
#include "Pathfinding/Nav3DQueryFilterSettings.h"

FNav3DNodeAddressWithLocation::FNav3DNodeAddressWithLocation(
	const FNav3DNodeAddress& NodeAddress, const FVector& Location)
	: NodeAddress(NodeAddress), Location(Location)
{
}

FNav3DNodeAddressWithLocation::FNav3DNodeAddressWithLocation(
	const FNav3DNodeAddress& NodeAddress,
	const FNav3DVolumeNavigationData& BoundsNavigationData)
	: NodeAddress(NodeAddress),
	  Location(
		  BoundsNavigationData.GetNodePositionFromAddress(NodeAddress, true))
{
}

FNav3DPathFinderDebugNodeCost::FNav3DPathFinderDebugNodeCost(
	const FNav3DNodeAddressWithLocation& From,
	const FNav3DNodeAddressWithLocation& To, const float Cost,
	const bool IsClosed)
	: From(From), To(To), Cost(Cost), bIsClosed(IsClosed)
{
}

void FNav3DPathFinderDebugNodeCost::Reset()
{
	*this = FNav3DPathFinderDebugNodeCost();
}

FNav3DPathFinderNodeAddress::FNav3DPathFinderNodeAddress(
	const FNav3DNodeAddress& NodeAddress, const double TraversalCost)
	: NodeAddress(NodeAddress), Cost(TraversalCost)
{
}

FNav3DPathFinderDebugData::FNav3DPathFinderDebugData()
	: LastProcessedSingleNode(), Iterations(0), VisitedNodes(0),
	  PathSegmentCount(0), PathLength(0)
{
}

void FNav3DPathFinderDebugData::Reset()
{
	LastProcessedSingleNode.Reset();
	ProcessedNeighbours.Reset();
	Iterations = 0;
	VisitedNodes = 0;
	CurrentBestPath.ResetForRepath();
}

FNav3DPathFindingParameters::FNav3DPathFindingParameters(
	const FNav3DVolumeNavigationData& VolumeNavigationData,
	const FVector& StartLocation, const FVector& EndLocation,
	const FNavigationQueryFilter& NavQueryFilter)
	: StartLocation(StartLocation), EndLocation(EndLocation),
	  NavigationQueryFilter(NavQueryFilter),
	  QueryFilterImplementation(static_cast<const FNav3DQueryFilter*>(
		  NavigationQueryFilter.GetImplementation())),
	  QueryFilterSettings(QueryFilterImplementation->QueryFilterSettings),
	  HeuristicCalculator(QueryFilterSettings.HeuristicCalculator),
	  CostCalculator(QueryFilterSettings.TraversalCostCalculator),
	  VolumeNavigationData(VolumeNavigationData)
{
}

TOptional<FNav3DPathFindingParameters> FNav3DPathFindingParameters::Initialize(
	const FNav3DVolumeNavigationData& VolumeNavigationData,
	const FVector& StartLocation, const FVector& EndLocation,
	const FNavigationQueryFilter& NavQueryFilter,
	const LayerIndex& MinLayerIndex)
{
	if (auto Result = FNav3DPathFindingParameters(
			VolumeNavigationData, StartLocation, EndLocation, NavQueryFilter);
		VolumeNavigationData.GetNodeAddressFromPosition(
			Result.StartNodeAddress, StartLocation, MinLayerIndex))
	{
		if (VolumeNavigationData.GetNodeAddressFromPosition(
			Result.EndNodeAddress, EndLocation, MinLayerIndex))
		{
			return Result;
		}
	}

	return TOptional<FNav3DPathFindingParameters>();
}
