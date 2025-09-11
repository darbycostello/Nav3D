#include "Pathfinding/Nav3DPathFindingTypes.h"
#include "Nav3D.h"
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
	const FVector& StartLocation,
	const FVector& EndLocation,
	const FNavigationQueryFilter& NavQueryFilter,
	const LayerIndex& MinLayerIndex)
{
	UE_LOG(LogNav3D, VeryVerbose, TEXT("FNav3DPathFindingParameters::Initialize: Attempting to initialize with start=%s, end=%s, minLayer=%d"), 
	       *StartLocation.ToString(), *EndLocation.ToString(), MinLayerIndex);
	
	// Check navigation bounds
	const auto& NavigationBounds = VolumeNavigationData.GetNavigationBounds();
	UE_LOG(LogNav3D, VeryVerbose, TEXT("FNav3DPathFindingParameters::Initialize: Navigation bounds: %s"), *NavigationBounds.ToString());
	UE_LOG(LogNav3D, VeryVerbose, TEXT("FNav3DPathFindingParameters::Initialize: Start location inside bounds: %s"), 
	       NavigationBounds.IsInside(StartLocation) ? TEXT("Yes") : TEXT("No"));
	UE_LOG(LogNav3D, VeryVerbose, TEXT("FNav3DPathFindingParameters::Initialize: End location inside bounds: %s"), 
	       NavigationBounds.IsInside(EndLocation) ? TEXT("Yes") : TEXT("No"));
	
	// Try to resolve both locations at the minimum layer first
	if (auto Result = FNav3DPathFindingParameters(
			VolumeNavigationData, StartLocation, EndLocation, NavQueryFilter);
		VolumeNavigationData.GetNodeAddressFromPosition(
			Result.StartNodeAddress, StartLocation, MinLayerIndex))
	{
		UE_LOG(LogNav3D, VeryVerbose, TEXT("FNav3DPathFindingParameters::Initialize: Start address resolved to layer=%d, node=%d, subnode=%d"), 
		       Result.StartNodeAddress.LayerIndex, Result.StartNodeAddress.NodeIndex, Result.StartNodeAddress.SubNodeIndex);
		
		if (VolumeNavigationData.GetNodeAddressFromPosition(
			Result.EndNodeAddress, EndLocation, MinLayerIndex))
		{
			UE_LOG(LogNav3D, VeryVerbose, TEXT("FNav3DPathFindingParameters::Initialize: End address resolved to layer=%d, node=%d, subnode=%d"), 
			       Result.EndNodeAddress.LayerIndex, Result.EndNodeAddress.NodeIndex, Result.EndNodeAddress.SubNodeIndex);
			return Result;
		}
		else
		{
			UE_LOG(LogNav3D, VeryVerbose, TEXT("FNav3DPathFindingParameters::Initialize: End location failed at min layer %d, trying multi-layer resolution"), MinLayerIndex);
		}
	}
	else
	{
		UE_LOG(LogNav3D, VeryVerbose, TEXT("FNav3DPathFindingParameters::Initialize: Start location failed at min layer %d, trying multi-layer resolution"), MinLayerIndex);
	}
	
	// Multi-layer resolution: try to find the highest layer where both locations can be resolved
	// Start from the maximum possible layer and work down to the minimum layer
	const int32 MaxLayer = VolumeNavigationData.GetLayerCount() - 1;
	
	for (int32 Layer = MaxLayer; Layer >= MinLayerIndex; --Layer)
	{
		UE_LOG(LogNav3D, VeryVerbose, TEXT("FNav3DPathFindingParameters::Initialize: Trying multi-layer resolution at layer %d"), Layer);
		
		if (auto Result = FNav3DPathFindingParameters(
				VolumeNavigationData, StartLocation, EndLocation, NavQueryFilter);
			VolumeNavigationData.GetNodeAddressFromPosition(
				Result.StartNodeAddress, StartLocation, Layer))
		{
			UE_LOG(LogNav3D, VeryVerbose, TEXT("FNav3DPathFindingParameters::Initialize: Start address resolved to layer=%d, node=%d, subnode=%d"), 
			       Result.StartNodeAddress.LayerIndex, Result.StartNodeAddress.NodeIndex, Result.StartNodeAddress.SubNodeIndex);
			
			if (VolumeNavigationData.GetNodeAddressFromPosition(
				Result.EndNodeAddress, EndLocation, Layer))
			{
				UE_LOG(LogNav3D, VeryVerbose, TEXT("FNav3DPathFindingParameters::Initialize: End address resolved to layer=%d, node=%d, subnode=%d"), 
				       Result.EndNodeAddress.LayerIndex, Result.EndNodeAddress.NodeIndex, Result.EndNodeAddress.SubNodeIndex);
				UE_LOG(LogNav3D, Verbose, TEXT("FNav3DPathFindingParameters::Initialize: Multi-layer resolution successful at layer %d"), Layer);
				return Result;
			}
			else
			{
				UE_LOG(LogNav3D, VeryVerbose, TEXT("FNav3DPathFindingParameters::Initialize: End location %s failed to resolve at layer %d"), 
				       *EndLocation.ToString(), Layer);
			}
		}
		else
		{
			UE_LOG(LogNav3D, VeryVerbose, TEXT("FNav3DPathFindingParameters::Initialize: Start location %s failed to resolve at layer %d"), 
			       *StartLocation.ToString(), Layer);
		}
	}
	
	UE_LOG(LogNav3D, Warning, TEXT("FNav3DPathFindingParameters::Initialize: Failed to resolve both locations at any layer from %d to %d"), 
	       MinLayerIndex, MaxLayer);

	return TOptional<FNav3DPathFindingParameters>();
}
