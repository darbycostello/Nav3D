#include "Pathfinding/Search/Nav3DPathHeuristicCalculator.h"
#include "Nav3DVolumeNavigationData.h"

float UNav3DPathHeuristicCalculator_Manhattan::GetHeuristicCost(
	const FNav3DVolumeNavigationData& BoundsData,
	const FNav3DNodeAddress& Start, const FNav3DNodeAddress& End) const
{
	const auto StartLocation = BoundsData.GetNodePositionFromAddress(Start, true);
	const auto EndLocation = BoundsData.GetNodePositionFromAddress(End, true);
	const auto Score = FMath::Abs(EndLocation.X - StartLocation.X) +
		FMath::Abs(EndLocation.Y - StartLocation.Y) +
		FMath::Abs(EndLocation.Z - StartLocation.Z);
	return Score;
}

float UNav3DPathHeuristicCalculator_Euclidean::GetHeuristicCost(
	const FNav3DVolumeNavigationData& BoundsData,
	const FNav3DNodeAddress& Start, const FNav3DNodeAddress& End) const
{
	const auto StartLocation = BoundsData.GetNodePositionFromAddress(Start, true);
	const auto EndLocation = BoundsData.GetNodePositionFromAddress(End, true);
	const auto Score = (StartLocation - EndLocation).Size();

	return Score;
}
