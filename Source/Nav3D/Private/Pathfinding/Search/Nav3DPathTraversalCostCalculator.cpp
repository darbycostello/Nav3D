#include "Pathfinding/Search/Nav3DPathTraversalCostCalculator.h"
#include "Nav3DVolumeNavigationData.h"

float UNav3DPathCostCalculator_Distance::GetTraversalCost(
	const FNav3DVolumeNavigationData& BoundsData,
	const FNav3DNodeAddress& Start, const FNav3DNodeAddress& End) const
{
	const auto StartLocation = BoundsData.GetNodePositionFromAddress(Start, true);
	const auto EndLocation = BoundsData.GetNodePositionFromAddress(End, true);
	const auto Cost = (StartLocation - EndLocation).Size();

	return Cost;
}

UNav3DPathCostCalculator_Fixed::UNav3DPathCostCalculator_Fixed() : Cost(1.0f)
{
}

float UNav3DPathCostCalculator_Fixed::GetTraversalCost(
	const FNav3DVolumeNavigationData& /*BoundsData*/,
	const FNav3DNodeAddress& /*Start*/,
	const FNav3DNodeAddress& /*End */) const
{
	return Cost;
}
