#include "Pathfinding/Core/Nav3DPath.h"

FVector::FReal FNav3DPath::GetCostFromNode(NavNodeRef PathNode) const
{
	const auto Index = PathPoints.IndexOfByPredicate(
		[&PathNode](const FNavPathPoint& NavPathPoint)
		{
			return NavPathPoint.NodeRef == PathNode;
		});

	return GetCostFromIndex(Index);
}

FVector::FReal FNav3DPath::GetCostFromIndex(const int32 PathPointIndex) const
{
	if (PathPointIndex < 0 || PathPointIndex >= PathPointCosts.Num())
	{
		return 0.0f;
	}

	auto Result = 0.f;

	for (auto Index = PathPointIndex; Index < PathPointCosts.Num(); ++Index)
	{
		Result += PathPointCosts[Index];
	}

	return Result;
}
