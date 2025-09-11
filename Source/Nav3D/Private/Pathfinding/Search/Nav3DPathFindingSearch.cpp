#include "Pathfinding/Search/Nav3DPathFindingSearch.h"

ENavigationQueryResult::Type UNav3DPathFindingSearch::GetPath(
	FNav3DPath& /*NavigationPath*/,
	const FNav3DPathFindingParameters& /*Params*/) const
{
	return ENavigationQueryResult::Error;
}