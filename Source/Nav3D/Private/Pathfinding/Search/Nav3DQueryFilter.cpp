#include "Pathfinding/Search/Nav3DQueryFilter.h"

void FNav3DQueryFilter::Reset()
{
	// Not implemented - Nav3D uses voxel-based navigation instead of nav areas.
}

void FNav3DQueryFilter::SetAreaCost(uint8 AreaType, float Cost)
{
	// Not implemented - Nav3D uses voxel-based navigation instead of nav areas.
}

void FNav3DQueryFilter::SetFixedAreaEnteringCost(uint8 AreaType, float Cost)
{
	// Not implemented - Nav3D uses voxel-based navigation instead of nav areas.
}

void FNav3DQueryFilter::SetExcludedArea(uint8 AreaType)
{
	// Not implemented - Nav3D uses voxel-based navigation instead of nav areas.
}

void FNav3DQueryFilter::SetAllAreaCosts(const float* CostArray, const int32 Count)
{
	// Not implemented - Nav3D uses voxel-based navigation instead of nav areas.
}

void FNav3DQueryFilter::GetAllAreaCosts(float* CostArray, float* FixedCostArray, const int32 Count) const
{
	// Not implemented - Nav3D uses voxel-based navigation instead of nav areas.
}

void FNav3DQueryFilter::SetBacktrackingEnabled(const bool bBacktracking)
{
	// Not implemented - Nav3D uses voxel-based navigation instead of nav areas.
}

bool FNav3DQueryFilter::IsBacktrackingEnabled() const { return false; }

float FNav3DQueryFilter::GetHeuristicScale() const
{
	return QueryFilterSettings.HeuristicScale;
}

bool FNav3DQueryFilter::IsEqual(
	const INavigationQueryFilterInterface* Other) const
{
	return Other == this;
}

void FNav3DQueryFilter::SetIncludeFlags(uint16 Flags)
{
	// Not implemented - Nav3D uses voxel-based navigation instead of nav areas.
}

uint16 FNav3DQueryFilter::GetIncludeFlags() const { return 0; }

void FNav3DQueryFilter::SetExcludeFlags(uint16 Flags)
{
	// Not implemented - Nav3D uses voxel-based navigation instead of nav areas.
}

uint16 FNav3DQueryFilter::GetExcludeFlags() const { return 0; }

INavigationQueryFilterInterface* FNav3DQueryFilter::CreateCopy() const
{
	return new FNav3DQueryFilter(*this);
}
