#pragma once
#include "Nav3DPathFindingSearch.h"

class NAV3D_API FNav3DGraphAStar final : public FGraphAStar<FNav3DVolumeNavigationData>
{
public:
	explicit FNav3DGraphAStar(const FNav3DVolumeNavigationData& Graph);
};
