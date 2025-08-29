#pragma once
#include "Pathfinding/Nav3DPathFindingTypes.h"
#include "Pathfinding/Search/Nav3DPathFindingSearch.h"
#include "Nav3DPathStepperThetaStar.h"

class NAV3D_API FNav3DPathStepperLazyThetaStar final : public FNav3DPathStepperThetaStar
{
public:
	FNav3DPathStepperLazyThetaStar(
		const FNav3DPathFindingParameters& Parameters,
		const FNav3DPathStepperThetaStarParameters& ThetaStarParameters);

protected:
	virtual ENav3DPathStepperStatus ProcessSingleNode(EGraphAStarResult& Result) override;
	virtual ENav3DPathStepperStatus ProcessNeighbour(EGraphAStarResult& Result) override;
};
