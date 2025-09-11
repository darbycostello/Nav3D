#pragma once

#include "Pathfinding/Stepper/Nav3DPathStepperAStar.h"
#include "Pathfinding/Stepper/Nav3DPathStepperThetaStar.h"
#include "Nav3DThetaStar.generated.h"

class UNav3DRaycaster;

UCLASS(Blueprintable)
class NAV3D_API UNav3DThetaStar final : public UNav3DPathFindingSearch
{
	GENERATED_BODY()

public:
	virtual ENavigationQueryResult::Type
	GetPath(FNav3DPath& NavigationPath,
	        const FNav3DPathFindingParameters& Params) const override;

private:
	UPROPERTY(EditAnywhere)
	FNav3DPathStepperThetaStarParameters ThetaStarParameters;
};
