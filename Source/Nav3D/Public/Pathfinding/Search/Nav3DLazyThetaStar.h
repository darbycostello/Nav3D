#pragma once
#include "Nav3DThetaStar.h"
#include "Nav3DLazyThetaStar.generated.h"

UCLASS(Blueprintable)
class NAV3D_API UNav3DLazyThetaStar final : public UNav3DPathFindingSearch
{
	GENERATED_BODY()

public:
	virtual ENavigationQueryResult::Type
	GetPath(FNav3DPath& NavigationPath,
	        const FNav3DPathFindingParameters& Params) const override;
	virtual TSharedPtr<FNav3DPathStepper>
	GetDebugPathStepper(FNav3DPathFinderDebugData& DebugData,
	                    const FNav3DPathFindingParameters Params) const override;

private:
	UPROPERTY(EditAnywhere)
	FNav3DPathStepperThetaStarParameters ThetaStarParameters;
};
