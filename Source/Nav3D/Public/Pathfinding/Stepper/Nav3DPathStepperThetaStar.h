#pragma once
#include "Nav3DPathStepperAStar.h"
#include "Raycasting/Nav3DRaycaster.h"
#include "Nav3DPathStepperThetaStar.generated.h"

USTRUCT()
struct NAV3D_API FNav3DPathStepperThetaStarParameters
{
	GENERATED_BODY()

	FNav3DPathStepperThetaStarParameters();

	UPROPERTY(Instanced, EditAnywhere)
	UNav3DRaycaster* Raycaster;
};

class NAV3D_API FNav3DPathStepperThetaStar : public FNav3DPathStepperAStar
{
public:
	FNav3DPathStepperThetaStar(
		const FNav3DPathFindingParameters& Parameters,
		const FNav3DPathStepperThetaStarParameters& ThetaStarParameters);

protected:
	virtual ENav3DPathStepperStatus Init(EGraphAStarResult& Result) override;
	virtual ENav3DPathStepperStatus ProcessSingleNode(EGraphAStarResult& Result) override;
	virtual ENav3DPathStepperStatus ProcessNeighbour(EGraphAStarResult& Result) override;
	virtual ENav3DPathStepperStatus Ended(EGraphAStarResult& Result) override;
	bool HasLineOfSight(FNav3DNodeAddress From, FNav3DNodeAddress To) const;
	const FNav3DPathStepperThetaStarParameters& ThetaStarParameters;

private:
	int LOSCheckCount;
};
