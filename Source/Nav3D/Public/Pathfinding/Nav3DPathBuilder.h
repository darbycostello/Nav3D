#pragma once
#include "Nav3DPathFindingProcessor.h"

class NAV3D_API FNav3DPathBuilder final : public FNav3DPathFindingProcessor
{
public:
	FNav3DPathBuilder(FNav3DPath& NavigationPath,
	                  const FNav3DPathStepper& Stepper);

	virtual void ProcessFinalPath(
		const TArray<FNav3DPathFinderNodeAddress>& NodeAddresses) override;

private:
	FNav3DPath& NavigationPath;
};
