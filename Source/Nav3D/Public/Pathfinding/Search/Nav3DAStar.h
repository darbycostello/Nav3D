// ReSharper disable CppUE4CodingStandardNamingViolationWarning
#pragma once
#include "Nav3DPathFindingSearch.h"
#include "Nav3DAStar.generated.h"

UCLASS()
class NAV3D_API UNav3DAStar final : public UNav3DPathFindingSearch
{
	GENERATED_BODY()

public:
	virtual ENavigationQueryResult::Type
	GetPath(FNav3DPath& NavigationPath, const FNav3DPathFindingParameters& Params) const override;
	virtual TSharedPtr<FNav3DPathStepper>
	GetDebugPathStepper(FNav3DPathFinderDebugData& DebugData,
	                    const FNav3DPathFindingParameters Params) const override;
};
