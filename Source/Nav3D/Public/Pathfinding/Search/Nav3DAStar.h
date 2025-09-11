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
};

class NAV3D_API FNav3DGraphAStar final : public FGraphAStar<FNav3DVolumeNavigationData>
{
public:
	FORCEINLINE explicit FNav3DGraphAStar(const FNav3DVolumeNavigationData& Graph)
		: FGraphAStar(Graph) {}
};
