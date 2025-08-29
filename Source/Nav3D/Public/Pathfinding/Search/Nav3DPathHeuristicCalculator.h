#pragma once

#include "Nav3DPathHeuristicCalculator.generated.h"

class FNav3DVolumeNavigationData;
struct FNav3DNodeAddress;

UCLASS(abstract, NotBlueprintable, EditInlineNew)
class NAV3D_API UNav3DPathHeuristicCalculator : public UObject
{
	GENERATED_BODY()

public:
	virtual float GetHeuristicCost(const FNav3DVolumeNavigationData& BoundsData,
	                               const FNav3DNodeAddress& Start,
	                               const FNav3DNodeAddress& End) const
	PURE_VIRTUAL(UNav3DPathHeuristicCalculator::GetHeuristicCost,
	             return 0.0f;);
};

UCLASS()
class NAV3D_API UNav3DPathHeuristicCalculator_Manhattan final
	: public UNav3DPathHeuristicCalculator
{
	GENERATED_BODY()

public:
	virtual float GetHeuristicCost(const FNav3DVolumeNavigationData& BoundsData,
	                               const FNav3DNodeAddress& Start,
	                               const FNav3DNodeAddress& End) const override;
};

UCLASS()
class NAV3D_API UNav3DPathHeuristicCalculator_Euclidean final
	: public UNav3DPathHeuristicCalculator
{
	GENERATED_BODY()

public:
	virtual float GetHeuristicCost(const FNav3DVolumeNavigationData& BoundsData,
	                               const FNav3DNodeAddress& Start,
	                               const FNav3DNodeAddress& End) const override;
};
