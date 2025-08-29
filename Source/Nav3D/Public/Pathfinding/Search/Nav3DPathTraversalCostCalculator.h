#pragma once

#include "Nav3DPathTraversalCostCalculator.generated.h"

struct FNav3DNodeAddress;
class FNav3DVolumeNavigationData;

UCLASS(abstract, NotBlueprintable, EditInlineNew)
class NAV3D_API UNav3DPathTraversalCostCalculator : public UObject
{
	GENERATED_BODY()

public:
	virtual float GetTraversalCost(const FNav3DVolumeNavigationData& BoundsData,
	                               const FNav3DNodeAddress& Start,
	                               const FNav3DNodeAddress& End) const
	PURE_VIRTUAL(UNav3DPathCostCalculator::GetCost, return 0.0f;);
};

UCLASS()
class NAV3D_API UNav3DPathCostCalculator_Distance final
	: public UNav3DPathTraversalCostCalculator
{
	GENERATED_BODY()

public:
	virtual float GetTraversalCost(const FNav3DVolumeNavigationData& BoundsData,
	                               const FNav3DNodeAddress& Start,
	                               const FNav3DNodeAddress& End) const override;
};

/*
 * Applies a fixed cost to node traversal. Not recommended unless working with tile-based game logic.
 */
UCLASS()
class NAV3D_API UNav3DPathCostCalculator_Fixed final
	: public UNav3DPathTraversalCostCalculator
{
	GENERATED_BODY()

public:
	UNav3DPathCostCalculator_Fixed();

	virtual float GetTraversalCost(const FNav3DVolumeNavigationData& BoundsData,
	                               const FNav3DNodeAddress& Start,
	                               const FNav3DNodeAddress& End) const override;

private:
	UPROPERTY(EditDefaultsOnly)
	float Cost;
};
