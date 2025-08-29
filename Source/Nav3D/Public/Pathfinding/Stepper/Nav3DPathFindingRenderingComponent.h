#pragma once
#include "Pathfinding/Nav3DPathFinderTest.h"
#include "Nav3DPathFindingRenderingComponent.generated.h"

UCLASS()
class NAV3D_API UNav3DPathFindingRenderingComponent final : public UPrimitiveComponent
{
	GENERATED_BODY()

public:
	UNav3DPathFindingRenderingComponent();

	ANav3DPathFinderTest* GetPathFinderTest() const;
	virtual FPrimitiveSceneProxy* CreateSceneProxy() override;

	virtual FBoxSphereBounds
	CalcBounds(const FTransform& LocalToWorld) const override;
};

FORCEINLINE ANav3DPathFinderTest* UNav3DPathFindingRenderingComponent::GetPathFinderTest() const
{
	return Cast<ANav3DPathFinderTest>(GetOwner());
}
