#pragma once
#include "CoreMinimal.h"
#include "Components/PrimitiveComponent.h"
#include "Pathfinding/Core/Nav3DPathTest.h"
#include "Nav3DPathTestRenderingComponent.generated.h"

UCLASS()
class NAV3D_API UNav3DPathTestRenderingComponent final : public UPrimitiveComponent
{
    GENERATED_BODY()
public:
    UNav3DPathTestRenderingComponent();

    ANav3DPathTest* GetPathTest() const { return Cast<ANav3DPathTest>(GetOwner()); }
    virtual FPrimitiveSceneProxy* CreateSceneProxy() override;
    virtual FBoxSphereBounds CalcBounds(const FTransform& LocalToWorld) const override;
};


